/*
 * Copyright (c) 2011-2020, Peter Abeles. All Rights Reserved.
 *
 * This file is part of BoofCV (http://boofcv.org).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package boofcv.alg.sfm.structure2;

import boofcv.abst.geo.TriangulateNViewsMetric;
import boofcv.struct.calib.CameraPinhole;
import boofcv.struct.feature.AssociatedIndex;
import boofcv.struct.feature.AssociatedTripleIndex;
import boofcv.struct.image.ImageDimension;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;
import gnu.trove.map.hash.TIntObjectHashMap;
import org.ddogleg.struct.FastArray;
import org.ddogleg.struct.FastQueue;
import org.ddogleg.struct.GrowQueue_B;
import org.ddogleg.struct.GrowQueue_I32;
import org.ejml.data.DMatrixRMaj;

import java.util.*;

import static boofcv.misc.BoofMiscOps.assertBoof;

/**
 * @author Peter Abeles
 */
public class SceneWorkingGraph {

	public final Map<String,View> views = new HashMap<>();
	public final List<Feature> features = new ArrayList<>();
	public final List<Observation> observations = new ArrayList<>();

	//--------------- Internal Work Space
	private final FastQueue<ImageInfo> listInfo = new FastQueue<>(ImageInfo::new,ImageInfo::reset);
	private final FastQueue<Point2D_F64> triangulateObs = new FastQueue<>(Point2D_F64::new);
	private final FastQueue<Se3_F64> triangulateViews = new FastQueue<>(Se3_F64::new);

	public void reset() {
		views.clear();
		features.clear();
		observations.clear();
	}

	public View lookupView( String id ) {
		return views.get(id);
	}


	public boolean isKnown( PairwiseImageGraph2.View pview ) {
		return views.containsKey(pview.id);
	}

	public Feature lookupFeature( PairwiseImageGraph2.View pview , int viewIdx ) {
		View v = views.get(pview.id);
		if( v == null )
			return null;
		return v.v2g.get(viewIdx);
	}

	public View addView( PairwiseImageGraph2.View pview ) {
		View v = new View();
		v.pview = pview;
		views.put(v.pview.id,v);
		return v;
	}

	/**
	 * Using the list of inliers, create a set of features for the entire scene.
	 */
	public void createFeaturesFromInliers(LookupSimilarImages db, TriangulateNViewsMetric triangulator) {
		features.clear();

		GrowQueue_B is_obs_inlier = new GrowQueue_B();
		FastArray<Feature> inlier_to_feature = new FastArray<>(Feature.class);

		for( View target_v : views.values() ) {
			// if there are no inliers saved with this view skip it.
			if( target_v.projectiveInliers.isEmpty() )
				continue;
			// quick sanity check to see if the data structure fulfills its contract
			assertBoof(target_v.projectiveInliers.views.get(0)==target_v.pview);

			// create a look up table so it's inexpensive to see if a feature as an inlier
			is_obs_inlier.resize(target_v.pview.totalObservations,false);
			final InlierInfo inliers = target_v.projectiveInliers;
			final int numInliers = inliers.observations.size;
			for (int i = 0; i < numInliers; i++) {
				is_obs_inlier.data[inliers.observations.get(0).get(i)] = true;
			}

			// Create the look up table for observation to 3D feature
			inlier_to_feature.resize(numInliers,null);

			// Look up the observations for all views in the inlier set
			listInfo.reset(); // todo move to inlier info?
			for (int viewsIter = 0; viewsIter < inliers.views.size; viewsIter++) {
				PairwiseImageGraph2.View v = inliers.views.get(viewsIter);
				ImageInfo info = listInfo.grow();
				db.lookupShape(v.id,info.dimension);
				db.lookupPixelFeats(v.id,info.pixels);
			}

			// Go through each feature in the inlier set and either create a new feature, add new observation, or merge
			// together
			for (int connIdx = 0; connIdx < target_v.pview.connections.size; connIdx++) {
				PairwiseImageGraph2.Motion m = target_v.pview.connections.get(connIdx);
				PairwiseImageGraph2.View other_v = m.other(target_v.pview);
				boolean isSrc = m.src == target_v.pview;
				// go through all the two-view inliers. Not to be confused with projective inliers
				for (int featIter = 0; featIter < m.inliers.size; featIter++) { // TODO ugh might not be iterating correctly here
					// get the feature index in the wv.pview frame
					AssociatedIndex a = m.inliers.data[featIter];
					int obsIdx = isSrc ? a.src : a.dst;
					// see if it's a projective inlier
					if( !is_obs_inlier.data[obsIdx])
						continue;

					// see if this observation has already been assigned a feature from the target view
					Feature tgt_f = inlier_to_feature.data[obsIdx];
					// see if a feature has already been assigned to an observation in the other view
					Feature oth_f = lookupFeature(other_v,isSrc ? a.dst : a.src);

					if( tgt_f == null ) {
						if( oth_f == null ) {
							// it is unknown to both views and a new feature should be created
//							triangulateFeature(inliers,inlier_to_feature.get(obsIdx));

						} else {
							// the feature is known to this other but not target view
							// TODO add observation from target view to feature
							inlier_to_feature.data[obsIdx] = tgt_f = oth_f;
						}
					} else if( oth_f == null ) {
						// TODO add the other view to this feature
					} else if( oth_f != tgt_f ) {
						// todo merge these two features together to create a single feature
					}
				}
			}
		}
	}

	/**
	 * Performs triangulation on the specified feature
	 *
	 * @param info (Input) Information on the inlier set
	 * @param featureIdx (Input) index of the feature in the inlier set which is to be triangulated
	 * @param triangulator (Input) Performs the triangulation
	 * @param X (Outut) storage for the triangulated feature in world coordinates
	 * @return true if succesful
	 */
	private boolean triangulateFeature(InlierInfo info, int featureIdx, TriangulateNViewsMetric triangulator, Point3D_F64 X)
	{
		assertBoof(info.observations.size==info.views.size);
		int numViews = info.views.size;
		
		// For numerical reasons, triangulate in the reference frame of the first view
		Se3_F64 world_to_origin = views.get(info.views.get(0).id).world_to_view;
		Se3_F64 origin_to_world = world_to_origin.invert(null);
		
		triangulateObs.reset();
		triangulateViews.reset();
		for (int viewIter = 0; viewIter < numViews; viewIter++) {
			// get the index of the observation for this feature in this view
			int observationIdx = info.observations.get(numViews).get(featureIdx);
			// look up the value of the observation and save it
			Point2D_F64 observationPixel = listInfo.get(viewIter).pixels.get(observationIdx);
			triangulateObs.grow().set(observationPixel);

			// compute origin_to_viewI
			Se3_F64 world_to_viewI = views.get(info.views.get(viewIter).id).world_to_view;
			origin_to_world.concat(world_to_viewI,triangulateViews.grow());
		}

		if( !triangulator.triangulate(triangulateObs.toList(), triangulateViews.toList(),X) )
			return false;

		// convert it back into the world frame
		SePointOps_F64.transform(origin_to_world,X,X);

		return true;
	}

	private void addLocalFeatures(PairwiseImageGraph2.View pview, View viewA, int[] l2g) {
		for (int viewIdx = 0; viewIdx < pview.totalObservations; viewIdx++) {
			// There was a contradiction and the feature should be skipped
			if( l2g[viewIdx] == -2 )
				continue;

			Observation o = createObservation();
			o.view = viewA;
			o.viewIdx = viewIdx;
			Feature f;
			if( l2g[viewIdx] == -1 ) {
				// The feature is unknown, so create a new feature
				f = createFeature();

			} else {
				// The view is known, so add this view to the feature
				f = features.get(l2g[viewIdx] );
			}
			f.visible.add(o);
			viewA.v2g.put(viewIdx,f);
		}
	}

	public Feature createFeature() {
		Feature f = new Feature();
		f.reset();
		features.add(f);
		return f;
	}

	private Observation createObservation() {
		Observation o = new Observation();
		observations.add(o);
		return o;
	}

	/**
	 *
	 * @param feature The feature being observed
	 * @param view The view it was observed in
	 * @param index The index of the feature in the view
	 */
	public void addObservation( Feature feature , View view , int index ) {
		Observation o = createObservation();
		o.view = view;
		o.viewIdx = index;
		feature.visible.add(o);
	}

	public void lookupCommon(String viewA, String viewB, String viewC,
							 List<Feature> features ,
							 FastQueue<AssociatedTripleIndex> matches ) {

	}

	public Collection<View> getAllViews() {
		return views.values();
	}

	public class Feature {
		// which views it's visible in
		public final List<Observation> visible = new ArrayList<>();

		public void reset() {
			visible.clear();
		}
	}

	public class Observation {
		public View view;
		// index of the feature in the view
		public int viewIdx;

		public void reset() {
			this.view = null;
			this.viewIdx = -1;
		}
	}

	/**
	 * Information on the set of inlier observations used to compute the camera location
	 */
	public static class InlierInfo
	{
		// List of views from which these inliers were selected from
		// the first view is always the view which contains this set of info
		public final FastArray<PairwiseImageGraph2.View> views = new FastArray<>(PairwiseImageGraph2.View.class);
		// indexes of observations for each view listed in 'views'.
		// All the views for a single feature will have the same index
		public final FastQueue<GrowQueue_I32> observations = new FastQueue<>(GrowQueue_I32::new);

		public boolean isEmpty() {
			return observations.size==0;
		}

		public void reset() {
			views.reset();
			observations.reset();
		}
	}

	public class View {
		public PairwiseImageGraph2.View pview;
		// view to global
		// provides a way to lookup features given their ID in the view
		public final TIntObjectHashMap<Feature> v2g = new TIntObjectHashMap<>();

		// Specifies which observations were used to compute the projective transform for this view
		// If empty that means one set of inliers are used to multiple views and only one view needed this to be saved
		// this happens for the seed view
		public final InlierInfo projectiveInliers = new InlierInfo();

		// projective camera matrix
		public final DMatrixRMaj projective = new DMatrixRMaj(3,4);
		// metric camera
		public final CameraPinhole pinhole = new CameraPinhole();
		public final Se3_F64 world_to_view = new Se3_F64();

		public void reset() {
			pview = null;
			v2g.clear();
			projective.zero();
			pinhole.reset();
			projectiveInliers.reset();
		}
	}

	private static class ImageInfo
	{
		final ImageDimension dimension = new ImageDimension();
		final FastQueue<Point2D_F64> pixels = new FastQueue<>(Point2D_F64::new,p->p.set(-1,-1));

		public void reset() {
			dimension.set(0,0);
			pixels.reset();
		}
	}
}
