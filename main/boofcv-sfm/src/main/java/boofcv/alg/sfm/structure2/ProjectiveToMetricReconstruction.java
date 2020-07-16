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
import boofcv.abst.geo.bundle.SceneObservations;
import boofcv.abst.geo.bundle.SceneStructureMetric;
import boofcv.alg.geo.GeometricResult;
import boofcv.alg.geo.MultiViewOps;
import boofcv.alg.geo.selfcalib.SelfCalibrationLinearDualQuadratic;
import boofcv.factory.geo.ConfigTriangulation;
import boofcv.factory.geo.FactoryMultiView;
import boofcv.struct.image.ImageDimension;
import georegression.struct.point.Point2D_F64;
import georegression.struct.se.Se3_F64;
import org.ejml.data.DMatrixRMaj;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;

import static boofcv.misc.BoofMiscOps.assertBoof;

/**
 * Upgrades a projective reconstruction into a metric reconstruction.
 *
 * @author Peter Abeles
 */
public class ProjectiveToMetricReconstruction {

	ConfigProjectiveToMetric config;

	// Bundle adjustment data structure and tuning parameters
	public SceneStructureMetric structure;
	public SceneObservations observations;

	PrintStream verbose;

	//------------------------- Workspace Variables
	// list of workGraph views
	List<SceneWorkingGraph.View> workViews;

	public boolean process( LookupSimilarImages db , PairwiseImageGraph2 imageGraph, SceneWorkingGraph sceneGraph )
	{
		workViews = new ArrayList<>(sceneGraph.getAllViews());

		// Self calibration and upgrade views
		if (!upgradeViewsToMetric(db))
			return false;

		// Compute 3D feature locations and prepare for bundle adjustment
//		sceneGraph.createFeaturesFromInliers();
//		triangulateFeatures();
		if( !buildMetricSceneForBundleAdjustment())
			return false;

		// Refine the initial estimate using bundle adjustment

		return true;
	}

	private boolean upgradeViewsToMetric(LookupSimilarImages db) {
		// TODO make sure these have zero center
		// Perform self calibration using all the found camera matrices
		SelfCalibrationLinearDualQuadratic selfcalib = new SelfCalibrationLinearDualQuadratic(config.aspectRatio);
		workViews.forEach(o->selfcalib.addCameraMatrix(o.projective));

		GeometricResult result = selfcalib.solve();
		if( result != GeometricResult.SUCCESS ) {
			if( verbose != null ) verbose.println("Self calibration failed. "+result);
			return false;
		}
		// homography to go from projective to metric
		DMatrixRMaj H = new DMatrixRMaj(4,4);
		// convert camera matrix from projective to metric
		if( !MultiViewOps.absoluteQuadraticToH(selfcalib.getQ(),H) ) {
			if( verbose != null ) verbose.println("Projective to metric failed to compute H");
			return false;
		}

		// Save the upgraded metric calibration for each camera
		ImageDimension shape = new ImageDimension();
		DMatrixRMaj K = new DMatrixRMaj(3,3);
		List<SelfCalibrationLinearDualQuadratic.Intrinsic> solutions = selfcalib.getSolutions();
		for (int viewIdx = 0; viewIdx < workViews.size(); viewIdx++) {
			SelfCalibrationLinearDualQuadratic.Intrinsic intrinsic = solutions.get(viewIdx);
			SceneWorkingGraph.View wv = workViews.get(viewIdx);
			db.lookupShape(wv.pview.id,shape);
			wv.pinhole.fsetK(intrinsic.fx,intrinsic.fy,intrinsic.skew,0,0,shape.width,shape.height);

			// ignore K since we already have that
			MultiViewOps.projectiveToMetric(wv.projective,H,wv.world_to_view,K);
		}

		// scale is arbitrary. Set max translation to 1. This should be better numerically
		double maxT = 0;
		for (int viewIdx = 0; viewIdx < workViews.size(); viewIdx++) {
			SceneWorkingGraph.View wv = workViews.get(viewIdx);
			maxT = Math.max(maxT,wv.world_to_view.T.norm());
		}
		for (int viewIdx = 0; viewIdx < workViews.size(); viewIdx++) {
			SceneWorkingGraph.View wv = workViews.get(viewIdx);
			wv.world_to_view.T.scale(1.0/maxT);
		}
		return true;
	}

	private void triangulateFeatures(SceneWorkingGraph sceneGraph) {
		TriangulateNViewsMetric triangulator = FactoryMultiView.
				triangulateNViewCalibrated(ConfigTriangulation.GEOMETRIC());

		List<Point2D_F64> observations = new ArrayList<>();
		List<Se3_F64> world_to_view = new ArrayList<>();

		for (int featIter = 0; featIter < sceneGraph.features.size(); featIter++) {
			SceneWorkingGraph.Feature f = sceneGraph.features.get(featIter);
			assertBoof(f.visible.size()>=2);

			observations.clear();
			world_to_view.clear();

			for (int visIter = 0; visIter < f.visible.size(); visIter++) {
				SceneWorkingGraph.Observation o = f.visible.get(visIter);
			}

//			triangulator.triangulate();
		}

		for (int viewIdx = 0; viewIdx < workViews.size(); viewIdx++) {
			SceneWorkingGraph.View wv = workViews.get(viewIdx);
			if( wv.projectiveInliers.isEmpty() )
				continue;



		}
		// TODO go through each view. See which features were used as an inlier when computing projective stuff

		// TODO for each feature create a list of views it appears in
	}

	private boolean buildMetricSceneForBundleAdjustment() {
//		final int numViews = workViews.size();
//
//		// Construct bundle adjustment data structure
//		structure = new SceneStructureMetric(false);
//		structure.initialize(3,3,inliers.size());
//		observations = new SceneObservations();
//		observations.initialize(3);
//
//		for (int i = 0; i < listPinhole.size(); i++) {
//			CameraPinhole cp = listPinhole.get(i);
//			BundlePinholeSimplified bp = new BundlePinholeSimplified();
//
//			bp.f = cp.fx;
//
//			structure.setCamera(i,false,bp);
//			structure.setView(i,i==0,worldToView.get(i));
//			structure.connectViewToCamera(i,i);
//		}
//		for (int i = 0; i < inliers.size(); i++) {
//			AssociatedTriple t = inliers.get(i);
//
//			observations.getView(0).add(i,(float)t.p1.x,(float)t.p1.y);
//			observations.getView(1).add(i,(float)t.p2.x,(float)t.p2.y);
//			observations.getView(2).add(i,(float)t.p3.x,(float)t.p3.y);
//
//			structure.connectPointToView(i,0);
//			structure.connectPointToView(i,1);
//			structure.connectPointToView(i,2);
//		}
//		// Initial estimate for point 3D locations
//		triangulatePoints(structure,observations);
		return true;
	}
}
