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

package boofcv.alg.sfm.structure;

import boofcv.abst.feature.associate.AssociateDescription;
import boofcv.abst.feature.detdesc.DetectDescribePoint;
import boofcv.alg.geo.robust.ModelMatcherMultiview;
import boofcv.factory.geo.ConfigEssential;
import boofcv.factory.geo.ConfigFundamental;
import boofcv.factory.geo.ConfigRansac;
import boofcv.factory.geo.FactoryMultiViewRobust;
import boofcv.struct.calib.CameraPinhole;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.feature.AssociatedIndex;
import boofcv.struct.feature.TupleDesc;
import boofcv.struct.geo.AssociatedPair;
import boofcv.struct.image.ImageBase;
import georegression.struct.point.Point2D_F64;
import org.ddogleg.fitting.modelset.ModelMatcher;
import org.ddogleg.struct.FastAccess;
import org.ddogleg.struct.FastQueue;
import org.ddogleg.struct.Stoppable;
import org.ejml.data.DMatrixRMaj;

import java.io.PrintStream;
import java.util.List;

/**
 * Determines connectivity between images by exhaustively considering all possible combination of views. Assocation
 * is found by detecting features inside of each image.
 *
 * @author Peter Abeles
 */
public class PairwiseImageMatching<T extends ImageBase<T>>
	implements Stoppable
{
	// Used to pre-maturely stop the scene estimation process
	private volatile boolean stopRequested = false;

	protected double MIN_ASSOCIATE_FRACTION = 0.05;
	protected int MIN_FEATURE_ASSOCIATED = 30;

	protected DetectDescribePoint<T,TupleDesc> detDesc;
	protected AssociateDescription<TupleDesc> associate;

	// Graph describing the relationship between images
	protected PairwiseImageGraph graph = new PairwiseImageGraph();

	protected ConfigRansac configRansac = new ConfigRansac();
	protected ConfigEssential configEssential = new ConfigEssential();
	protected ConfigFundamental configFundamental = new ConfigFundamental();

	// Temporary storage for feature pairs which are inliers
	protected FastQueue<AssociatedPair> pairs = new FastQueue<>(AssociatedPair::new);

	protected ModelMatcherMultiview<DMatrixRMaj,AssociatedPair> ransacEssential;
	protected ModelMatcher<DMatrixRMaj,AssociatedPair> ransacFundamental;

	// print is verbose or not
	protected PrintStream verbose;
	protected int verboseLevel;

	public PairwiseImageMatching(DetectDescribePoint<T, TupleDesc> detDesc,
								 AssociateDescription<TupleDesc> associate ) {
		this();
		this.detDesc = detDesc;
		this.associate = associate;
	}

	protected PairwiseImageMatching(){
		configRansac.inlierThreshold = 2.5;
		configRansac.iterations = 4000;
	}

	protected void declareModelFitting() {
		ransacEssential = FactoryMultiViewRobust.essentialRansac(configEssential, configRansac);
		ransacFundamental = FactoryMultiViewRobust.fundamentalRansac(configFundamental, configRansac);
	}

	/**
	 * Specifies magic numbers for pruning connections
	 *
	 * @param minFeatureAssociate Minimum number of features a connection needs to have
	 * @param minFeatureAssociateFraction Fraction of total features for edge and both images.
	 */
	public void configure( int minFeatureAssociate , double minFeatureAssociateFraction ) {
		this.MIN_ASSOCIATE_FRACTION = minFeatureAssociateFraction;
		this.MIN_FEATURE_ASSOCIATED = minFeatureAssociate;
	}

	public void addCamera( String cameraName  ) {
		addCamera(cameraName,null,null);
	}
	public void addCamera( String cameraName , Point2Transform2_F64 pixelToNorm , CameraPinhole pinhole ) {
		graph.cameras.put( cameraName, new PairwiseImageGraph.Camera(cameraName,pixelToNorm,pinhole));
	}

	/**
	 * Adds a new observation from a camera. Detects features inside the and saves those.
	 *
	 * @param image The image
	 */
	public void addImage(T image , String cameraName ) {

		PairwiseImageGraph.View view = new PairwiseImageGraph.View(graph.nodes.size(),
				new FastQueue<>(detDesc::createDescription));

		view.camera = graph.cameras.get(cameraName);
		if( view.camera == null )
			throw new IllegalArgumentException("Must have added the camera first");

		view.index = graph.nodes.size();
		graph.nodes.add(view);

		detDesc.detect(image);

		// Pre-declare memory
		view.descriptions.growArray(detDesc.getNumberOfFeatures());
		view.observationPixels.growArray(detDesc.getNumberOfFeatures());

		for (int i = 0; i < detDesc.getNumberOfFeatures(); i++) {
			Point2D_F64 p = detDesc.getLocation(i);

			// save copies since detDesc recycles memory
			view.descriptions.grow().setTo(detDesc.getDescription(i));
			view.observationPixels.grow().set(p);
		}

		if( view.camera.pixelToNorm == null ){
			return;
		}

		view.observationNorm.growArray(detDesc.getNumberOfFeatures());
		for (int i = 0; i < view.observationPixels.size; i++) {
			Point2D_F64 p = view.observationPixels.get(i);
			view.camera.pixelToNorm.compute(p.x,p.y,view.observationNorm.grow());
		}

		if( verbose != null ) {
			verbose.println("Detected Features: "+detDesc.getNumberOfFeatures());
		}
	}


	/**
	 * Determines connectivity between images. Results can be found by calling {@link #getGraph()}.
	 * @return true if successful or false if it failed
	 */
	public boolean process() {
		if( graph.nodes.size() < 2 )
			return false;
		stopRequested = false;

		declareModelFitting();

		for (int i = 0; i < graph.nodes.size(); i++) {
			if( verbose != null )
				verbose.print("Matching node "+i+" -> ");
			associate.setSource(graph.nodes.get(i).descriptions);
			for (int j = i+1; j < graph.nodes.size(); j++) {
				associate.setDestination(graph.nodes.get(j).descriptions);
				associate.associate();
				if( associate.getMatches().size < MIN_FEATURE_ASSOCIATED )
					continue;

				boolean connected = connectViews(graph.nodes.get(i),graph.nodes.get(j),associate.getMatches());
				if( verbose != null ) {
					if( connected )
						verbose.print("+");
					else
						verbose.print("-");
				}

				if( stopRequested )
					return false;
			}
			if( verbose != null ) {
				verbose.println();
			}
		}
		return graph.edges.size() >= 1;
	}

	/**
	 * Returns the found graph
	 */
	public PairwiseImageGraph getGraph() {
		return graph;
	}

	/**
	 * Associate features between the two views. Then compute a homography and essential matrix using LSMed. Add
	 * features to the edge if they an inlier in essential. Save fit score of homography vs essential.
	 */
	protected boolean connectViews(PairwiseImageGraph.View viewA , PairwiseImageGraph.View viewB ,
								   FastAccess<AssociatedIndex> matches) {

		// Estimate fundamental/essential with RANSAC
		PairwiseImageGraph.Motion edge = new PairwiseImageGraph.Motion();
		int inliersEpipolar;

		CameraPinhole pinhole0 = viewA.camera.pinhole;
		CameraPinhole pinhole1 = viewB.camera.pinhole;

		if( pinhole0 != null && pinhole1 != null ) {
			// Fully calibrated camera pair
			ransacEssential.setIntrinsic(0,pinhole0);
			ransacEssential.setIntrinsic(1,pinhole1);

			if( !fitEpipolar(matches, viewA.observationNorm.toList(), viewB.observationNorm.toList(),ransacEssential,edge) ) {
				if( verbose != null && verboseLevel >= 1 ) {
					verbose.println(" fit essential failed");
				}
				return false;
			}
			edge.metric = true;
			inliersEpipolar = ransacEssential.getMatchSet().size();
			edge.F.set(ransacEssential.getModelParameters());
		} else if( fitEpipolar(matches,
					viewA.observationPixels.toList(), viewB.observationPixels.toList(),
					ransacFundamental,edge) ) {
			// transform is only known up to a projective transform
			edge.metric = false;
			inliersEpipolar = ransacFundamental.getMatchSet().size();
			edge.F.set(ransacFundamental.getModelParameters());
		} else {
			if( verbose != null && verboseLevel >= 1 ) {
				verbose.println(" fit fundamental failed");
			}
			return false;
		}

		if( inliersEpipolar < MIN_FEATURE_ASSOCIATED ) {
			if( verbose != null && verboseLevel >= 1 ) {
				verbose.println(" too too few inliers. "+inliersEpipolar+" min="+MIN_FEATURE_ASSOCIATED+
						" obsA="+viewA.observationNorm.size+" obsB="+viewB.observationNorm.size);
			}
			return false;
		}

		// If only a very small number of features are associated do not consider the view
		double fractionA = inliersEpipolar/(double)viewA.descriptions.size;
		double fractionB = inliersEpipolar/(double)viewB.descriptions.size;

		if( fractionA < MIN_ASSOCIATE_FRACTION | fractionB < MIN_ASSOCIATE_FRACTION )
			return false;

		// If the geometry is good for triangulation this number will be lower
		edge.viewSrc = viewA;
		edge.viewDst = viewB;
		edge.index = graph.edges.size();
		viewA.connections.add(edge);
		viewB.connections.add(edge);
		graph.edges.add(edge);

		return true;
	}

	/**
	 * Uses ransac to fit an epipolar model to the associated features. Adds list of matched features to the edge.
	 *
	 * @param matches List of matched features by index
	 * @param pointsA Set of observations from image A
	 * @param pointsB Set of observations from image B
	 * @param ransac Model fitter
	 * @param edge Edge which will contain a description of found motion
	 * @return true if no error
	 */
	boolean fitEpipolar(FastAccess<AssociatedIndex> matches ,
						List<Point2D_F64> pointsA , List<Point2D_F64> pointsB ,
						ModelMatcher<?,AssociatedPair> ransac ,
						PairwiseImageGraph.Motion edge )
	{
		pairs.resize(matches.size);
		for (int i = 0; i < matches.size; i++) {
			AssociatedIndex a = matches.get(i);
			pairs.get(i).p1.set(pointsA.get(a.src));
			pairs.get(i).p2.set(pointsB.get(a.dst));
		}
		if( !ransac.process(pairs.toList()) )
			return false;
		int N = ransac.getMatchSet().size();
		for (int i = 0; i < N; i++) {
			AssociatedIndex a = matches.get(ransac.getInputIndex(i));
			edge.associated.add( a.copy() );
		}
		return true;
	}

	@Override
	public void requestStop() {
		stopRequested = true;
	}

	@Override
	public boolean isStopRequested() {
		return stopRequested;
	}

	public Class<TupleDesc> getDescriptionType() {
		return detDesc.getDescriptionType();
	}

	public ConfigEssential getConfigEssential() {
		return configEssential;
	}

	public ConfigFundamental getConfigFundamental() {
		return configFundamental;
	}

	public ConfigRansac getConfigRansac() {
		return configRansac;
	}

	public void reset() {
		graph = new PairwiseImageGraph();
	}

	public void setVerbose(PrintStream verbose , int level ) {
		this.verbose = verbose;
		this.verboseLevel = level;
	}
}
