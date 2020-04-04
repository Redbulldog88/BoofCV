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

package boofcv.abst.feature.describe;

import boofcv.alg.feature.describe.DescribePointSift;
import boofcv.struct.Configuration;

/**
 * Configuration for {@link DescribePointSift}
 * 
 * @author Peter Abeles
 */
public class ConfigSiftDescribe implements Configuration {

	/** Width of sub-region in samples.*/
	public int widthSubregion=4;

	/** Width of grid in subregions. */
	public int widthGrid=4;

	/** Number of histogram bins.*/
	public int numHistogramBins=8;

	/** Conversion of sigma to pixels.  Used to scale the descriptor region. */
	public double sigmaToPixels=1.0;

	/** Sigma for Gaussian weighting function is set to this value * region width.*/
	public double weightingSigmaFraction=0.5;

	/** Maximum fraction a single element can have in descriptor. Helps with non-affine changes in lighting. See paper.*/
	public double maxDescriptorElementValue=0.2;

	@Override
	public void checkValidity() {
		
	}
}
