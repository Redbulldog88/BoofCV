/*
 * Copyright (c) 2011-2019, Peter Abeles. All Rights Reserved.
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

package boofcv.alg.misc.impl;

import boofcv.struct.image.*;

import boofcv.concurrency.BoofConcurrency;
import org.ddogleg.sorting.QuickSelect;
import javax.annotation.Generated;
import boofcv.alg.misc.ImageMiscOps;

/**
 * Implementation of algorithms in ImageBandMath
 *
 * <p>DO NOT MODIFY: Generated by boofcv.alg.misc.impl.GenerateImplImageBandMath.</p>
 *
 * @author Nico Stuurman
 * @author Peter Abeles
 */
@Generated("boofcv.alg.misc.impl.GenerateImplImageBandMath")
@SuppressWarnings("Duplicates")
public class ImplImageBandMath_MT {

	public static void minimum(Planar<GrayU8> input , GrayU8 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayU8[] bands = input.bands;
		
		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput+w;
			// for(int x = 0; x < w; x++ ) {
			for (; indexInput < indexEnd; indexInput++, indexOutput++ ) {
				int minimum = Integer.MAX_VALUE;
				for( int i = startBand; i <= lastBand; i++ ) {
					int value = bands[i].data[ indexInput ] & 0xFF;
					if ( value < minimum) {
						minimum = value;
					}
				}
				output.data[indexOutput] = (byte) minimum;
			}
		});
	}

	public static void maximum(Planar<GrayU8> input , GrayU8 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayU8[] bands = input.bands;

		BoofConcurrency.range(0,h,y->{
			int indexInput = input.startIndex + y * input.stride;
			int indexOutput = output.startIndex + y * output.stride;

			int indexEnd = indexInput + w;
			// for(int x = 0; x < w; x++ ) {
			for(; indexInput < indexEnd; indexInput++, indexOutput++) {
				int maximum = -Integer.MAX_VALUE;
				for (int i = startBand; i <= lastBand; i++) {
					int value = bands[i].data[ indexInput ] & 0xFF;
					if( value > maximum) {
						maximum = value;
					}
				}
				output.data[indexOutput] = (byte) maximum;
			} 
		});
	}

	public static void median(Planar<GrayU8> input , GrayU8 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayU8[] bands = input.bands;
		final int numBands = lastBand - startBand + 1;

		// handle edge case
		if( numBands == 1 ) {
			ImageMiscOps.copy(0, 0, 0, 0, input.getWidth(), input.getHeight(), input.getBand(startBand), output);
			return;
		}

		final int middle = numBands/2;
		boolean isEven = numBands % 2 == 0;
		BoofConcurrency.range(0,h,y->{
			final int[] valueArray = new int[numBands];
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput+w;
			for (; indexInput < indexEnd; indexInput++, indexOutput++ ) {
				for( int i = startBand; i <= lastBand; i++ ) {
					valueArray[i] = bands[i].data[indexInput]& 0xFF;
				}
				if (isEven) {
					// Would a single quick sort be faster?
					int val0 = QuickSelect.select(valueArray, middle, numBands);
					int val1 = QuickSelect.select(valueArray, middle+1, numBands);
					output.data[indexOutput] = (byte) ((val0+val1)/2);
				} else {
					output.data[indexOutput] = (byte)QuickSelect.select(valueArray, middle, numBands);
				}
			}
		});
	}

	public static void average(Planar<GrayU8> input , GrayU8 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayU8[] bands = input.bands;
		int divisor = lastBand - startBand+1;
		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput + w;
			for (; indexInput < indexEnd; indexInput++, indexOutput++) {
			int sum = bands[startBand].data[indexInput] & 0xFF;
				for (int i = startBand+1; i <= lastBand; i++) {
					sum += bands[i].data[indexInput] & 0xFF;
				}
				output.data[indexOutput] = (byte) (sum/divisor);
			}
		});
	}

	public static void stdDev(Planar<GrayU8> input , GrayU8 output , GrayU8 avg, int startBand, int lastBand ) {

		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayU8[] bands = input.bands;
		int divisor = lastBand - startBand;

		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput + w;

			for (; indexInput < indexEnd; indexInput++, indexOutput++) {
				int sum = 0;
				for (int i = startBand; i <= lastBand; i++) {
					int diff = (bands[i].data[indexInput] & 0xFF) - (avg.data[indexInput] & 0xFF);
					sum += diff * diff;
				}
				output.data[indexOutput] = (byte) Math.sqrt(sum/divisor);
			}
		});
	}

	public static void minimum(Planar<GrayS16> input , GrayS16 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayS16[] bands = input.bands;
		
		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput+w;
			// for(int x = 0; x < w; x++ ) {
			for (; indexInput < indexEnd; indexInput++, indexOutput++ ) {
				int minimum = Integer.MAX_VALUE;
				for( int i = startBand; i <= lastBand; i++ ) {
					int value = bands[i].data[ indexInput ] ;
					if ( value < minimum) {
						minimum = value;
					}
				}
				output.data[indexOutput] = (short) minimum;
			}
		});
	}

	public static void maximum(Planar<GrayS16> input , GrayS16 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayS16[] bands = input.bands;

		BoofConcurrency.range(0,h,y->{
			int indexInput = input.startIndex + y * input.stride;
			int indexOutput = output.startIndex + y * output.stride;

			int indexEnd = indexInput + w;
			// for(int x = 0; x < w; x++ ) {
			for(; indexInput < indexEnd; indexInput++, indexOutput++) {
				int maximum = -Integer.MAX_VALUE;
				for (int i = startBand; i <= lastBand; i++) {
					int value = bands[i].data[ indexInput ] ;
					if( value > maximum) {
						maximum = value;
					}
				}
				output.data[indexOutput] = (short) maximum;
			} 
		});
	}

	public static void median(Planar<GrayS16> input , GrayS16 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayS16[] bands = input.bands;
		final int numBands = lastBand - startBand + 1;

		// handle edge case
		if( numBands == 1 ) {
			ImageMiscOps.copy(0, 0, 0, 0, input.getWidth(), input.getHeight(), input.getBand(startBand), output);
			return;
		}

		final int middle = numBands/2;
		boolean isEven = numBands % 2 == 0;
		BoofConcurrency.range(0,h,y->{
			final int[] valueArray = new int[numBands];
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput+w;
			for (; indexInput < indexEnd; indexInput++, indexOutput++ ) {
				for( int i = startBand; i <= lastBand; i++ ) {
					valueArray[i] = bands[i].data[indexInput];
				}
				if (isEven) {
					// Would a single quick sort be faster?
					int val0 = QuickSelect.select(valueArray, middle, numBands);
					int val1 = QuickSelect.select(valueArray, middle+1, numBands);
					output.data[indexOutput] = (short) ((val0+val1)/2);
				} else {
					output.data[indexOutput] = (short)QuickSelect.select(valueArray, middle, numBands);
				}
			}
		});
	}

	public static void average(Planar<GrayS16> input , GrayS16 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayS16[] bands = input.bands;
		int divisor = lastBand - startBand+1;
		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput + w;
			for (; indexInput < indexEnd; indexInput++, indexOutput++) {
			int sum = bands[startBand].data[indexInput] ;
				for (int i = startBand+1; i <= lastBand; i++) {
					sum += bands[i].data[indexInput] ;
				}
				output.data[indexOutput] = (short) (sum/divisor);
			}
		});
	}

	public static void stdDev(Planar<GrayS16> input , GrayS16 output , GrayS16 avg, int startBand, int lastBand ) {

		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayS16[] bands = input.bands;
		int divisor = lastBand - startBand;

		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput + w;

			for (; indexInput < indexEnd; indexInput++, indexOutput++) {
				int sum = 0;
				for (int i = startBand; i <= lastBand; i++) {
					int diff = (bands[i].data[indexInput] ) - (avg.data[indexInput] );
					sum += diff * diff;
				}
				output.data[indexOutput] = (short) Math.sqrt(sum/divisor);
			}
		});
	}

	public static void minimum(Planar<GrayU16> input , GrayU16 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayU16[] bands = input.bands;
		
		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput+w;
			// for(int x = 0; x < w; x++ ) {
			for (; indexInput < indexEnd; indexInput++, indexOutput++ ) {
				int minimum = Integer.MAX_VALUE;
				for( int i = startBand; i <= lastBand; i++ ) {
					int value = bands[i].data[ indexInput ] & 0xFFFF;
					if ( value < minimum) {
						minimum = value;
					}
				}
				output.data[indexOutput] = (short) minimum;
			}
		});
	}

	public static void maximum(Planar<GrayU16> input , GrayU16 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayU16[] bands = input.bands;

		BoofConcurrency.range(0,h,y->{
			int indexInput = input.startIndex + y * input.stride;
			int indexOutput = output.startIndex + y * output.stride;

			int indexEnd = indexInput + w;
			// for(int x = 0; x < w; x++ ) {
			for(; indexInput < indexEnd; indexInput++, indexOutput++) {
				int maximum = -Integer.MAX_VALUE;
				for (int i = startBand; i <= lastBand; i++) {
					int value = bands[i].data[ indexInput ] & 0xFFFF;
					if( value > maximum) {
						maximum = value;
					}
				}
				output.data[indexOutput] = (short) maximum;
			} 
		});
	}

	public static void median(Planar<GrayU16> input , GrayU16 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayU16[] bands = input.bands;
		final int numBands = lastBand - startBand + 1;

		// handle edge case
		if( numBands == 1 ) {
			ImageMiscOps.copy(0, 0, 0, 0, input.getWidth(), input.getHeight(), input.getBand(startBand), output);
			return;
		}

		final int middle = numBands/2;
		boolean isEven = numBands % 2 == 0;
		BoofConcurrency.range(0,h,y->{
			final int[] valueArray = new int[numBands];
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput+w;
			for (; indexInput < indexEnd; indexInput++, indexOutput++ ) {
				for( int i = startBand; i <= lastBand; i++ ) {
					valueArray[i] = bands[i].data[indexInput]& 0xFFFF;
				}
				if (isEven) {
					// Would a single quick sort be faster?
					int val0 = QuickSelect.select(valueArray, middle, numBands);
					int val1 = QuickSelect.select(valueArray, middle+1, numBands);
					output.data[indexOutput] = (short) ((val0+val1)/2);
				} else {
					output.data[indexOutput] = (short)QuickSelect.select(valueArray, middle, numBands);
				}
			}
		});
	}

	public static void average(Planar<GrayU16> input , GrayU16 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayU16[] bands = input.bands;
		int divisor = lastBand - startBand+1;
		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput + w;
			for (; indexInput < indexEnd; indexInput++, indexOutput++) {
			int sum = bands[startBand].data[indexInput] & 0xFFFF;
				for (int i = startBand+1; i <= lastBand; i++) {
					sum += bands[i].data[indexInput] & 0xFFFF;
				}
				output.data[indexOutput] = (short) (sum/divisor);
			}
		});
	}

	public static void stdDev(Planar<GrayU16> input , GrayU16 output , GrayU16 avg, int startBand, int lastBand ) {

		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayU16[] bands = input.bands;
		int divisor = lastBand - startBand;

		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput + w;

			for (; indexInput < indexEnd; indexInput++, indexOutput++) {
				int sum = 0;
				for (int i = startBand; i <= lastBand; i++) {
					int diff = (bands[i].data[indexInput] & 0xFFFF) - (avg.data[indexInput] & 0xFFFF);
					sum += diff * diff;
				}
				output.data[indexOutput] = (short) Math.sqrt(sum/divisor);
			}
		});
	}

	public static void minimum(Planar<GrayS32> input , GrayS32 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayS32[] bands = input.bands;
		
		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput+w;
			// for(int x = 0; x < w; x++ ) {
			for (; indexInput < indexEnd; indexInput++, indexOutput++ ) {
				int minimum = Integer.MAX_VALUE;
				for( int i = startBand; i <= lastBand; i++ ) {
					int value = bands[i].data[ indexInput ] ;
					if ( value < minimum) {
						minimum = value;
					}
				}
				output.data[indexOutput] =  minimum;
			}
		});
	}

	public static void maximum(Planar<GrayS32> input , GrayS32 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayS32[] bands = input.bands;

		BoofConcurrency.range(0,h,y->{
			int indexInput = input.startIndex + y * input.stride;
			int indexOutput = output.startIndex + y * output.stride;

			int indexEnd = indexInput + w;
			// for(int x = 0; x < w; x++ ) {
			for(; indexInput < indexEnd; indexInput++, indexOutput++) {
				int maximum = -Integer.MAX_VALUE;
				for (int i = startBand; i <= lastBand; i++) {
					int value = bands[i].data[ indexInput ] ;
					if( value > maximum) {
						maximum = value;
					}
				}
				output.data[indexOutput] =  maximum;
			} 
		});
	}

	public static void median(Planar<GrayS32> input , GrayS32 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayS32[] bands = input.bands;
		final int numBands = lastBand - startBand + 1;

		// handle edge case
		if( numBands == 1 ) {
			ImageMiscOps.copy(0, 0, 0, 0, input.getWidth(), input.getHeight(), input.getBand(startBand), output);
			return;
		}

		final int middle = numBands/2;
		boolean isEven = numBands % 2 == 0;
		BoofConcurrency.range(0,h,y->{
			final int[] valueArray = new int[numBands];
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput+w;
			for (; indexInput < indexEnd; indexInput++, indexOutput++ ) {
				for( int i = startBand; i <= lastBand; i++ ) {
					valueArray[i] = bands[i].data[indexInput];
				}
				if (isEven) {
					// Would a single quick sort be faster?
					int val0 = QuickSelect.select(valueArray, middle, numBands);
					int val1 = QuickSelect.select(valueArray, middle+1, numBands);
					output.data[indexOutput] =  ((val0+val1)/2);
				} else {
					output.data[indexOutput] = QuickSelect.select(valueArray, middle, numBands);
				}
			}
		});
	}

	public static void average(Planar<GrayS32> input , GrayS32 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayS32[] bands = input.bands;
		int divisor = lastBand - startBand+1;
		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput + w;
			for (; indexInput < indexEnd; indexInput++, indexOutput++) {
			int sum = bands[startBand].data[indexInput] ;
				for (int i = startBand+1; i <= lastBand; i++) {
					sum += bands[i].data[indexInput] ;
				}
				output.data[indexOutput] =  (sum/divisor);
			}
		});
	}

	public static void stdDev(Planar<GrayS32> input , GrayS32 output , GrayS32 avg, int startBand, int lastBand ) {

		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayS32[] bands = input.bands;
		int divisor = lastBand - startBand;

		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput + w;

			for (; indexInput < indexEnd; indexInput++, indexOutput++) {
				int sum = 0;
				for (int i = startBand; i <= lastBand; i++) {
					int diff = (bands[i].data[indexInput] ) - (avg.data[indexInput] );
					sum += diff * diff;
				}
				output.data[indexOutput] = (int) Math.sqrt(sum/divisor);
			}
		});
	}

	public static void minimum(Planar<GrayS64> input , GrayS64 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayS64[] bands = input.bands;
		
		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput+w;
			// for(int x = 0; x < w; x++ ) {
			for (; indexInput < indexEnd; indexInput++, indexOutput++ ) {
				long minimum = Long.MAX_VALUE;
				for( int i = startBand; i <= lastBand; i++ ) {
					long value = bands[i].data[ indexInput ] ;
					if ( value < minimum) {
						minimum = value;
					}
				}
				output.data[indexOutput] =  minimum;
			}
		});
	}

	public static void maximum(Planar<GrayS64> input , GrayS64 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayS64[] bands = input.bands;

		BoofConcurrency.range(0,h,y->{
			int indexInput = input.startIndex + y * input.stride;
			int indexOutput = output.startIndex + y * output.stride;

			int indexEnd = indexInput + w;
			// for(int x = 0; x < w; x++ ) {
			for(; indexInput < indexEnd; indexInput++, indexOutput++) {
				long maximum = -Long.MAX_VALUE;
				for (int i = startBand; i <= lastBand; i++) {
					long value = bands[i].data[ indexInput ] ;
					if( value > maximum) {
						maximum = value;
					}
				}
				output.data[indexOutput] =  maximum;
			} 
		});
	}

	public static void median(Planar<GrayS64> input , GrayS64 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayS64[] bands = input.bands;
		final int numBands = lastBand - startBand + 1;

		// handle edge case
		if( numBands == 1 ) {
			ImageMiscOps.copy(0, 0, 0, 0, input.getWidth(), input.getHeight(), input.getBand(startBand), output);
			return;
		}

		final int middle = numBands/2;
		boolean isEven = numBands % 2 == 0;
		BoofConcurrency.range(0,h,y->{
			final long[] valueArray = new long[numBands];
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput+w;
			for (; indexInput < indexEnd; indexInput++, indexOutput++ ) {
				for( int i = startBand; i <= lastBand; i++ ) {
					valueArray[i] = bands[i].data[indexInput];
				}
				if (isEven) {
					// Would a single quick sort be faster?
					long val0 = QuickSelect.select(valueArray, middle, numBands);
					long val1 = QuickSelect.select(valueArray, middle+1, numBands);
					output.data[indexOutput] =  ((val0+val1)/2);
				} else {
					output.data[indexOutput] = QuickSelect.select(valueArray, middle, numBands);
				}
			}
		});
	}

	public static void average(Planar<GrayS64> input , GrayS64 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayS64[] bands = input.bands;
		long divisor = lastBand - startBand+1;
		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput + w;
			for (; indexInput < indexEnd; indexInput++, indexOutput++) {
			long sum = bands[startBand].data[indexInput] ;
				for (int i = startBand+1; i <= lastBand; i++) {
					sum += bands[i].data[indexInput] ;
				}
				output.data[indexOutput] =  (sum/divisor);
			}
		});
	}

	public static void stdDev(Planar<GrayS64> input , GrayS64 output , GrayS64 avg, int startBand, int lastBand ) {

		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayS64[] bands = input.bands;
		long divisor = lastBand - startBand;

		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput + w;

			for (; indexInput < indexEnd; indexInput++, indexOutput++) {
				long sum = 0;
				for (int i = startBand; i <= lastBand; i++) {
					long diff = (bands[i].data[indexInput] ) - (avg.data[indexInput] );
					sum += diff * diff;
				}
				output.data[indexOutput] = (long) Math.sqrt(sum/divisor);
			}
		});
	}

	public static void minimum(Planar<GrayF32> input , GrayF32 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayF32[] bands = input.bands;
		
		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput+w;
			// for(int x = 0; x < w; x++ ) {
			for (; indexInput < indexEnd; indexInput++, indexOutput++ ) {
				float minimum = Float.MAX_VALUE;
				for( int i = startBand; i <= lastBand; i++ ) {
					float value = bands[i].data[ indexInput ] ;
					if ( value < minimum) {
						minimum = value;
					}
				}
				output.data[indexOutput] =  minimum;
			}
		});
	}

	public static void maximum(Planar<GrayF32> input , GrayF32 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayF32[] bands = input.bands;

		BoofConcurrency.range(0,h,y->{
			int indexInput = input.startIndex + y * input.stride;
			int indexOutput = output.startIndex + y * output.stride;

			int indexEnd = indexInput + w;
			// for(int x = 0; x < w; x++ ) {
			for(; indexInput < indexEnd; indexInput++, indexOutput++) {
				float maximum = -Float.MAX_VALUE;
				for (int i = startBand; i <= lastBand; i++) {
					float value = bands[i].data[ indexInput ] ;
					if( value > maximum) {
						maximum = value;
					}
				}
				output.data[indexOutput] =  maximum;
			} 
		});
	}

	public static void median(Planar<GrayF32> input , GrayF32 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayF32[] bands = input.bands;
		final int numBands = lastBand - startBand + 1;

		// handle edge case
		if( numBands == 1 ) {
			ImageMiscOps.copy(0, 0, 0, 0, input.getWidth(), input.getHeight(), input.getBand(startBand), output);
			return;
		}

		final int middle = numBands/2;
		boolean isEven = numBands % 2 == 0;
		BoofConcurrency.range(0,h,y->{
			final float[] valueArray = new float[numBands];
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput+w;
			for (; indexInput < indexEnd; indexInput++, indexOutput++ ) {
				for( int i = startBand; i <= lastBand; i++ ) {
					valueArray[i] = bands[i].data[indexInput];
				}
				if (isEven) {
					// Would a single quick sort be faster?
					float val0 = QuickSelect.select(valueArray, middle, numBands);
					float val1 = QuickSelect.select(valueArray, middle+1, numBands);
					output.data[indexOutput] =  ((val0+val1)/2);
				} else {
					output.data[indexOutput] = QuickSelect.select(valueArray, middle, numBands);
				}
			}
		});
	}

	public static void average(Planar<GrayF32> input , GrayF32 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayF32[] bands = input.bands;
		float divisor = lastBand - startBand+1;
		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput + w;
			for (; indexInput < indexEnd; indexInput++, indexOutput++) {
			float sum = bands[startBand].data[indexInput] ;
				for (int i = startBand+1; i <= lastBand; i++) {
					sum += bands[i].data[indexInput] ;
				}
				output.data[indexOutput] =  (sum/divisor);
			}
		});
	}

	public static void stdDev(Planar<GrayF32> input , GrayF32 output , GrayF32 avg, int startBand, int lastBand ) {

		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayF32[] bands = input.bands;
		float divisor = lastBand - startBand;

		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput + w;

			for (; indexInput < indexEnd; indexInput++, indexOutput++) {
				float sum = 0;
				for (int i = startBand; i <= lastBand; i++) {
					float diff = (bands[i].data[indexInput] ) - (avg.data[indexInput] );
					sum += diff * diff;
				}
				output.data[indexOutput] = (float) Math.sqrt(sum/divisor);
			}
		});
	}

	public static void minimum(Planar<GrayF64> input , GrayF64 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayF64[] bands = input.bands;
		
		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput+w;
			// for(int x = 0; x < w; x++ ) {
			for (; indexInput < indexEnd; indexInput++, indexOutput++ ) {
				double minimum = Double.MAX_VALUE;
				for( int i = startBand; i <= lastBand; i++ ) {
					double value = bands[i].data[ indexInput ] ;
					if ( value < minimum) {
						minimum = value;
					}
				}
				output.data[indexOutput] =  minimum;
			}
		});
	}

	public static void maximum(Planar<GrayF64> input , GrayF64 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayF64[] bands = input.bands;

		BoofConcurrency.range(0,h,y->{
			int indexInput = input.startIndex + y * input.stride;
			int indexOutput = output.startIndex + y * output.stride;

			int indexEnd = indexInput + w;
			// for(int x = 0; x < w; x++ ) {
			for(; indexInput < indexEnd; indexInput++, indexOutput++) {
				double maximum = -Double.MAX_VALUE;
				for (int i = startBand; i <= lastBand; i++) {
					double value = bands[i].data[ indexInput ] ;
					if( value > maximum) {
						maximum = value;
					}
				}
				output.data[indexOutput] =  maximum;
			} 
		});
	}

	public static void median(Planar<GrayF64> input , GrayF64 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayF64[] bands = input.bands;
		final int numBands = lastBand - startBand + 1;

		// handle edge case
		if( numBands == 1 ) {
			ImageMiscOps.copy(0, 0, 0, 0, input.getWidth(), input.getHeight(), input.getBand(startBand), output);
			return;
		}

		final int middle = numBands/2;
		boolean isEven = numBands % 2 == 0;
		BoofConcurrency.range(0,h,y->{
			final double[] valueArray = new double[numBands];
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput+w;
			for (; indexInput < indexEnd; indexInput++, indexOutput++ ) {
				for( int i = startBand; i <= lastBand; i++ ) {
					valueArray[i] = bands[i].data[indexInput];
				}
				if (isEven) {
					// Would a single quick sort be faster?
					double val0 = QuickSelect.select(valueArray, middle, numBands);
					double val1 = QuickSelect.select(valueArray, middle+1, numBands);
					output.data[indexOutput] =  ((val0+val1)/2);
				} else {
					output.data[indexOutput] = QuickSelect.select(valueArray, middle, numBands);
				}
			}
		});
	}

	public static void average(Planar<GrayF64> input , GrayF64 output, int startBand, int lastBand ) {
		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayF64[] bands = input.bands;
		double divisor = lastBand - startBand+1;
		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput + w;
			for (; indexInput < indexEnd; indexInput++, indexOutput++) {
			double sum = bands[startBand].data[indexInput] ;
				for (int i = startBand+1; i <= lastBand; i++) {
					sum += bands[i].data[indexInput] ;
				}
				output.data[indexOutput] =  (sum/divisor);
			}
		});
	}

	public static void stdDev(Planar<GrayF64> input , GrayF64 output , GrayF64 avg, int startBand, int lastBand ) {

		final int h = input.getHeight();
		final int w = input.getWidth();

		final GrayF64[] bands = input.bands;
		double divisor = lastBand - startBand;

		BoofConcurrency.range(0,h,y->{
			int indexInput = input.getStartIndex() + y * input.getStride();
			int indexOutput = output.getStartIndex() + y * output.getStride();

			int indexEnd = indexInput + w;

			for (; indexInput < indexEnd; indexInput++, indexOutput++) {
				double sum = 0;
				for (int i = startBand; i <= lastBand; i++) {
					double diff = (bands[i].data[indexInput] ) - (avg.data[indexInput] );
					sum += diff * diff;
				}
				output.data[indexOutput] = (double) Math.sqrt(sum/divisor);
			}
		});
	}

}
