/*
 * Copyright (c) 2011-2016, Peter Abeles. All Rights Reserved.
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

package boofcv.alg.misc;

import boofcv.misc.AutoTypeImage;
import boofcv.misc.CodeGeneratorBase;
import boofcv.struct.image.ImageType;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;

import static boofcv.misc.AutoTypeImage.*;


/**
 * Generates functions inside of {@link boofcv.alg.misc.ImageMiscOps}.
 *
 * @author Peter Abeles
 */
public class GeneratePixelMath extends CodeGeneratorBase {

	String className = "PixelMath";

	private AutoTypeImage input;

	ImageType.Family families[] = new ImageType.Family[]{ImageType.Family.SINGLE_BAND,ImageType.Family.INTERLEAVED};

	public void generate() throws FileNotFoundException {
		printPreamble();

		printAbs();
		printInvert();

		List<TwoTemplate> listTwo = new ArrayList<TwoTemplate>();
		listTwo.add( new Multiple());
		listTwo.add( new Divide());
		listTwo.add( new Plus());
		listTwo.add( new Minus(true));
		listTwo.add( new Minus(false));

		for( TwoTemplate t : listTwo ) {
			print_img_scalar(t,false);
			print_img_scalar(t,true);
		}

		printAll();
		out.println("}");
	}

	private void printPreamble() throws FileNotFoundException {
		setOutputFile(className);
		out.print("import boofcv.struct.image.*;\n" +
				"\n" +
				"import boofcv.alg.InputSanityCheck;\n" +
				"import javax.annotation.Generated;\n" +
				"\n" +
				"/**\n" +
				" * Functions which perform basic arithmetic (e.g. addition, subtraction, multiplication, or " +
				"division) on a pixel by pixel basis.\n" +
				" *\n" +
				" * <p>DO NOT MODIFY: Generated by " + getClass().getName() + ".</p>\n" +
				" *\n" +
				" * @author Peter Abeles\n" +
				" */\n" +
				generatedString() +
				"public class " + className + " {\n\n");
	}

	public void printAll() {

		AutoTypeImage types[] = AutoTypeImage.getSpecificTypes();

		for( AutoTypeImage t : types ) {
			input = t;

			printBoundImage();
			printDiffAbs();
			printAverageBand();
		}

		AutoTypeImage outputsAdd[] = new AutoTypeImage[]{U16,S16,S32,S32,S32,S64,F32,F64};
		AutoTypeImage outputsSub[] = new AutoTypeImage[]{I16,S16,S32,S32,S32,S64,F32,F64};

		for( int i = 0; i < types.length; i++ ) {
			printAddTwoImages(types[i],outputsAdd[i]);
			printSubtractTwoImages(types[i],outputsSub[i]);

			if( !types[i].isInteger() ) {
				printMultTwoImages(types[i],types[i]);
				printDivTwoImages(types[i],types[i]);
				printLog(types[i],types[i]);
				printPow2(types[i], types[i]);
				printSqrt(types[i], types[i]);
			}
		}
	}

	private void print( String funcName , String javadoc , String operation , AutoTypeImage types[] ) {
		for( AutoTypeImage t : types ) {
			input = t;
			for (ImageType.Family family : families) {
				String inputName, columns;
				if (family == ImageType.Family.INTERLEAVED) {
					inputName = input.getInterleavedName();
					columns = "input.width*input.numBands";
				} else {
					inputName = input.getSingleBandName();
					columns = "input.width";
				}

				out.println(javadoc + "\n" +
						"\tpublic static void " + funcName + "( " + inputName + " input , " + inputName + " output ) {\n" +
						"\n" +
						"\t\tInputSanityCheck.checkSameShape(input,output);\n" +
						"\n" +
						"\t\tint columns = " + columns + ";\n" +
						"\t\t" + funcName + "(input.data,input.startIndex,input.stride,\n" +
						"\t\t\t\toutput.data,output.startIndex,output.stride,\n" +
						"\t\t\t\tinput.height,columns);\n" +
						"\t}\n");
			}
			printArray(funcName,operation);
		}
	}

	public void printArray( String funcName , String operation )
	{
		String arrayType = input.getDataType();

		out.println("\tprivate static void "+funcName+"( "+arrayType+"[] input , int inputStart , int inputStride ,\n" +
				"\t\t\t\t\t\t\t   "+arrayType+"[] output , int outputStart , int outputStride ,\n" +
				"\t\t\t\t\t\t\t   int rows , int cols )\n" +
				"\t{\n" +
				"\t\tfor( int y = 0; y < rows; y++ ) {\n" +
				"\t\t\tint indexSrc = inputStart + y*inputStride;\n" +
				"\t\t\tint indexDst = outputStart + y*outputStride;\n" +
				"\t\t\tint end = indexSrc + cols;\n" +
				"\n" +
				"\t\t\tfor( ; indexSrc < end; indexSrc++ , indexDst++) {\n" +
				"\t\t\t\toutput[indexDst] = "+input.getTypeCastFromSum()+operation +";\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\t}\n");
	}

	public void printAbs()
	{
		String javaDoc = "\t/**\n" +
				"\t * Sets each pixel in the output image to be the absolute value of the input image.\n" +
				"\t * Both the input and output image can be the same instance.\n" +
				"\t * \n" +
				"\t * @param input The input image. Not modified.\n" +
				"\t * @param output Where the absolute value image is written to. Modified.\n" +
				"\t */";

		print("abs",javaDoc,"Math.abs(input[indexSrc])",AutoTypeImage.getSigned());
	}

	public void printInvert()
	{
		String javaDoc = "\t/**\n" +
				"\t * Changes the sign of every pixel in the image: output[x,y] = -input[x,y]\n" +
				"\t *\n" +
				"\t * @param input The input image. Not modified.\n" +
				"\t * @param output Where the inverted image is written to. Modified.\n" +
				"\t */";

		print("invert",javaDoc,"-input[indexSrc]",AutoTypeImage.getSigned());
	}

	private void print_img_scalar( TwoTemplate template , boolean bounded ) {

		String funcName = template.getName();
		String varName = template.getVariableName();

		for( AutoTypeImage t : template.getTypes() ) {
			input = t;
			String variableType;
			if( template.isScaleOp() )
				variableType = input.isInteger() ? "double" : input.getSumType();
			else
				variableType = input.getSumType();

			String funcArrayName = input.isSigned() ? funcName : funcName+"U";
			funcArrayName += template.isImageFirst() ? "_A" : "_B";

			for (ImageType.Family family : families) {
				String inputName, columns, banded;
				if (family == ImageType.Family.INTERLEAVED) {
					inputName = input.getInterleavedName();
					columns = "input.width*input.numBands";
					banded = "B";
				} else {
					inputName = input.getSingleBandName();
					columns = "input.width";
					banded = "";
				}

				if( bounded ) {
					String sumType = input.getSumType();

					String prototype;
					if (template.isImageFirst()) {
						prototype = "( " + inputName + " input , " + variableType + " " + varName +
								" , " + sumType +" lower , " +sumType+ " upper , " + inputName + " output )";
					} else {
						prototype = "( " + variableType + " " + varName + " , " + inputName + " input , " +
								sumType +" lower , " +sumType+ " upper , " + inputName + " output )";;
					}

					out.println(template.getJavaDoc());
					out.println("\tpublic static void " + funcName + prototype + " {\n" +
							"\n" +
							"\t\tInputSanityCheck.checkSameShape(input,output);\n" +
							"\n" +
							"\t\tint columns = " + columns + ";\n" +
							"\t\t" + funcArrayName + "(input.data,input.startIndex,input.stride," + varName + ", lower, upper ,\n" +
							"\t\t\t\toutput.data,output.startIndex,output.stride,\n" +
							"\t\t\t\tinput.height,columns);\n" +
							"\t}\n");
				} else {
					String prototype;
					if (template.isImageFirst()) {
						prototype = "( " + inputName + " input , " + variableType + " " + varName + " , " + inputName + " output )";
					} else {
						prototype = "( " + variableType + " " + varName + " , " + inputName + " input , " + inputName + " output )";
					}

					out.println(template.getJavaDoc());
					out.println("\tpublic static void " + funcName + prototype + " {\n" +
							"\n" +
							"\t\tInputSanityCheck.checkSameShape"+banded+"(input,output);\n" +
							"\n" +
							"\t\tint columns = " + columns + ";\n" +
							"\t\t" + funcArrayName + "(input.data,input.startIndex,input.stride," + varName + " , \n" +
							"\t\t\t\toutput.data,output.startIndex,output.stride,\n" +
							"\t\t\t\tinput.height,columns);\n" +
							"\t}\n");
				}
			}
			if( bounded ) {
				print_array_scalar_bounded(funcArrayName, variableType, varName, template.getOperation());
			} else {
				print_array_scalar(funcArrayName, variableType, varName, template.getOperation());
			}
		}
	}

	public void print_array_scalar(String funcName , String varType , String varName , String operation  )
	{
		String arrayType = input.getDataType();

		String typeCast = varType.equals(input.getDataType()) ? "" : "("+input.getDataType()+")";

		out.println("\tprivate static void "+funcName+"( "+arrayType+"[] input , int inputStart , int inputStride , \n" +
				"\t\t\t\t\t\t\t   "+varType+" "+varName+" ,\n" +
				"\t\t\t\t\t\t\t   "+arrayType+"[] output , int outputStart , int outputStride ,\n" +
				"\t\t\t\t\t\t\t   int rows , int cols )\n" +
				"\t{\n" +
				"\t\tfor( int y = 0; y < rows; y++ ) {\n" +
				"\t\t\tint indexSrc = inputStart + y*inputStride;\n" +
				"\t\t\tint indexDst = outputStart + y*outputStride;\n" +
				"\t\t\tint end = indexSrc + cols;\n" +
				"\n" +
				"\t\t\tfor( ; indexSrc < end; indexSrc++ , indexDst++) {\n" +
				"\t\t\t\toutput[indexDst] = "+typeCast+operation +";\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\t}\n");
	}

	public void print_array_scalar_bounded(String funcName , String varType , String varName , String operation  )
	{
		String arrayType = input.getDataType();

		String sumType = input.getSumType();
		String typeCast = varType.equals(sumType) ? "" : "("+sumType+")";

		out.println("\tprivate static void "+funcName+"( "+arrayType+"[] input , int inputStart , int inputStride , \n" +
				"\t\t\t\t\t\t\t   "+varType+" "+varName+" , "+sumType+" lower , "+sumType+" upper ,\n" +
				"\t\t\t\t\t\t\t   "+arrayType+"[] output , int outputStart , int outputStride ,\n" +
				"\t\t\t\t\t\t\t   int rows , int cols )\n" +
				"\t{\n" +
				"\t\tfor( int y = 0; y < rows; y++ ) {\n" +
				"\t\t\tint indexSrc = inputStart + y*inputStride;\n" +
				"\t\t\tint indexDst = outputStart + y*outputStride;\n" +
				"\t\t\tint end = indexSrc + cols;\n" +
				"\n" +
				"\t\t\tfor( ; indexSrc < end; indexSrc++ , indexDst++) {\n" +
				"\t\t\t\t"+sumType+" val = "+typeCast+operation+";\n" +
				"\t\t\t\tif( val < lower ) val = lower;\n" +
				"\t\t\t\tif( val > upper ) val = upper;\n" +
				"\t\t\t\toutput[indexDst] = "+input.getTypeCastFromSum()+"val;\n"+
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\t}\n");
	}

	public void printBoundImage() {

		String bitWise = input.getBitWise();
		String sumType = input.getSumType();

		out.print("\t/**\n" +
				"\t * Bounds image pixels to be between these two values\n" +
				"\t * \n" +
				"\t * @param img Image\n" +
				"\t * @param min minimum value.\n" +
				"\t * @param max maximum value.\n" +
				"\t */\n" +
				"\tpublic static void boundImage( "+input.getSingleBandName()+" img , "+sumType+" min , "+sumType+" max ) {\n" +
				"\t\tfinal int h = img.getHeight();\n" +
				"\t\tfinal int w = img.getWidth();\n" +
				"\n" +
				"\t\t"+input.getDataType()+"[] data = img.data;\n" +
				"\n" +
				"\t\tfor (int y = 0; y < h; y++) {\n" +
				"\t\t\tint index = img.getStartIndex() + y * img.getStride();\n" +
				"\t\t\tint indexEnd = index+w;\n" +
				"\t\t\t// for(int x = 0; x < w; x++ ) {\n" +
				"\t\t\tfor (; index < indexEnd; index++) {\n" +
				"\t\t\t\t"+sumType+" value = data[index]"+bitWise+";\n" +
				"\t\t\t\tif( value < min )\n" +
				"\t\t\t\t\tdata[index] = "+input.getTypeCastFromSum()+"min;\n" +
				"\t\t\t\telse if( value > max )\n" +
				"\t\t\t\t\tdata[index] = "+input.getTypeCastFromSum()+"max;\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\t}\n\n");
	}

	public void printDiffAbs() {

		String bitWise = input.getBitWise();
		String typeCast = input.isInteger() ? "("+input.getDataType()+")" : "";

		out.print("\t/**\n" +
				"\t * <p>\n" +
				"\t * Computes the absolute value of the difference between each pixel in the two images.<br>\n" +
				"\t * d(x,y) = |img1(x,y) - img2(x,y)|\n" +
				"\t * </p>\n" +
				"\t * @param imgA Input image. Not modified.\n" +
				"\t * @param imgB Input image. Not modified.\n" +
				"\t * @param diff Absolute value of difference image. Modified.\n" +
				"\t */\n" +
				"\tpublic static void diffAbs( "+input.getSingleBandName()+" imgA , "+input.getSingleBandName()+" imgB , "+input.getSingleBandName()+" diff ) {\n" +
				"\t\tInputSanityCheck.checkSameShape(imgA,imgB,diff);\n" +
				"\t\t\n" +
				"\t\tfinal int h = imgA.getHeight();\n" +
				"\t\tfinal int w = imgA.getWidth();\n" +
				"\n" +
				"\t\tfor (int y = 0; y < h; y++) {\n" +
				"\t\t\tint indexA = imgA.getStartIndex() + y * imgA.getStride();\n" +
				"\t\t\tint indexB = imgB.getStartIndex() + y * imgB.getStride();\n" +
				"\t\t\tint indexDiff = diff.getStartIndex() + y * diff.getStride();\n" +
				"\t\t\t\n" +
				"\t\t\tint indexEnd = indexA+w;\n" +
				"\t\t\t// for(int x = 0; x < w; x++ ) {\n" +
				"\t\t\tfor (; indexA < indexEnd; indexA++, indexB++, indexDiff++ ) {\n" +
				"\t\t\t\tdiff.data[indexDiff] = "+typeCast+"Math.abs((imgA.data[indexA] "+bitWise+") - (imgB.data[indexB] "+bitWise+"));\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\t}\n\n");
	}

	public void printAddTwoImages( AutoTypeImage typeIn , AutoTypeImage typeOut  ) {

		String bitWise = typeIn.getBitWise();
		String typeCast = typeOut.isInteger() ? "("+typeOut.getDataType()+")" : "";

		out.print("\t/**\n" +
				"\t * <p>\n" +
				"\t * Performs pixel-wise addition<br>\n" +
				"\t * output(x,y) = imgA(x,y) + imgB(x,y)\n" +
				"\t * </p>\n" +
				"\t * @param imgA Input image. Not modified.\n" +
				"\t * @param imgB Input image. Not modified.\n" +
				"\t * @param output Output image. Modified.\n" +
				"\t */\n" +
				"\tpublic static void add( "+typeIn.getSingleBandName()+" imgA , "+typeIn.getSingleBandName()+" imgB , "+typeOut.getSingleBandName()+" output ) {\n" +
				"\t\tInputSanityCheck.checkSameShape(imgA,imgB,output);\n" +
				"\t\t\n" +
				"\t\tfinal int h = imgA.getHeight();\n" +
				"\t\tfinal int w = imgA.getWidth();\n" +
				"\n" +
				"\t\tfor (int y = 0; y < h; y++) {\n" +
				"\t\t\tint indexA = imgA.getStartIndex() + y * imgA.getStride();\n" +
				"\t\t\tint indexB = imgB.getStartIndex() + y * imgB.getStride();\n" +
				"\t\t\tint indexOut = output.getStartIndex() + y * output.getStride();\n" +
				"\t\t\t\n" +
				"\t\t\tint indexEnd = indexA+w;\n" +
				"\t\t\t// for(int x = 0; x < w; x++ ) {\n" +
				"\t\t\tfor (; indexA < indexEnd; indexA++, indexB++, indexOut++ ) {\n" +
				"\t\t\t\toutput.data[indexOut] = "+typeCast+"((imgA.data[indexA] "+bitWise+") + (imgB.data[indexB] "+bitWise+"));\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\t}\n\n");
	}

	public void printSubtractTwoImages( AutoTypeImage typeIn , AutoTypeImage typeOut ) {

		String bitWise = typeIn.getBitWise();
		String typeCast = typeOut.isInteger() ? "("+typeOut.getDataType()+")" : "";

		out.print("\t/**\n" +
				"\t * <p>\n" +
				"\t * Performs pixel-wise subtraction.<br>\n" +
				"\t * output(x,y) = imgA(x,y) - imgB(x,y)\n" +
				"\t * </p>\n" +
				"\t * @param imgA Input image. Not modified.\n" +
				"\t * @param imgB Input image. Not modified.\n" +
				"\t * @param output Output image. Modified.\n" +
				"\t */\n" +
				"\tpublic static void subtract( "+typeIn.getSingleBandName()+" imgA , "+typeIn.getSingleBandName()+" imgB , "
				+typeOut.getSingleBandName()+" output ) {\n" +
				"\t\tInputSanityCheck.checkSameShape(imgA,imgB,output);\n" +
				"\t\t\n" +
				"\t\tfinal int h = imgA.getHeight();\n" +
				"\t\tfinal int w = imgA.getWidth();\n" +
				"\n" +
				"\t\tfor (int y = 0; y < h; y++) {\n" +
				"\t\t\tint indexA = imgA.getStartIndex() + y * imgA.getStride();\n" +
				"\t\t\tint indexB = imgB.getStartIndex() + y * imgB.getStride();\n" +
				"\t\t\tint indexOut = output.getStartIndex() + y * output.getStride();\n" +
				"\t\t\t\n" +
				"\t\t\tint indexEnd = indexA+w;\n" +
				"\t\t\t// for(int x = 0; x < w; x++ ) {\n" +
				"\t\t\tfor (; indexA < indexEnd; indexA++, indexB++, indexOut++ ) {\n" +
				"\t\t\t\toutput.data[indexOut] = "+typeCast+"((imgA.data[indexA] "+bitWise+") - (imgB.data[indexB] "+bitWise+"));\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\t}\n\n");
	}

	public void printMultTwoImages( AutoTypeImage typeIn , AutoTypeImage typeOut  ) {

		String bitWise = typeIn.getBitWise();
		String typeCast = typeOut.isInteger() ? "("+typeOut.getDataType()+")" : "";

		out.print("\t/**\n" +
				"\t * <p>\n" +
				"\t * Performs pixel-wise multiplication<br>\n" +
				"\t * output(x,y) = imgA(x,y) * imgB(x,y)\n" +
				"\t * </p>\n" +
				"\t * @param imgA Input image. Not modified.\n" +
				"\t * @param imgB Input image. Not modified.\n" +
				"\t * @param output Output image. Modified.\n" +
				"\t */\n" +
				"\tpublic static void multiply( "+typeIn.getSingleBandName()+" imgA , "+typeIn.getSingleBandName()+" imgB , "+typeOut.getSingleBandName()+" output ) {\n" +
				"\t\tInputSanityCheck.checkSameShape(imgA,imgB,output);\n" +
				"\t\t\n" +
				"\t\tfinal int h = imgA.getHeight();\n" +
				"\t\tfinal int w = imgA.getWidth();\n" +
				"\n" +
				"\t\tfor (int y = 0; y < h; y++) {\n" +
				"\t\t\tint indexA = imgA.getStartIndex() + y * imgA.getStride();\n" +
				"\t\t\tint indexB = imgB.getStartIndex() + y * imgB.getStride();\n" +
				"\t\t\tint indexOut = output.getStartIndex() + y * output.getStride();\n" +
				"\t\t\t\n" +
				"\t\t\tint indexEnd = indexA+w;\n" +
				"\t\t\t// for(int x = 0; x < w; x++ ) {\n" +
				"\t\t\tfor (; indexA < indexEnd; indexA++, indexB++, indexOut++ ) {\n" +
				"\t\t\t\toutput.data[indexOut] = "+typeCast+"((imgA.data[indexA] "+bitWise+") * (imgB.data[indexB] "+bitWise+"));\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\t}\n\n");
	}

	public void printLog( AutoTypeImage typeIn , AutoTypeImage typeOut ) {
		String bitWise = typeIn.getBitWise();
		String typeCast = typeOut != AutoTypeImage.F64 ? "("+typeOut.getDataType()+")" : "";

		out.print("\t/**\n" +
				"\t * Sets each pixel in the output image to log( 1 + input(x,y)) of the input image.\n" +
				"\t * Both the input and output image can be the same instance.\n" +
				"\t *\n" +
				"\t * @param input The input image. Not modified.\n" +
				"\t * @param output Where the log image is written to. Modified.\n" +
				"\t */\n" +
				"\tpublic static void log( "+typeIn.getSingleBandName()+" input , "+typeOut.getSingleBandName()+" output ) {\n" +
				"\n" +
				"\t\tInputSanityCheck.checkSameShape(input,output);\n" +
				"\n" +
				"\t\tfor( int y = 0; y < input.height; y++ ) {\n" +
				"\t\t\tint indexSrc = input.startIndex + y* input.stride;\n" +
				"\t\t\tint indexDst = output.startIndex + y* output.stride;\n" +
				"\t\t\tint end = indexSrc + input.width;\n" +
				"\n" +
				"\t\t\tfor( ; indexSrc < end; indexSrc++ , indexDst++) {\n" +
				"\t\t\t\toutput.data[indexDst] = "+typeCast+"Math.log(1 + input.data[indexSrc]"+bitWise+");\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\t}\n\n");
	}

	public void printPow2( AutoTypeImage typeIn , AutoTypeImage typeOut ) {
		String bitWise = typeIn.getBitWise();

		out.print("\t/**\n" +
				"\t * Raises each pixel in the input image to the power of two. Both the input and output image can be the \n" +
				"\t * same instance." +
				"\t *\n" +
				"\t * @param input The input image. Not modified.\n" +
				"\t * @param output Where the pow2 image is written to. Modified.\n" +
				"\t */\n" +
				"\tpublic static void pow2( "+typeIn.getSingleBandName()+" input , "+typeOut.getSingleBandName()+" output ) {\n" +
				"\n" +
				"\t\tInputSanityCheck.checkSameShape(input,output);\n" +
				"\n" +
				"\t\tfor( int y = 0; y < input.height; y++ ) {\n" +
				"\t\t\tint indexSrc = input.startIndex + y* input.stride;\n" +
				"\t\t\tint indexDst = output.startIndex + y* output.stride;\n" +
				"\t\t\tint end = indexSrc + input.width;\n" +
				"\n" +
				"\t\t\tfor( ; indexSrc < end; indexSrc++ , indexDst++) {\n" +
				"\t\t\t\t"+typeOut.getDataType()+" v = input.data[indexSrc]"+bitWise+";\n" +
				"\t\t\t\toutput.data[indexDst] = v*v;\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\t}\n\n");
	}

	public void printSqrt( AutoTypeImage typeIn , AutoTypeImage typeOut ) {
		String bitWise = typeIn.getBitWise();
		String typeCast = typeOut != AutoTypeImage.F64 ? "("+typeOut.getDataType()+")" : "";

		out.print("\t/**\n" +
				"\t * Computes the square root of each pixel in the input image. Both the input and output image can be the\n" +
				"\t * same instance.\n" +
				"\t *\n" +
				"\t * @param input The input image. Not modified.\n" +
				"\t * @param output Where the sqrt() image is written to. Modified.\n" +
				"\t */\n" +
				"\tpublic static void sqrt( "+typeIn.getSingleBandName()+" input , "+typeOut.getSingleBandName()+" output ) {\n" +
				"\n" +
				"\t\tInputSanityCheck.checkSameShape(input,output);\n" +
				"\n" +
				"\t\tfor( int y = 0; y < input.height; y++ ) {\n" +
				"\t\t\tint indexSrc = input.startIndex + y* input.stride;\n" +
				"\t\t\tint indexDst = output.startIndex + y* output.stride;\n" +
				"\t\t\tint end = indexSrc + input.width;\n" +
				"\n" +
				"\t\t\tfor( ; indexSrc < end; indexSrc++ , indexDst++) {\n" +
				"\t\t\t\toutput.data[indexDst] = "+typeCast+"Math.sqrt(input.data[indexSrc]"+bitWise+");\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\t}\n\n");
	}

	public void printDivTwoImages( AutoTypeImage typeIn , AutoTypeImage typeOut  ) {

		String bitWise = typeIn.getBitWise();
		String typeCast = typeOut.isInteger() ? "("+typeOut.getDataType()+")" : "";

		out.print("\t/**\n" +
				"\t * <p>\n" +
				"\t * Performs pixel-wise division<br>\n" +
				"\t * output(x,y) = imgA(x,y) / imgB(x,y)\n" +
				"\t * </p>\n" +
				"\t * @param imgA Input image. Not modified.\n" +
				"\t * @param imgB Input image. Not modified.\n" +
				"\t * @param output Output image. Modified.\n" +
				"\t */\n" +
				"\tpublic static void divide( "+typeIn.getSingleBandName()+" imgA , "+typeIn.getSingleBandName()+" imgB , "+typeOut.getSingleBandName()+" output ) {\n" +
				"\t\tInputSanityCheck.checkSameShape(imgA,imgB,output);\n" +
				"\t\t\n" +
				"\t\tfinal int h = imgA.getHeight();\n" +
				"\t\tfinal int w = imgA.getWidth();\n" +
				"\n" +
				"\t\tfor (int y = 0; y < h; y++) {\n" +
				"\t\t\tint indexA = imgA.getStartIndex() + y * imgA.getStride();\n" +
				"\t\t\tint indexB = imgB.getStartIndex() + y * imgB.getStride();\n" +
				"\t\t\tint indexOut = output.getStartIndex() + y * output.getStride();\n" +
				"\t\t\t\n" +
				"\t\t\tint indexEnd = indexA+w;\n" +
				"\t\t\t// for(int x = 0; x < w; x++ ) {\n" +
				"\t\t\tfor (; indexA < indexEnd; indexA++, indexB++, indexOut++ ) {\n" +
				"\t\t\t\toutput.data[indexOut] = "+typeCast+"((imgA.data[indexA] "+bitWise+") / (imgB.data[indexB] "+bitWise+"));\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\t}\n\n");
	}


	public void printAverageBand() {
		
		String imageName = input.getSingleBandName();
		String sumType = input.getSumType();
		String typecast = input.getTypeCastFromSum();
		String bitwise = input.getBitWise();
		
		out.print("\t/**\n" +
				"\t * Computes the average for each pixel across all bands in the {@link MultiSpectral} image.\n" +
				"\t * \n" +
				"\t * @param input MultiSpectral image\n" +
				"\t * @param output Gray scale image containing average pixel values\n" +
				"\t */\n" +
				"\tpublic static void averageBand( MultiSpectral<"+imageName+"> input , "+imageName+" output ) {\n" +
				"\t\tfinal int h = input.getHeight();\n" +
				"\t\tfinal int w = input.getWidth();\n" +
				"\n" +
				"\t\t"+imageName+"[] bands = input.bands;\n" +
				"\t\t\n" +
				"\t\tfor (int y = 0; y < h; y++) {\n" +
				"\t\t\tint indexInput = input.getStartIndex() + y * input.getStride();\n" +
				"\t\t\tint indexOutput = output.getStartIndex() + y * output.getStride();\n" +
				"\n" +
				"\t\t\tint indexEnd = indexInput+w;\n" +
				"\t\t\t// for(int x = 0; x < w; x++ ) {\n" +
				"\t\t\tfor (; indexInput < indexEnd; indexInput++, indexOutput++ ) {\n" +
				"\t\t\t\t"+sumType+" total = 0;\n" +
				"\t\t\t\tfor( int i = 0; i < bands.length; i++ ) {\n" +
				"\t\t\t\t\ttotal += bands[i].data[ indexInput ]"+bitwise+";\n" +
				"\t\t\t\t}\n" +
				"\t\t\t\toutput.data[indexOutput] = "+typecast+"(total / bands.length);\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\t}\n\n");
	}


	class Multiple implements TwoTemplate {

		@Override
		public String getVariableName() { return "value";}

		@Override
		public boolean isScaleOp() { return true; }

		@Override
		public boolean isImageFirst() { return true; }

		@Override
		public AutoTypeImage[] getTypes() { return AutoTypeImage.getSpecificTypes(); }

		@Override
		public String getJavaDoc() {
			return "\t/**\n" +
					"\t * Multiply each element by a scalar value. Both input and output images can\n" +
					"\t * be the same instance.\n" +
					"\t *\n" +
					"\t * @param input The input image. Not modified.\n" +
					"\t * @param value What each element is multiplied by.\n" +
					"\t * @param output The output image. Modified.\n" +
					"\t */";
		}

		@Override
		public String getName() {return "multiply";}

		@Override
		public String getOperation() {
			String round = input.isInteger() ? "Math.round" : "";

			return round+"((input[indexSrc] "+input.getBitWise()+") * value)";
		}
	}

	class Divide implements TwoTemplate {

		@Override public String getVariableName() { return "denominator";}

		@Override public boolean isScaleOp() { return true; }

		@Override public boolean isImageFirst() { return true; }

		@Override public AutoTypeImage[] getTypes() { return AutoTypeImage.getSpecificTypes(); }

		@Override
		public String getJavaDoc() {
			return "\t/**\n" +
					"\t * Divide each element by a scalar value. Both input and output images can be the same instance.\n" +
					"\t *\n" +
					"\t * @param input The input image. Not modified.\n" +
					"\t * @param denominator What each element is divided by.\n" +
					"\t * @param output The output image. Modified.\n" +
					"\t */";
		}

		@Override
		public String getName() {return "divide";}

		@Override
		public String getOperation() {
			String round = input.isInteger() ? "Math.round" : "";

			return round+"((input[indexSrc] "+input.getBitWise()+") / denominator)";
		}
	}

	class Plus implements TwoTemplate {

		@Override public String getVariableName() { return "value";}

		@Override public boolean isScaleOp() { return false; }

		@Override public boolean isImageFirst() { return true; }

		@Override public AutoTypeImage[] getTypes() { return AutoTypeImage.getSpecificTypes(); }

		@Override
		public String getJavaDoc() {
			return "\t/**\n" +
					"\t * Adds a scalar value to each element. Both input and output images can be the same instance.\n" +
					"\t *\n" +
					"\t * @param input The input image. Not modified.\n" +
					"\t * @param value What is added to each element.\n" +
					"\t * @param output The output image. Modified.\n" +
					"\t */";
		}

		@Override
		public String getName() {return "plus";}

		@Override
		public String getOperation() {
			return "((input[indexSrc] "+input.getBitWise()+") + value)";
		}
	}

	class Minus implements TwoTemplate {

		boolean imageFirst;

		public Minus(boolean imageFirst) {
			this.imageFirst = imageFirst;
		}

		@Override public String getVariableName() { return "value";}

		@Override public boolean isScaleOp() { return false; }

		@Override public boolean isImageFirst() { return imageFirst; }

		@Override public AutoTypeImage[] getTypes() { return AutoTypeImage.getSpecificTypes(); }

		@Override
		public String getJavaDoc() {
			if( imageFirst )
				return "\t/**\n" +
						"\t * Subtracts a scalar value from each element. Both input and output images can be the same instance.\n" +
						"\t *\n" +
						"\t * @param input The input image. Not modified.\n" +
						"\t * @param value What is subtracted from each element.\n" +
						"\t * @param output The output image. Modified.\n" +
						"\t */";
			else
				return "\t/**\n" +
						"\t * Subtracts each element's value from a scalar. Both input and output images can be the same instance.\n" +
						"\t *\n" +
						"\t * @param value Scalar value\n" +
						"\t * @param input The input image. Not modified.\n" +
						"\t * @param output The output image. Modified.\n" +
						"\t */";
		}

		@Override
		public String getName() {return "minus";}

		@Override
		public String getOperation() {
			if( imageFirst )
				return "((input[indexSrc] "+input.getBitWise()+") - value)";
			else
				return "(value - (input[indexSrc] "+input.getBitWise()+"))";
		}
	}

	interface Template {
		String getJavaDoc();

		String getName();

		String getOperation();
	}

	interface TwoTemplate extends Template {
		String getVariableName();

		boolean isScaleOp();

		boolean isImageFirst();

		AutoTypeImage[] getTypes();
	}

	public static void main( String args[] ) throws FileNotFoundException {
		GeneratePixelMath gen = new GeneratePixelMath();
		gen.generate();
	}
}
