/*
 * Copyright 2015 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "harrisdetector.h"

HarrisDetector::HarrisDetector() : neighbourhood(3), aperture(3), k(0.04), maxStrength(0.0), threshold(0.01), nonMaxSize(7) 
{  
	setLocalMaxWindowSize(nonMaxSize);
}

// Create kernel used in non-maxima suppression
void HarrisDetector::setLocalMaxWindowSize(int size) 
{
	nonMaxSize= size;
	kernel.create(nonMaxSize,nonMaxSize,CV_8U);
}

// Compute Harris corners
void HarrisDetector::detect(const cv::Mat& image) 
{
	// Harris computation
	cv::cornerHarris(image, cornerStrength, neighbourhood, aperture, k);           // Harris parameter
	
	// internal threshold computation
	double minStrength; // not used
	cv::minMaxLoc(cornerStrength, &minStrength, &maxStrength);

	// local maxima detection
	cv::Mat dilated;  // temporary image
	cv::dilate(cornerStrength, dilated, cv::Mat());
	cv::compare(cornerStrength, dilated, localMax,cv::CMP_EQ);
}

// Get the corner map from the computed Harris values
cv::Mat HarrisDetector::getCornerMap(double qualityLevel) 
{

  cv::Mat cornerMap;

  // thresholding the corner strength
  threshold= qualityLevel*maxStrength;
  cv::threshold(cornerStrength,cornerTh,threshold,255,cv::THRESH_BINARY);

  // convert to 8-bit image
  cornerTh.convertTo(cornerMap,CV_8U);

  // non-maxima suppression
  cv::bitwise_and(cornerMap,localMax,cornerMap);

  return cornerMap;
}

// Get the feature points vector from the computed Harris values
void HarrisDetector::getCorners(std::vector<cv::Point> &points, double qualityLevel) 
{

  // Get the corner map
  cv::Mat cornerMap= getCornerMap(qualityLevel);
  // Get the corners
  getCorners(points, cornerMap);
}

// Get the feature points vector from the computed corner map
void HarrisDetector::getCorners(std::vector<cv::Point> &points, const cv::Mat& cornerMap) 
{
			  
  // Iterate over the pixels to obtain all feature points
  for( int y = 0; y < cornerMap.rows; y++ ) 
  {
  	  const uchar* rowPtr = cornerMap.ptr<uchar>(y);
  	  for( int x = 0; x < cornerMap.cols; x++ ) 
	  {
		  // if it is a feature point
		  if (rowPtr[x]) 
		  {
			  points.push_back(cv::Point(x,y));
		  }
	  } 
  }
}

// Draw circles at feature point locations on an image
 void HarrisDetector::drawOnImage(cv::Mat &image, const std::vector<cv::Point> &points, cv::Scalar color, int radius, int thickness) 
 {
	  std::vector<cv::Point>::const_iterator it= points.begin();

	  // for all corners
	  while (it!=points.end()) 
	  {
		// draw a circle at each corner location
		cv::circle(image,*it,radius,color,thickness);
		++it;
	  } 
}