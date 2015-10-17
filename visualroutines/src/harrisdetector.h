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

/*------------------------------------------------------------------------------------------*\
   This file contains material supporting chapter 8 of the cookbook:  
   Computer Vision Programming using the OpenCV Library. 
   by Robert Laganiere, Packt Publishing, 2011.
   This program is free software; permission is hereby granted to use, copy, modify, 
   and distribute this source code, or portions thereof, for any purpose, without fee, 
   subject to the restriction that the copyright notice may not be removed 
   or altered from any source or altered source distribution. 
   The software is released on an as-is basis and without any warranties of any kind. 
   In particular, the software is not guaranteed to be fault-tolerant or free from failure. 
   The author disclaims all warranties with regard to this software, any use, 
   and any consequent failure, is purely the responsibility of the user.
 
   Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name
\*------------------------------------------------------------------------------------------*/


#ifndef HARRISDETECTOR_H
#define HARRISDETECTOR_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

class HarrisDetector {

  private:

	  // 32-bit float image of corner strength
	  cv::Mat cornerStrength;
	  // 32-bit float image of thresholded corners
	  cv::Mat cornerTh;
	  // image of local maxima (internal)
	  cv::Mat localMax;
	  // size of neighbourhood for derivatives smoothing
	  int neighbourhood; 
	  // aperture for gradient computation
	  int aperture; 
	  // Harris parameter
	  double k;
	  // maximum strength for threshold computation
	  double maxStrength;
	  // calculated threshold (internal)
	  double threshold;
	  // size of neighbourhood for non-max suppression
	  int nonMaxSize; 
	  // kernel for non-max suppression
	  cv::Mat kernel;

  public:

	HarrisDetector();
	virtual ~HarrisDetector(){};
	void setLocalMaxWindowSize(int size);
	void detect(const cv::Mat& image);
	cv::Mat getCornerMap(double qualityLevel);
	void getCorners(std::vector<cv::Point> &points, const cv::Mat& cornerMap); 
	void getCorners(std::vector<cv::Point> &points, double qualityLevel);
	void drawOnImage(cv::Mat &image, const std::vector<cv::Point> &points, cv::Scalar color= cv::Scalar(255,0,0), int radius=3, int thickness=2);

};



#endif // HARRISDETECTOR_H
