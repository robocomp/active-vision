/*
 *    Copyright (C) 2015 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "harrisdetector.h"
#include "tabletype.h"


using namespace cv;
typedef std::vector<cv::Point> Points;


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	// PRIMITIVES
	tuple< Mat, Mat, Mat, PointSeq > getImage();
	void  computeHarrisCorners(const Mat& img, Points &points);
	vector< Point > filterTable(const PointSeq &pointSeq, const Points &points);
	HarrisDetector harrisdetector;
	Points cluster(const Points &points, cv::Mat& frame);

public slots:
	void compute(); 	
	void checkTableButton();
	

private:
	InnerModel *innerModel;
	
// 	Mat canny(const Mat &img);
// 	Mat hough(const Mat &img);
	void initMachine();
	float distance(PointSeq, PointSeq);
	QVec metropolis( float);

	//QStateMachine machine;
	
	//enum class State {INIT, GET_IMAGE, HARRIS, STOP, FILTER_TABLE_HEIGHT, DRAW_HARRIS, RENDER_TABLE};
	enum class State {INIT, TRY_TABLE, FIT_TABLE, SENSE};
	State state = State::INIT;
	
	TableType *table;
};

#endif

