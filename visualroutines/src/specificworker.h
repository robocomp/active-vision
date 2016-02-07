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
#include "tableobject.h"
#include <QGLViewer/qglviewer.h>
#include "glviewer.h"
#include <qcustomplot.h>
#include "deque.h"
#include <osgviewer/osgview.h>
#include <innermodel/innermodelviewer.h>
#include <omp.h>

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
	HarrisDetector harrisdetector;
	Points cluster(const Points &points, cv::Mat& frame);
	std::vector<cv::Point> filterTable(const PointSeq &pointsSeq, const std::vector<cv::Point> &points);

public slots:
	void compute(); 	
	void resetButtonSlot();
	void startButtonSlot();
	

private:
	InnerModel *innerModel, *localInnerModel;
	
// 	Mat canny(const Mat &img);
// 	Mat hough(const Mat &img);
	void initMachine();
	float distance(PointSeq, PointSeq);
	QVec metropolis( float error, const QVec& pose, bool reset=false);
	RoboCompRGBD::PointSeq filterTablePoints(const PointSeq& points, const Mat& depth, bool addNoise = false);
	QVec getRandomOffSet();
	QVec getInitialSample();
	tuple< Mat, Mat, Mat, PointSeq > renderAndGenerateImages();
	
	enum class State {INIT, FIT_TABLE, SENSE};
	State state = State::INIT;
	
	GLViewer *viewer;
	TableObject table;
	QVec correctPose, initialPose;
	Deque<double> xQ,yQ,yposeTQ, yposeRQ;
	
	TableType *tabletype;
	
	Mat frameColor, frameColor2;
	
	OsgView *osgView;
	InnerModelViewer *innerViewer;
};

#endif

