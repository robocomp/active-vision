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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	//initMachine();
	connect(tableButton, SIGNAL(clicked()), this, SLOT(checkTableButton()));
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
// 		for(auto p : params)
// 			std::cout << p.first << p.second.value <<  std::endl;
		
		RoboCompCommonBehavior::Parameter par = params.at("VisualRoutines.InnerModel");
		qDebug() << QString::fromStdString(par.value);
		
		
		if( QFile::exists(QString::fromStdString(par.value)) )
		{
			innerModel = new InnerModel(par.value);
// 			#ifdef USE_QTGUI
// 			innerViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), true);
// 			show();
// 			#endif
		}
		else
		{ std::cout << "Innermodel path " << par.value << " not found. "; qFatal("Abort");	}
	}
	catch(std::exception e)
	{
		qFatal("Error reading config params");
	}
	
	//Objects
	table = new TableType("vtable", innerModel);
	
	
	//timer.setSingleShot(true);
	timer.start(Period);

	return true;
}


void SpecificWorker::compute()
{
 	static Mat gray, depth, frame;
// 	static std::vector<cv::Point> points;
 	static PointSeq pointSeq;

	float d;
	PointSeq sample;
	QVec best;
	
	switch( state )
 	{
		case State::INIT:
			//qDebug() << "State::INIT";
			state = State::SENSE;
			break;
			
		case State::TRY_TABLE:
			//qDebug() <<__FUNCTION__ << "State::TRY_TABLE";
			table->update(this);
			/*if (table->state != TableType::State::STOP)
				table->update(this);
			else
				state  = State::INIT;
			*/
			break;
			
		case State::SENSE:
			std::tie(frame, gray, depth, pointSeq) = this->getImage();
			state = State::FIT_TABLE;
			break;
			
		case State::FIT_TABLE:
			sample = table->newSample();
			d = distance( sample, pointSeq);
			best = metropolis( d );
			//draw something
			break;
	}	
}

////////////////////
/// Primitives
////////////////////

float SpecificWorker::distance(PointSeq, PointSeq)
{

}

QVec SpecificWorker::metropolis(float)
{

}


std::tuple<Mat, Mat, Mat, PointSeq> SpecificWorker::getImage()
{
	qDebug() << "State::GETIMAGE";

	RoboCompDifferentialRobot::TBaseState bState;
  RoboCompJointMotor::MotorStateMap hState;
	RoboCompRGBD::ColorSeq colorSeq;
	RoboCompRGBD::DepthSeq depthSeq;
	RoboCompRGBD::PointSeq pointSeq;
	
	try
	{
		rgbd_proxy->getImage(colorSeq, depthSeq, pointSeq, hState, bState);
		Mat depth(480, 640, CV_32FC1,  &(depthSeq)[0]), depth_norm, depth_norm_scaled;
		Mat frame(480, 640, CV_8UC3,  &(colorSeq)[0]), greyMat;
		normalize( depth, depth_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
		convertScaleAbs( depth_norm, depth_norm_scaled );
		//cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
		cv::cvtColor(frame, greyMat, cv::COLOR_RGB2GRAY);
		return std::make_tuple(frame, greyMat, depth_norm_scaled, pointSeq);
		
	}
	catch(const Ice::Exception &e)
	{	std::cout << "Error reading from Camera" << e << std::endl;	}
	
	return make_tuple( Mat(), Mat(), Mat(), PointSeq());
}

void  SpecificWorker::computeHarrisCorners( const Mat &img, std::vector<cv::Point> &points)
{
	qDebug() << "State::HARRIS";
	harrisdetector.detect(img);
	harrisdetector.getCorners( points, .01);
	qDebug() << __FUNCTION__ << "corners" << points.size();
}

std::vector<cv::Point> SpecificWorker::filterTable(const PointSeq &pointsSeq, const std::vector<cv::Point> &points)
{
	std::vector<cv::Point> pointsCopy;
	for( auto p : points)
	{
		int index = p.y *640 + p.x;
		QVec floorCoor = innerModel->transform("floor", QVec::vec3( pointsSeq[index].x, pointsSeq[index].y, pointsSeq[index].z), "rgbd");
		if(  floorCoor.y() > 700 and floorCoor.y() < 780)
				pointsCopy.push_back(p);
	}
	qDebug() << __FUNCTION__ << "corners" << pointsCopy.size();
	return pointsCopy;
	
}

Points SpecificWorker::cluster(const Points &points, cv::Mat& frame)
{
	// group corners
  Mat samples( points.size(), 2, CV_32F);
  for( uint i = 0; i < points.size(); i++ )
	{
		samples.at<float>(i,0) = points[i].x;
		samples.at<float>(i,1) = points[i].y;	//Se podría meter la coor depth
	}
  int clusterCount = 4;
  Mat labels;
  int attempts = 5;
  Mat centers;
  cv::kmeans( samples, clusterCount, labels, cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 
							10000, 0.0001), attempts, KMEANS_PP_CENTERS, centers );
	Points plist;
	for( int i = 0; i < centers.rows; i++ )
	{
		cv::Point p(centers.at<float>(i, 0), centers.at<float>(i, 1));
		cv::circle(frame, p, 4, cv::Scalar(0,255,0) ,3);
		plist.push_back(p);
  }
	return plist;
}
/////////////////
/// GUI
////////////////

void SpecificWorker::checkTableButton()
{
	state = State::TRY_TABLE;
}


///FUSCA

// 	switch( state )
// 	{
// 		case State::INIT:
// 			qDebug() << "State::INIT";
// 			state = State::GET_IMAGE;
// 			break;
// 			
// 		case State::GET_IMAGE:
// 			tie(frame, gray, depth, pointSeq) = getImage();
// 			state = State::HARRIS;
// 			break;
// 			
// 		case  State::HARRIS:
// 			computeHarrisCorners( gray, points );
// 			computeHarrisCorners( depth, points );
// 			state = State::FILTER_TABLE_HEIGHT;
// 			break;
// 			
// 		case State::FILTER_TABLE_HEIGHT:
// 			points = filterTable( pointSeq , points);
// 			//state = State::DRAW_HARRIS;
// 			state = State::RENDER_TABLE;
// 			break;
// 			
// 		case  State::DRAW_HARRIS:
// 			qDebug() << "State::Draw";
// 			harrisdetector.drawOnImage( frame, points);
// 			imshow("Harris", frame);
// 			state = State::RENDER_TABLE;
// 			break;
// 			
// 		case State::RENDER_TABLE:	
// 			qDebug() << "State::RENDER_TABLE";
// 			table->render( frame);
// 			imshow("Cog-Table", frame);
// 			state = State::STOP;
// 			break;
// 		case  State::STOP:
// 			qDebug() << "State::STOP";
// 			break;
// 	}
