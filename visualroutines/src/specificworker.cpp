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
	//timer.setSingleShot(true);
	timer.start(Period);

	return true;
}


void SpecificWorker::compute()
{
	static Mat gray, depth, frame;
	static std::vector<cv::Point> points;
	static PointSeq pointSeq;
	
	switch( state )
	{
		case State::INIT:
			qDebug() << "State::INIT";
			state = State::GETIMAGE;
			break;
		case State::GETIMAGE:
			tie(frame, gray, depth, pointSeq) = getImage();
			//imshow("Color", frame);
			state = State::HARRIS;
			break;
		case  State::HARRIS:
			computeHarrisCorners( gray, points );
			computeHarrisCorners( depth, points );
			state = State::FILTER_TABLE_HEIGHT;
			break;
		case State::FILTER_TABLE_HEIGHT:
			points = filterTable( pointSeq , points);
			state = State::DRAW;
			break;
		case  State::DRAW:
			qDebug() << "State::Draw";
			harrisdetector.drawOnImage( frame, points);
			imshow("Harris", frame);
			state = State::STOP;
			break;
		case  State::STOP:
			qDebug() << "State::STOP";
			break;
	}

	

}

std::tuple<Mat, Mat, Mat, PointSeq> SpecificWorker::getImage()
{
	qDebug() << "State::GETIMAGE";

	static RoboCompDifferentialRobot::TBaseState bState;
    static RoboCompJointMotor::MotorStateMap hState;
	static RoboCompRGBD::ColorSeq colorSeq;
	static RoboCompRGBD::DepthSeq depthSeq;
	static RoboCompRGBD::PointSeq pointSeq;
	
	try
	{
		rgbd_proxy->getImage(colorSeq, depthSeq, pointSeq, hState, bState);
		Mat depth(480, 640, CV_32FC1,  &(depthSeq)[0]), depth_norm, depth_norm_scaled;
		Mat frame(480, 640, CV_8UC3,  &(colorSeq)[0]), greyMat;
		normalize( depth, depth_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
		convertScaleAbs( depth_norm, depth_norm_scaled );
		cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
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


/*
Mat SpecificWorker::canny(const Mat &img)
{
	
		 /// Canny detector
		Mat dst, detected_edges;
		int lowThreshold = 50;
		int highThreshold = 150;
		int kernel_size = 3;
		Canny( img, detected_edges, lowThreshold, highThreshold, kernel_size );

		/// Using Canny's output as a mask, we display our result
		dst = Scalar::all(0);
		img.copyTo( dst, detected_edges);
		return detected_edges;
	
}

Mat SpecificWorker::hough(const Mat &img)
{
	vector<Vec2f> lines;
	Mat cdst;
	
	Mat detected_edges = canny(img);
	HoughLines(detected_edges, lines, 1, CV_PI/180, 70, 0, 0 );
	cvtColor(img, cdst, CV_GRAY2BGR); 
	
		// draw lines
		for( auto l : lines )
		{
			float rho = l[0], theta = l[1];
			Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a));
			line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
		}
	return cdst;
}
*/


//imshow("Depth", depth_norm_scaled);
		//	imshow("Canny", dst);
		//    imshow("Hough", cdst);
		
	
		//         QImage img = QImage(&rgbMatrix[0], 640, 480, QImage::Format_RGB888);
		//         label->setPixmap(QPixmap::fromImage(img));
		//         label->resize(label->pixmap()->size());
		// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
