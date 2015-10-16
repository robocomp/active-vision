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
	//namedWindow( "Display window", 1 );// Create a window for display.
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
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
	static RoboCompDifferentialRobot::TBaseState bState;
    static RoboCompJointMotor::MotorStateMap hState;
    static RoboCompRGBD::imgType rgbMatrix;
    static RoboCompRGBD::depthType distanceMatrix;
	
	try
	{
		rgbd_proxy->getData(rgbMatrix,distanceMatrix, hState, bState);
        
        Mat frame(480, 640, CV_8UC3,  &(rgbMatrix)[0]);
		cv::Mat greyMat;
		cv::cvtColor(frame, greyMat, cv::COLOR_BGR2GRAY);
		
		Mat dst, dst_norm, dst_norm_scaled;
		dst = Mat::zeros( greyMat.size(), CV_32FC1 );

		/// Detector parameters
		int blockSize = 2;
		int apertureSize = 3;
		double k = 0.04;

		/// Detecting corners
		cornerHarris( greyMat, dst, blockSize, apertureSize, k, BORDER_DEFAULT );
		
		/// Normalizing
		normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
		convertScaleAbs( dst_norm, dst_norm_scaled );
		
        imshow("3D viewer",dst_norm_scaled);
		
        
//         QImage img = QImage(&rgbMatrix[0], 640, 480, QImage::Format_RGB888);
//         label->setPixmap(QPixmap::fromImage(img));
//         label->resize(label->pixmap()->size());
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
		
	}
	catch(const Ice::Exception &e)
	{	std::cout << "Error reading from Camera" << e << std::endl;	}
}







