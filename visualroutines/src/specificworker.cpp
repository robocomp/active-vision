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
	 
		//GLVIEWER
	viewer = new GLViewer(glviewer);
	glPointSize(5.0);
	glDisable(GL_LIGHTING);
	viewer->resize(frame->width(), frame->height());
	viewer->showEntireScene();	
	viewer->show();
	
	qsrand(QTime::currentTime().msec());
	
	customPlot->addGraph();
	customPlot->graph(0)->setPen(QPen(Qt::blue)); // line color blue for first graph
	customPlot->graph(0)->setName("Error");
	customPlot->xAxis->setRange(0, 100);
	customPlot->yAxis->setRange(0, 10000);
	
 	for(int i=0; i<100; i++)
		xQ.enqueue((double)i);		
	
	connect(resetButton, SIGNAL(clicked()), this, SLOT(resetButtonSlot()));
	connect(startButton, SIGNAL(clicked()), this, SLOT(startButtonSlot()));
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
			localInnerModel = new InnerModel("etc/table.xml");
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
	
	viewer->setGridIsDrawn(true);
	QVec cam = innerModel->transform("world", "rgbd").normalize();
  //viewer->camera()->setViewDirection(qglviewer::Vec(cam.x()+0.2,cam.y(),cam.z()));
	//viewer->camera()->centerScene();
	
	//Objects
	table.setInnerModel(localInnerModel);
	
	tabletype = new TableType("tabletype", localInnerModel);
	
	//timer.setSingleShot(true);
	timer.start(100);

	return true;
}


void SpecificWorker::compute()
{
 	static Mat gray, depth;
	static Mat frame(480, 640, CV_8UC3);
 	static PointSeq pointSeq, tablePoints;
	float d;
	static  PointSeq sample, pointSeqW;
	static QVec newPose;
	QImage img;
	
	
	switch( state )
 	{
		case State::INIT:
			qDebug() << "State::INIT";
			break;
					
		case State::SENSE:
			qDebug() << "State::SENSE";
			std::tie(frame, gray, depth, pointSeq) = this->getImage();
			img = QImage(depth.data, depth.cols, depth.rows, QImage::Format_Indexed8);
			label->setPixmap(QPixmap::fromImage(img).scaled(label->width(), label->height()));
			pointSeqW = filterTablePoints(pointSeq, depth);
			viewer->setSensedCloud(pointSeqW, QVec::vec3(1,0,0));
			initialPose = localInnerModel->transform("world","vtable");
			newPose = localInnerModel->transform("world","vtable") + getRandomOffSet();
			sample = table.renderPose( newPose, pointSeqW);	
			metropolis( 0 , QVec() , true);	
			viewer->setCloud(sample, QVec::vec3(0,1,0));
			state = State::FIT_TABLE;
			break;
			
		case State::FIT_TABLE:
 			d = distance( sample, pointSeqW);
 			qDebug() << __FUNCTION__ << d;
			yQ.enqueue(d/1000); 
			lcdNumber->display(d/1000);
 			newPose = metropolis( d , newPose);	
			sample = table.renderPose( newPose, sample);	
			
			tabletype->moveTable(newPose, "world");
			tabletype->render( frameColor, this->label2 );
			
			viewer->setCloud(sample, QVec::vec3(0,1,0));
			break;
	}	
	
	//Make 3D points move
	viewer->updateGL();
	//cv::waitKey(1);
	//Draw error signal
	customPlot->graph(0)->setData(xQ.getVector(), yQ.getVector());
	customPlot->replot(QCustomPlot::rpImmediate);
	
}

////////////////////
/// Primitives
////////////////////

QVec SpecificWorker::getRandomOffSet()
{
	QVec res = QVec::uniformVector(3, -500, 500);
	res[1]= 0;
	return res;
}


PointSeq SpecificWorker::filterTablePoints(const PointSeq &points, Mat &depth)
{
	PointSeq lp;
	int lowThreshold=50;
	int ratio = 3;
	int kernel_size = 5;
	
	// filter depthimage
  cv::Canny( depth, depth, lowThreshold, lowThreshold*ratio, kernel_size );
	cv::Size size = depth.size();
	
	qDebug() << __FUNCTION__ << depth.depth() << size.width << size.height;
	
	for (int i=0; i< size.height; i+=4) 
	{
		for (int j=0; j< size.width; j+=4) 
		{
			if( depth.at<uchar>(i,j) > 0 )
			{
				PointXYZ p = points[j+i*size.width];
				QVec pw = innerModel->transform("world", QVec::vec3(p.x,p.y,p.z),"rgbd"); 			
				
				//GET convez hull from innermodel!!!
				if( pw.y()>100 and pw.z()<1000 )
				{
					PointXYZ pn = { pw.x(),pw.y(),pw.z(),0 };
					lp.push_back(pn);
				}
			}
		}
	}
	qDebug() << __FUNCTION__ << lp.size();
	
  //imshow( "canny", depth );
	return lp;
}

/**
 * @brief Computes the sum of the minimum distances between the clouds
 * 
 * @param  ...
 * @param  ...
 * @return float
 */
float SpecificWorker::distance(PointSeq orig, PointSeq dest)
{
	double sum=0;
	float minDist = std::numeric_limits< float >::max();
	float d;
	
	//qDebug() << __FUNCTION__ << orig.size() << dest.size();
	for( auto p : orig )
	{
		for( auto q: dest )
		{
				d = ((p.x-q.x)*(p.x-q.x) + (p.y-q.y)*(p.y-q.y) + (p.z-q.z)*(p.z-q.z));
				if( d < minDist )
					minDist = d;
		}
		sum += minDist;
		minDist = std::numeric_limits< float >::max();
	}
	return sum;
}

QVec SpecificWorker::metropolis(float error, const QVec &pose, bool reset)
{
	static float errorAnt = std::numeric_limits< float >::max();
	static QVec lastPose = initialPose;
	static float cont = 0;
	
	if(reset)
	{
		errorAnt = std::numeric_limits< float >::max();
		lastPose = initialPose;
		cont = 0;
		return QVec();
	}
	
	double p = exp(-error/1000);
	double pAnt = exp(-errorAnt/1000);
	
	double ratio = p/pAnt;
	//qDebug() << __FUNCTION__ << "ratio" << ratio <<p << pAnt;
	if( ratio >= 1 )
	{
		errorAnt = error;
		lastPose = pose;
		//qDebug() << __FUNCTION__ << "accetp";
	}
	else
	{	
		double draw = (double)qrand()/RAND_MAX;
		if( draw < ratio )
		{
			errorAnt = error;
			lastPose = pose;
			//qDebug() << __FUNCTION__ << "accetp with draw > ratio" << draw;
		}
		//else
			//qDebug() << __FUNCTION__ << "reject";
	}
	
	// draw a new pose with cooling policy
	double factor = exp(-cont);
	QVec delta = QVec::uniformVector(3, -300*factor, 300*factor);
	cont = cont + 0.05;
	
	delta[1]=0;
	return lastPose + delta;
}


std::tuple<Mat, Mat, Mat, PointSeq> SpecificWorker::getImage()
{
	//qDebug() << "State::GETIMAGE";

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
		cv::cvtColor(frame, frameColor, cv::COLOR_BGR2RGB);
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

void SpecificWorker::resetButtonSlot()
{
	state = State::INIT;
}

void SpecificWorker::startButtonSlot()
{
	state = State::SENSE;
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
