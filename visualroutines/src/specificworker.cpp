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
	this->resize(QDesktopWidget().availableGeometry(this).size() * 0.6);
	 
		//GLVIEWER
	viewer = new GLViewer(glviewer);
	glPointSize(15.0);
	glDisable(GL_LIGHTING);
	viewer->resize(frame->width(), frame->height());
	viewer->showEntireScene();	
	viewer->show();
	
	osgView = new OsgView(this->frameInner);
	show();
	
	qsrand(QTime::currentTime().msec());
	
	
	customPlot->addGraph();
	customPlot->graph(0)->setPen(QPen(Qt::blue)); // line color blue for first graph
	customPlot->graph(0)->setName("Fit Error");
	customPlot->xAxis->setRange(0, 100);
	customPlot->yAxis->setRange(0, 10000);
	
	customPlot->addGraph();
	customPlot->graph(1)->setPen(QPen(Qt::magenta)); // line color blue for first graph
	customPlot->graph(1)->setName("Pose Error");
	customPlot->xAxis->setRange(0, 100);
	customPlot->yAxis->setRange(0, 500);
	
 	for(int i=0; i<100; i++)
		{	xQ.enqueue((double)i);yposeTQ.enqueue((double)i); yposeRQ.enqueue((double)i);}
	
	
	 /////////////////////////////////////////////
   ///             FSPF
   ////////////////////////////////////////////
  PlaneFilter::PlaneFilterParams filterParams;
	filterParams.maxPoints = 2000;
	filterParams.numSamples = 20000;
	filterParams.numLocalSamples = 80;
	filterParams.planeSize = 100;
	filterParams.WorldPlaneSize = 50;
	filterParams.minInlierFraction = 0.8;
	filterParams.maxError = 20;
	filterParams.numRetries = 2;
	filterParams.maxDepthDiff = 1800;
	// Parameters for polygonization
	filterParams.runPolygonization = false;
	filterParams.minConditionNumber = 0.1;
	filterParams.filterOutliers = true;
	planeFilter = new PlaneFilter( filterParams);
	
	
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
			innerViewer = new InnerModelViewer(localInnerModel, "root", osgView->getRootGroup(), false);
			//localInnerModel->print();
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
	
	tabletype = new TableType("vtable_t", localInnerModel);
	
	//timer.setSingleShot(true);
	timer.start(5);

	return true;
}

void SpecificWorker::compute2()
{
	static bool firstTime=true;
	static QVec correctPose;
	static PointSeq groundTruth;
	static QList<QVec> lista;
	static QFile file("out.txt");
	static QTextStream out;
		
	if( firstTime)
	{
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
         return;
		out.setDevice(&file);
		tie(correctPose, groundTruth) = initMapError();
		for(float i=-300; i< 300; i+=10)
			for (float j = -300 ; j < 300; j+=10) 
				lista.append( QVec::vec6( i,0.,j,0.,0.,0.) );
		qDebug() << __FUNCTION__ << "lista creada" << lista.size();
		tie(correctPose, groundTruth) = initMapError();
		firstTime = false;
	}
		
	double error, poseErr;
	QVec  newPose;
	if( lista.isEmpty() == false) 
	{
		newPose = lista.takeFirst() + correctPose;
		tie(error, poseErr) = mapError(newPose, groundTruth, correctPose);
		out << newPose.x() << " " << newPose.z() << " " << error << " " << poseErr;
		out << "\n";
		qDebug() << __FUNCTION__ << newPose << error << poseErr;
	}
	else
	{
		file.close();
		qFatal("All done");
	}
}

void SpecificWorker::compute()
{
		double poseError, error, factor;
		static int iter=0;
		const double poseThreshold = 15;
		const int maxIter = 1000;
		
		tie(poseError, error, factor) = experiment();
		iter++;
		
// 		if( state == State::FIT_TABLE and (error < poseThreshold or factor < 0.001) )
// 		{ 
// 			qDebug() << "Exp. finished in " << iter << "iteraciones" << "error" << poseError << "factor" << factor;
// 			state = State::SENSE;
// 		}
// 		else if( iter > maxIter)
// 		{
// 			qDebug() << "Exp. aborted";
// 			state = State::SENSE;
// 		}
			
}

tuple< double, double, double> SpecificWorker::experiment()
{
 	static Mat gray, grayR, depth, depthR;
	static Mat frame(480, 640, CV_8UC3), frameR(480, 640, CV_8UC3);
 	static PointSeq pointSeq, pointSeqR; 
	float d;
	static  PointSeq sample, pointSeqW, pointSeqWNoise;
	static QVec newPose;
	QImage img;
	QVec offset;
	double  poseErr, factor;
	
	switch( state )
 	{
		case State::INIT:
			//qDebug() << "State::INIT";
			break;
					
		case State::SENSE:
			qDebug() << "State::SENSE";
			std::tie(frame, gray, depth, pointSeq) = this->getImage();
			img = QImage(depth.data, depth.cols, depth.rows, QImage::Format_Indexed8);
			label->setPixmap(QPixmap::fromImage(img).scaled(label->width(), label->height()));
			pointSeqWNoise = table.filterTablePoints(pointSeq, depth, true);
			viewer->setSensedCloud(pointSeqWNoise, QVec::vec3(1,0,0));
			
			// ground truth according to .xml
			correctPose = innerModel->transform6D("world","table_t");
			
			// compute initial search pose
			initialPose = correctPose + table.getInitialPose();
			newPose = initialPose;
			table.setPose( initialPose );
			
			// render model
			std::tie(frameR, grayR, depthR, pointSeqR) = renderAndGenerateImages();
			
			// filter points not belonging to the table
			sample = table.filterTablePoints(pointSeqR, depthR, false);
	
			//Initialize metropolis
			metropolis( 0 , QVec() , true);	
			//table.initMetropolis();
			
			//copy model point cloud to the viewer
			viewer->setCloud(sample, QVec::vec3(0,1,0));

			qDebug() << __FUNCTION__ << "sensed:" << pointSeqWNoise.size() << "model:" << sample.size() << pointSeqR.size();
		
		state = State::FIT_TABLE;
			break;
			
		case State::FIT_TABLE:
			
			poseErr = (correctPose - newPose).norm2();
			
			//compute distance between clouds
 			d = distance( sample, pointSeqWNoise);	
			if( d==0 ) break;
			
 			tie(newPose, factor) = metropolis( d , newPose);	
			//newPose = table.metropolis(d);
			
			table.setPose( newPose );
			
			std::tie(frameR, grayR, depthR, pointSeqR) = renderAndGenerateImages();
			
			sample = table.filterTablePoints(pointSeqR, depthR, false);
			
			tabletype->moveTable(newPose, "world");
			tabletype->render( frameColor, this->label2 );			
			
			//table.projectMeshOnFrame( framecolor, label2);
			
			viewer->setCloud(sample, QVec::vec3(0,1,0));
			
			yposeTQ.enqueue( poseErr );
			yQ.enqueue(d/1000); 
			lcdNumber->display(poseErr);
			//qDebug() << __FUNCTION__ << "cloud distance: "<< d << "pose distance:" << poseErr << "sample size:" << sample.size() << "real size:" << pointSeqWNoise.size();
			break;
	}	
	
	//Make 3D points move
	viewer->resize(this->frame->width(),this->frame->height());
	viewer->updateGL();

	//Draw error signal
	customPlot->graph(0)->setData(xQ.getVector(), yQ.getVector());
	customPlot->graph(1)->setData(xQ.getVector(), yposeTQ.getVector());
	customPlot->replot(QCustomPlot::rpImmediate);
	
	return make_tuple( poseErr, d, factor);
}

tuple< QVec, PointSeq>  SpecificWorker::initMapError()
{
	static Mat gray, depth;
	static Mat frame(480, 640, CV_8UC3);
 	static PointSeq pointSeq, pointSeqWNoise; 
	QImage img;
	
	std::tie(frame, gray, depth, pointSeq) = this->getImage();
	img = QImage(depth.data, depth.cols, depth.rows, QImage::Format_Indexed8);
	label->setPixmap(QPixmap::fromImage(img).scaled(label->width(), label->height()));
	pointSeqWNoise = table.filterTablePoints(pointSeq, depth, false);
	viewer->setSensedCloud(pointSeqWNoise, QVec::vec3(1,0,0));			
	// ground truth according to .xml
	QVec correctPose = innerModel->transform6D("world","table_t");
	return make_tuple(correctPose, pointSeqWNoise);
}

tuple<double, double> SpecificWorker::mapError(const QVec &newPose, const PointSeq &groundTruth, const QVec &correctPose)
{
	static Mat grayR, depthR;
	static Mat frameR(480, 640, CV_8UC3);
 	static PointSeq pointSeqR; 
	PointSeq sample;
	
	table.setPose( newPose );
	std::tie(frameR, grayR, depthR, pointSeqR) = renderAndGenerateImages();		
	sample = table.filterTablePoints(pointSeqR, depthR, false);
	double d = distance( sample, groundTruth);	
	double poseErr = (correctPose - newPose).norm2();
	
	tabletype->moveTable(newPose, "world");
	tabletype->render( frameColor, this->label2 );			
	viewer->setCloud(sample, QVec::vec3(0,1,0));		
	yposeTQ.enqueue( poseErr );
	yQ.enqueue(d/1000); 
	lcdNumber->display(poseErr);
	
	//Make 3D points move
	viewer->resize(this->frame->width(),this->frame->height());
	viewer->updateGL();

	//Draw error signal
	customPlot->graph(0)->setData(xQ.getVector(), yQ.getVector());
	customPlot->graph(1)->setData(xQ.getVector(), yposeTQ.getVector());
	customPlot->replot(QCustomPlot::rpImmediate);
	
	return make_tuple(d, poseErr);
}

////////////////////
/// Primitives
////////////////////

tuple< Mat, Mat, Mat, PointSeq > SpecificWorker::renderAndGenerateImages()
{
	RoboCompRGBD::ColorSeq colorSeq;
	RoboCompRGBD::DepthSeq depthSeq;
	RoboCompRGBD::PointSeq pointSeq;

	IMVCamera cam = innerViewer->cameras["rgbd"];
	const int width = cam.RGBDNode->width;
	const int height = cam.RGBDNode->height;
	const float focal = float(cam.RGBDNode->focal);
	double fovy, aspectRatio, Zn, Zf;
	
	innerViewer->update();
	osgView->autoResize();
	osgView->frame();
	cam.viewerCamera->frame();
	
	if (colorSeq.size() != (uint)(width*height))
	{
		colorSeq.resize ( width*height );
		depthSeq.resize ( width*height );
		pointSeq.resize ( width*height );
	}
	
	RTMat rt = localInnerModel->getTransformationMatrix("root", "rgbd");
	innerViewer->cameras["rgbd"].viewerCamera->getCameraManipulator()->setByMatrix(QMatToOSGMat4(rt));
	cam.viewerCamera->getCamera()->getProjectionMatrixAsPerspective(fovy, aspectRatio, Zn, Zf);

	const unsigned char *rgb = cam.rgb->data();
	const float *d = (float *)cam.d->data();
	
//	#pragma omp parallel
	for (int i=0; i<height; ++i)
	{
		for (int j=0; j<width; ++j)
		{
			const int index  = j + i*width;
			const int indexI = j + (height-1-i)*width;
			colorSeq[index].red   = rgb[3*indexI+0];
			colorSeq[index].green = rgb[3*indexI+1];
			colorSeq[index].blue  = rgb[3*indexI+2];
			if (d[indexI] <= 1.)
			{
				depthSeq[index] = Zn*Zf / ( Zf - d[indexI]* ( Zf-Zn ) );
			}
			else
			{
				depthSeq[i] = NAN;
			}
  		pointSeq[index].x = depthSeq[index] * (float(j)    - (width/2.)) / focal;
			pointSeq[index].y = depthSeq[index] * ((height/2.) -   float(i)) / focal;
			pointSeq[index].z = depthSeq[index];
			pointSeq[index].w = 1.;
		}
	}
	Mat frame(480, 640, CV_8UC3,  &(colorSeq)[0]), greyMat;
	cv::cvtColor(frame, greyMat, cv::COLOR_RGB2GRAY);
	Mat depth_image(480, 640, CV_32FC1,  &(depthSeq)[0]), depth_norm, depth_norm_scaled;
	normalize( depth_image, depth_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
	convertScaleAbs( depth_norm, depth_norm_scaled );
	return std::make_tuple(frame, greyMat, depth_norm_scaled, pointSeq);	
}


QVec SpecificWorker::getInitialSample()
{
	QVec res = QVec::zeros(6); 
	res.inject(QVec::uniformVector(3, -150, 150),0);
//	res.inject(QVec::uniformVector(1, -0.1, 0.1),3);	
//	res.inject(QVec::uniformVector(1, -0.4, 0.4),4);
//	res.inject(QVec::uniformVector(1, -0.1, 0.1),5);	
	res[1]= 0; 
	return res;
}



/**
 * @brief Computes the sum of the minimum distances between the clouds
 * 
 * @param  ...
 * @param  ...
 * @return float
 */

double SpecificWorker::distance(PointSeq orig, PointSeq dest)
{
	double sum=0;
	
	#pragma omp parallel
	for( uint i=0; i<orig.size(); i++ )
	{
		float minDist = std::numeric_limits< float >::max();
		float d;
		for( uint j=0; j< dest.size(); j++ )
		{
				PointXYZ &p = orig[i];
				PointXYZ &q = dest[j];
				d = ((p.x-q.x)*(p.x-q.x) + (p.y-q.y)*(p.y-q.y) + (p.z-q.z)*(p.z-q.z));
				if( d < minDist )
				{
					minDist = d;
				}
		}
		sum += minDist;
	}	
	return sum;
}

tuple< QVec, double> SpecificWorker::metropolis(double error, const QVec &pose, bool reset)
{
	static float errorAnt = std::numeric_limits< float >::max();
	static QVec lastPose = initialPose;
	static float cont = 0;
	static float reps=0, acc=0;
	
	if(reset)
	{
		errorAnt = std::numeric_limits< float >::max();
		lastPose = initialPose;
		cont = 0;
		return make_tuple( QVec(), 0.);
	}
// 	double p = exp(-error/1000);
// 	double pAnt = exp(-errorAnt/1000);
// 	
// 	double ratio = p/pAnt;
// 	qDebug() << __FUNCTION__ << "ratio" << ratio << "p" << p << "pant" << pAnt << "error" << error << "errorAnt" << errorAnt;
	

	if( errorAnt-error > 1 )
	{
		qDebug() <<	 "ACCEPT one, err diff" << errorAnt - error << "ratio" << errorAnt/error << "errsA" << errorAnt << "err" << error;
		errorAnt = error;
		lastPose = pose;
	}
	else
	{	
 		double draw = (double)qrand()/RAND_MAX;
 		if( draw >= fabs(error/errorAnt)/3.5 )
 		{
			qDebug() << __FUNCTION__ << "ACCEPT 2 with draw > fixed ratio" << draw << "ratio" << errorAnt/error << "errsA" << errorAnt << "err" << error;
 			errorAnt = error;
 			lastPose = pose;
			acc+=1;
			
		}
// 		else
// 			qDebug() << __FUNCTION__ << "reject";
	}
	
	// draw a new pose with cooling policy
	double factor = exp(-cont);
	//QVec delta = QVec::uniformVector(3, -300*factor, 300*factor);
	//QVec delta = getRandomOffSet()*(float)factor;
	
	QVec delta = table.getSample(factor);
		
	cont = cont + 0.02;	
	reps+=1;
	double acratio = acc/reps*100.0;
	if (acratio>0) 
		qDebug() << __FUNCTION__ << "ratio percentage:" << (int)acratio << "%";
	return make_tuple(lastPose + delta, factor);
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


/////////////////
/// GUI
////////////////

void SpecificWorker::resetButtonSlot()
{
	state = State::SENSE;
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





// void  SpecificWorker::computeHarrisCorners( const Mat &img, std::vector<cv::Point> &points)
// {
// 	qDebug() << "State::HARRIS";
// 	harrisdetector.detect(img);
// 	harrisdetector.getCorners( points, .01);
// 	qDebug() << __FUNCTION__ << "corners" << points.size();
// }
// 
// std::vector<cv::Point> SpecificWorker::filterTable(const PointSeq &pointsSeq, const std::vector<cv::Point> &points)
// {
// 	std::vector<cv::Point> pointsCopy;
// 	for( auto p : points)
// 	{
// 		int index = p.y *640 + p.x;
// 		QVec floorCoor = innerModel->transform("floor", QVec::vec3( pointsSeq[index].x, pointsSeq[index].y, pointsSeq[index].z), "rgbd");
// 		if(  floorCoor.y() > 700 and floorCoor.y() < 780)
// 				pointsCopy.push_back(p);
// 	}
// 	qDebug() << __FUNCTION__ << "corners" << pointsCopy.size();
// 	return pointsCopy;
// 	
// }
// 
// Points SpecificWorker::cluster(const Points &points, cv::Mat& frame)
// {
// 	// group corners
//   Mat samples( points.size(), 2, CV_32F);
//   for( uint i = 0; i < points.size(); i++ )
// 	{
// 		samples.at<float>(i,0) = points[i].x;
// 		samples.at<float>(i,1) = points[i].y;	//Se podría meter la coor depth
// 	}
//   int clusterCount = 4;
//   Mat labels;
//   int attempts = 5;
//   Mat centers;
//   cv::kmeans( samples, clusterCount, labels, cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 
// 							10000, 0.0001), attempts, KMEANS_PP_CENTERS, centers );
// 	Points plist;
// 	for( int i = 0; i < centers.rows; i++ )
// 	{
// 		cv::Point p(centers.at<float>(i, 0), centers.at<float>(i, 1));
// 		cv::circle(frame, p, 4, cv::Scalar(0,255,0) ,3);
// 		plist.push_back(p);
//   }
// 	return plist;
// }

