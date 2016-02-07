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

#include "tabletype.h"
#include "specificworker.h"

TableType::TableType(QString _name, InnerModel *_innerModel): height(700), length(900), width(1400), 
																															topThickness(80), legWidth(90), innerModel(_innerModel), name(_name)
{
	//Creamos unos transforms virtuales en el mundo qe representaran nuestra cog-mesa, De momento cogemos t_table del .xml como origen
	//innerModel->newTransform(name, "static", innerModel->getNode("floor"), 0, 0, 900, 0, 0, 0);
	
	//innerModel->newTransform(name, "static", innerModel->getNode("floor"), 200, 0, 1050, 0, 0, 0);
	
	innerModel->newTransform("vtable_top", "static", innerModel->getNode(name), 0, this->height, 0, 0, 0, 0);
	//innerModel->newTransform("vtable_down", "static", innerModel->getNode("vtable_top"), 0, -this->topThickness, 0, 0, 0, 0);
	
	tabletop = new TabletopType("top", "vtable_top", innerModel, QVec::vec3(0,0,0));
	
	legs.push_back( new LegType("leg0", "vtable_top", innerModel, QVec::vec3(-width/2+legWidth/2, -topThickness, -length/2+legWidth/2 )));
	legs.push_back( new LegType("leg1", "vtable_top", innerModel, QVec::vec3(+width/2-legWidth/2, -topThickness, -length/2+legWidth/2 )));
	legs.push_back( new LegType("leg2", "vtable_top", innerModel, QVec::vec3(-width/2+legWidth/2, -topThickness, length/2-legWidth/2 )));
	legs.push_back( new LegType("leg3", "vtable_top", innerModel, QVec::vec3(+width/2-legWidth/2, -topThickness, length/2-legWidth/2 )));
}

TableType::TableType(const TableType& other)
{
}

TableType::~TableType()
{
}

/**
 * @brief Create a new sample of the table and compute a set of 3D points along its mesh
 * 
 * @return RoboCompRGBD::PointSeq
 */
PointSeq TableType::newSample()
{

}

void TableType::update(SpecificWorker *handler)
{
// 	static Mat gray, depth, frame;
// 	static RoboCompRGBD::PointSeq pointsSeq;
// 	static std::vector<cv::Point> points;
// 	static Points clusters;
// 	cv::Point currentPoint;
// 	static QPoints clusters3D;
// 	static std::vector<float> temp;
// 	
// 	static float minCluster, minTable;
// 	
// 	switch( state )
// 	{
// 		case State::INIT:
// 			//qDebug() << "State::INIT";
// 			state = State::GET_IMAGE;
// 			break;
// 			
// 		case State::GET_IMAGE:
// 			qDebug() << "State::INIT";
// 			tie(frame, gray, depth, pointsSeq) = handler->getImage();
// 			state = State::HARRIS;
// 			break;
// 			
// 		case  State::HARRIS:
// 			handler->computeHarrisCorners( gray, points );
// 			handler->computeHarrisCorners( depth, points );
// 			// executive.do("HARRIS");
// 			state = State::FILTER_TABLE_HEIGHT;
// 			break;
// 			
// 		case State::FILTER_TABLE_HEIGHT:
// 			points = handler->filterTable( pointsSeq , points);
// 			state = State::DRAW_HARRIS;
// 			//state = State::RENDER_TABLE;
// 			break;
// 				
//  		case  State::DRAW_HARRIS:
//  			qDebug() << "State::Draw";
//  			handler->harrisdetector.drawOnImage( frame, points);
//  			state = State::CLUSTER;
//  			break;
// 			
// 		case State::CLUSTER:
// 			clusters = handler->cluster(points, frame);
// 			if (clusters.size() == 4)
// 			{
// 				for( auto c: clusters )
// 				{
// 					int index = c.y *640 + c.x;
// 					clusters3D.push_back( innerModel->transform("floor", QVec::vec3( pointsSeq[index].x, pointsSeq[index].y, pointsSeq[index].z), "rgbd"));
// 				}
// 				state = State::SELECT_FIRST_CORNER;
// 			}
// 			else
// 			{
// 				qDebug() << "Could not find four clusters. STOP";
// 				state = State::STOP;
// 			}
// 			break;
// 			
// 		//Pick the cluster-corner pair with minimun distance 
// 		case State::SELECT_FIRST_CORNER:
// 			qDebug() << "State::SELECT_FIRST_CORNER";
// 			tie(minCluster, minTable) = selectFirstCorner(clusters3D);	 
// 			cv::circle(frame, cv::Point(innerModel->project( "floor", clusters3D[minCluster], "rgbd").x(),
// 																	innerModel->project( "floor", clusters3D[minCluster], "rgbd").y()),
// 																	5, cv::Scalar(255,0,255) ,3);
// 			cv::circle(frame, cv::Point(innerModel->project( "floor", tabletop->getCorners("floor")[minTable], "rgbd").x(),
// 																	innerModel->project( "floor", tabletop->getCorners("floor")[minTable], "rgbd").y()),
// 																	5, cv::Scalar(255,0,2559) ,3);
// 
// 			//moveTable(minTable, clusters3D[minCluster], "floor");
// 			
// 			
// 			//tabletop->moveCornerTo(minTable, clusters3D[minCluster], "floor");
// 			clusters3D.erase(clusters3D.begin()+minCluster);
// // 			state = State::SELECT_SECOND_CORNER;
// 			//render(handler, frame);
// 			
// 			state = State::STOP;
// 			break;
// 
// 				//Pick the second cluster-corner pair with minimun distance 
// 		case State::SELECT_SECOND_CORNER:
// 			qDebug() << "State::SELECT_SECOND_CORNER";
// 			tie(minCluster, minTable) = selectFirstCorner(clusters3D);	 
// 			cv::circle(frame, cv::Point(innerModel->project( "floor", clusters3D[minCluster], "rgbd").x(),
// 																	innerModel->project( "floor", clusters3D[minCluster], "rgbd").y()),
// 																	5, cv::Scalar(55,100,255) ,3);
// 			cv::circle(frame, cv::Point(innerModel->project( "floor", tabletop->getCorners("floor")[minTable], "rgbd").x(),
// 																	innerModel->project( "floor", tabletop->getCorners("floor")[minTable], "rgbd").y()),
// 																	5, cv::Scalar(55,100,255) ,3);
// 			
// 			tabletop->moveCornerTo(minTable, clusters3D[minCluster], "floor");
// 			clusters3D.erase(clusters3D.begin()+minCluster);
// 		
// 			state = State::STOP;
// 			break;
// 
// 			
// 		case State::RENDER_TABLE:	
// // 			qDebug() << "State::RENDER_TABLE";
// 				//render(handler, frame);
// 			break;
// 
// 		case  State::STOP:
// 			qDebug() << "State::STOP";
// 			//render(handler, frame);
// 			break;
// 	}
 }

void TableType::moveTable(const QVec& pos, const QString& parent)
{
	Q_ASSERT(pos.size()==6);

	//QVec t = innerModel->transform(name, pos, parent) - tabletop->getCorners(name)[corner];
	//t = t + innerModel->transform("floor",name);
	innerModel->updateTransformValues(name,pos.x(),pos.y(),pos.z(),pos.rx(), pos.ry(), pos.rz(),parent);
}

// void TableType::moveTable(uint corner, const QVec& pos, const QString& parent)
// {
// 	Q_ASSERT(pos.size()==3);
// 
// 	QVec t = innerModel->transform(name, pos, parent) - tabletop->getCorners(name)[corner];
// 	t = innerModel->transform("vtable_top", pos, parent);
// 	tabletop->makeItLong(corner, t.z());
// 	//tabletop->makeItWider(t.x());
// 	
// 	//innerModel->updateTranslationValues(name,t.x(),t.y(),t.z(),"floor");
// }


std::tuple<int, int> TableType::selectFirstCorner(const QPoints &clusters3D)
{
	qDebug() << __FUNCTION__;
	QPoints qp = tabletop->getCorners("floor");
	float minDist = std::numeric_limits< float >::max(), minCluster=0, minTable=0;
	uint i=0,j=0;
	
	for(i=0; i < clusters3D.size(); i++)
		for(j=0; j<qp.size(); j++)
		{
			float d = (clusters3D[i]-qp[j]).norm2();
			//qDebug() << __FUNCTION__ << clusters3D[i] << qp[j] << "dist" << d;
			if ( d < minDist)
			{
				minDist = d;
				minCluster = i;
				minTable = j;
			}
		}
	//qDebug() << __FUNCTION__ << "harris" << minCluster << "table" << minTable;
	return std::make_tuple(minCluster,minTable);
}

void TableType::render(const cv::Mat& frame, QLabel *label)
{	
	qDebug() << __FUNCTION__ ;
	std::vector< std::vector < cv::Point> > lines;
	tabletop->render(lines);
	
	for( auto l : legs)
		l->render(lines);
	
	//pintamos todas la líneas sobre la imagen
	Mat m = frame.clone();
	cv::polylines(m, lines, false, cv::Scalar(0,0,200));

 	QImage img(m.data, m.cols, m.rows, QImage::Format_RGB888);
 	label->setPixmap(QPixmap::fromImage(img).scaled(label->width(), label->height()));
	//label->setPixmap(QPixmap::fromImage(img));
}

