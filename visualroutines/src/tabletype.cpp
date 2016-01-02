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

TableType::TableType(QString _name, InnerModel *_innerModel): height(700), length(900), width(1400), topThickness(80), legWidth(90), innerModel(_innerModel), name(_name)
{
	//Creamos unos transforms virtuales en el mundo qe representaran nuestra cog-mesa, De momento cogemos t_table del .xml como origen
	innerModel->newTransform(name, "static", innerModel->getNode("t_table"), 0, 0, 0, 0, 0, 0);
	innerModel->newTransform("vtable_top", "static", innerModel->getNode("vtable"), 0, this->height, 0, 0, 0, 0);
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

void TableType::update(SpecificWorker *handler)
{
	static Mat gray, depth, frame;
	static RoboCompRGBD::PointSeq pointsSeq;
	static std::vector<cv::Point> points;
	static Points clusters;
	QImage *img;
	cv::Point currentPoint;
	static QPoints clusters3D;
	static std::vector<float> temp;
	
	static float minCluster, minTable;
	
	switch( state )
	{
		case State::INIT:
			//qDebug() << "State::INIT";
			state = State::GET_IMAGE;
			break;
			
		case State::GET_IMAGE:
			qDebug() << "State::INIT";
			tie(frame, gray, depth, pointsSeq) = handler->getImage();
			state = State::HARRIS;
			break;
			
		case  State::HARRIS:
			handler->computeHarrisCorners( gray, points );
			handler->computeHarrisCorners( depth, points );
			// executive.do("HARRIS");
			state = State::FILTER_TABLE_HEIGHT;
			break;
			
		case State::FILTER_TABLE_HEIGHT:
			points = handler->filterTable( pointsSeq , points);
			state = State::DRAW_HARRIS;
			//state = State::RENDER_TABLE;
			break;
				
 		case  State::DRAW_HARRIS:
 			qDebug() << "State::Draw";
 			handler->harrisdetector.drawOnImage( frame, points);
 			//imshow("Harris", frame);
 			state = State::CLUSTER;
 			break;
			
		case State::CLUSTER:
			clusters = handler->cluster(points, frame);
			if (clusters.size() == 4)
			{
				for( auto c: clusters )
				{
					int index = c.y *640 + c.x;
					clusters3D.push_back( innerModel->transform("floor", QVec::vec3( pointsSeq[index].x, pointsSeq[index].y, pointsSeq[index].z), "rgbd"));
				}
				state = State::SELECT_FIRST_CORNER;
			}
			else
			{
				qDebug() << "Could not find four clusters. STOP";
				state = State::STOP;
			}
			break;
			
		//Pick the cluster-corner pair with minimun distance 
		case State::SELECT_FIRST_CORNER:
			tie(minCluster, minTable) = selectFirstCorner(clusters3D);	 
			cv::circle(frame, cv::Point(innerModel->project( "floor", clusters3D[minCluster], "rgbd").x(),
																	innerModel->project("floor", clusters3D[minCluster], "rgbd").y()),
																	5, cv::Scalar(255,0,255) ,3);
			state = State::RENDER_TABLE;
			break;
			
					
		case State::RENDER_TABLE:	
			qDebug() << "State::RENDER_TABLE";
			this->render( frame);
			//imshow("Cog-Table", frame);
			img = new QImage(frame.data, frame.cols, frame.rows, QImage::Format_RGB888);
			handler->label->setPixmap(QPixmap::fromImage(*img).scaled(handler->label->width(), handler->label->height()));
			delete img;
			state = State::STOP;
			break;

		case  State::STOP:
			qDebug() << "State::STOP";
			break;
	}
	
}

std::tuple<int, int> TableType::selectFirstCorner(const QPoints &clusters3D)
{
	QPoints qp = tabletop->getCorners();
	float minDist = std::numeric_limits< float >::max(), minCluster=0, minTable=0;
	int i=0,j=0;
	
	for(i=0; i < clusters3D.size(); i++)
		for(j=0; j<qp.size(); j++)
		{
			float d = (clusters3D[i]-qp[j]).norm2();
			if ( d < minDist)
			{
				minDist = d;
				minCluster = i;
				minTable = j;
			}
		}
	return std::make_tuple(minCluster,minTable);
}

void TableType::render(cv::Mat& frame)
{
	std::vector< std::vector < cv::Point> > lines;
	tabletop->render(lines);
	
	for( auto l : legs)
		l->render(lines);
	
	//pintamos todas la líneas sobre la imagen
	cv::polylines(frame, lines, false, cv::Scalar(0,0,200));
		
}

//recorremos el tablero muesteando puntos cada 25mm (step)
// 	std::vector< std::vector < cv::Point> > lines;
// 	int step = 50;
// 	for( int x = -this->width/2 ;x < this->width/2; x += step)
// 	{
// 		std::vector< cv::Point > line;
// 		for( int z = -this->length/2 ; z < this->length/2; z += step)
// 		{
// 			QVec qi = innerModel->project("rgbd", innerModel->transform("rgbd", QVec::vec3(x,0,z), "vtable_top") , "rgbd");
// 			line.push_back( cv::Point(qi.x(), qi.y()));
// 		}
// 		lines.push_back(line);
// 	}
// 	for( int z = -this->length/2 ; z < this->length/2; z += step)		
// 	{
// 		std::vector< cv::Point > line;
// 		for( int x = -this->width/2 ;x < this->width/2; x += step)
// 		{
// 			QVec qi = innerModel->project("rgbd", innerModel->transform("rgbd", QVec::vec3(x,0,z), "vtable_top") , "rgbd");
// 			line.push_back( cv::Point(qi.x(), qi.y()));
// 		}
// 		lines.push_back(line);
// 	}


// 	switch( state )
// 	{
// 		case State::INIT:
// 			//qDebug() << "State::INIT";
// 			state = State::GET_IMAGE;
// 			break;
// 			
// 		case State::GET_IMAGE:
// 			qDebug() << "State::INIT";
// 			tie(frame, gray, depth, pointSeq) = handler->getImage();
// 			// executive.do("GET_IMAGE");
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
// 			points = handler->filterTable( pointSeq , points);
// 			//state = State::DRAW_HARRIS;
// 			state = State::RENDER_TABLE;
// 			break;
// 			
// 		case  State::DRAW_HARRIS:
// 			qDebug() << "State::Draw";
// 			handler->harrisdetector.drawOnImage( frame, points);
// 			imshow("Harris", frame);
// 			state = State::RENDER_TABLE;
// 			break;
// 			
// 		case State::RENDER_TABLE:	
// 			qDebug() << "State::RENDER_TABLE";
// 			this->render( frame);
// 			//imshow("Cog-Table", frame);
// 			img = new QImage(frame.data, frame.cols, frame.rows, QImage::Format_RGB888);
// 			handler->label->setPixmap(QPixmap::fromImage(*img).scaled(handler->label->width(), handler->label->height()));
// 			delete img;
// 			state = State::STOP;
// 			break;
// 		case  State::STOP:
// 			qDebug() << "State::STOP";
// 			break;
// 	}
	