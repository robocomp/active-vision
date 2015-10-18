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