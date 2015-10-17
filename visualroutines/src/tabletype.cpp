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

TableType::TableType(): height(700), length(900), width(1400), topThickness(80), legWidth(90)
{
	legs.push_back( new LegType("leg0", QVec::vec3(-width/2+legWidth/2, 0, -length/2+legWidth/2 )));
	legs.push_back( new LegType("leg1", QVec::vec3(+width/2-legWidth/2, 0, -length/2+legWidth/2 )));
}

TableType::TableType(const TableType& other)
{
}

TableType::~TableType()
{
}

cv::Mat TableType::render(cv::Mat& frame, InnerModel *innerModel)
{

	//Este es el ejemplo básico. Necesitaríamos 
	    // la matriz de cámara. La sacamos el InnerModel (DONE)
		// los puntos de la cog-mesa 3D. Hay que muestrear el objeto. Podemos empezar por el tablero
	    // Proyectarlos y guardarlos en el array (puede ser un array de arrays de puntos (líneas)
	
	//Creamos unos transforms virtuales en el mundo qe representaran nuestra cog-mesa, De momento cogemos t_table del .xml como origen
	innerModel->newTransform("vtable", "static", innerModel->getNode("t_table"), 0, 0, 0, 0, 0, 0);
	innerModel->newTransform("vtable_top", "static", innerModel->getNode("vtable"), 0, this->height, 0, 0, 0, 0);
	innerModel->newTransform("vtable_down", "static", innerModel->getNode("vtable_top"), 0, -this->topThickness, 0, 0, 0, 0);
	
	//recorremos el tablero muesteando puntos cada 25mm (step)
	std::vector< std::vector < cv::Point> > lines;
	int step = 50;
	for( int x = -this->width/2 ;x < this->width/2; x += step)
	{
		std::vector< cv::Point > line;
		for( int z = -this->length/2 ; z < this->length/2; z += step)
		{
			QVec qi = innerModel->project("rgbd", innerModel->transform("rgbd", QVec::vec3(x,0,z), "vtable_top") , "rgbd");
			line.push_back( cv::Point(qi.x(), qi.y()));
		}
		lines.push_back(line);
	}
	for( int z = -this->length/2 ; z < this->length/2; z += step)		
	{
		std::vector< cv::Point > line;
		for( int x = -this->width/2 ;x < this->width/2; x += step)
		{
			QVec qi = innerModel->project("rgbd", innerModel->transform("rgbd", QVec::vec3(x,0,z), "vtable_top") , "rgbd");
			line.push_back( cv::Point(qi.x(), qi.y()));
		}
		lines.push_back(line);
	}
	
	for( auto l : legs)
		l->render(frame, innerModel, "vtable_down", lines);
	
	//pintamos todas la líneas sobre la imagen
	cv::polylines(frame, lines, false, cv::Scalar(0,0,200));
	
	//cuando iteremos no se debe borrar
	innerModel->removeNode("vtable");
	
	return frame;
}
