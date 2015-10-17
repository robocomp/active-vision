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

TableType::TableType(): height(700), length(900), width(1200), topThickness(30)
{

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
	
	//elegimos un origen en el mundo: Y alienada con el mundo, X y Z con el robot
	innerModel->newTransform("vtable", "static", innerModel->getNode("table"), 0, 0, 0, 0, 0, 0);
	//innerModel->updateTransformValues("vtable", 0, 0, 0, 0, 0, 0, "table");

	//recorremos el tablero muesteando puntos cada 50mm
	std::vector< std::vector < cv::Point> > lines;
	for( int x = -this->width/2 ;x < this->width/2; x += 50)
	{
		std::vector< cv::Point > line;
		for( int z = this->length/2 ; z < this->length*3/2; z += 50)
		{
			QVec qi = innerModel->project("rgbd", innerModel->transform("rgbd", QVec::vec3(x,this->height,z), "vtable") , "rgbd");
			line.push_back( cv::Point(qi.x(), qi.y()));
		}
		lines.push_back(line);
	}
	
	cv::polylines(frame, lines, false, cv::Scalar(0,0,200));
	  
	return frame;
}
