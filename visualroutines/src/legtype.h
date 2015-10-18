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

#ifndef LEGTYPE_H
#define LEGTYPE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <innermodel/innermodel.h>
#include <iostream>

// OpenMesh
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

class LegType
{
	public:
		LegType(const QString &_name, const QString &parent, InnerModel *_innerModel, const QVec &offset, float _length=750, float _width=70);
		LegType(const LegType& other);
		~LegType();
		typedef OpenMesh::PolyMesh_ArrayKernelT<>  MyMesh;
		
		/**
		 * @brief Renders the object on the camera using InneModel
		 * 
		 * @param lines array of arrays of points
		 * @return void
		 */
		void render(std::vector< std::vector < cv::Point> > &lines);
		
	private:
			
		QString name;
		InnerModel *innerModel;
		
		//Geometric parameters
		QVec offset;
		float width, length;
		
			/**
		 * @brief Makes the tabletop bigger in the Z dimension
		 * 
		 * @param k final lenght in mm
		 * @return void
		 */
		void makeItLonger(float k);
		
			/**
		 * @brief Makes the tabletop bigger in the Z dimension
		 * 
		 * @param w final width in mm
		 * @return void
		 */
		void makeItWider(float w);
		
	  //OpenMesh structures	
		MyMesh mesh;
		MyMesh::VertexHandle vhandle[8];
};

#endif // LEGTYPE_H
