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

#ifndef TABLETOPTYPE_H
#define TABLETOPTYPE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <innermodel/innermodel.h>
#include <iostream>

// OpenMesh
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

class TabletopType
{
	public:
		TabletopType(const QString &_name, const QString &_parent, InnerModel *_innerModel, const QVec &offset, float _length=900, float _width=1400, float _thick=80);
		TabletopType(const TabletopType& other);
		~TabletopType();
		std::vector<QVec> getCorners();
		
		// type of mesh structure form OPenMesh
		typedef OpenMesh::PolyMesh_ArrayKernelT<>  MyMesh;
		
		/**
		 * @brief Renders the object on the camera using InneModel
		 * 
		 * @param lines array of arrays of points
		 * @return void
		 */
		void render(std::vector< std::vector< cv::Point > >& lines);
	
	private:
		QString name;
		InnerModel *innerModel;
		
		//Geomtric parameters
		QVec offset;
		float width, length, thick;
		
		//Parts
		
		
		/**
		 * @brief Makes the tabletop bigger in the Z dimension
		 * 
		 * @param k final lenght in mm
		 * @return void
		 */
		void makeItLonger(float k);
		/**
		 * @brief Makes the tabletop bigger in the X dimension
		 * 
		 * @param w final width in mm
		 * @return void
		 */
		void makeItWider(float w);
		
		/**
		 * @brief  Makes the tabletop bigger in the Y dimension
		 * 
		 * @param t final thikness in mm
		 * @return void
		 */
		void makeItThicker(float t);
	
		//mesh variables form OPenMesh
		MyMesh mesh;
		MyMesh::VertexHandle vhandle[8];
};

#endif // TABLETOPTYPE_H
