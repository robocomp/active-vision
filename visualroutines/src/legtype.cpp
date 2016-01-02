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

#include "legtype.h"

LegType::LegType( const QString &_name, const QString &parent, InnerModel *_innerModel,const QVec &_offset, float _length, float _width) : 
									name(_name), innerModel(_innerModel), offset(_offset),  width(_width), length(_length)
{
	//create the virtual transform. Probably should go in constructor
	innerModel->newTransform(name, "static", innerModel->getNode(parent), offset.x(), offset.y(), offset.z(), 0, 0, 0);
	
	//Create the vertices of a unitary cube
	vhandle[0] = mesh.add_vertex(MyMesh::Point(-1, -1, 1));  
  vhandle[1] = mesh.add_vertex(MyMesh::Point( 1, -1, 1));
  vhandle[2] = mesh.add_vertex(MyMesh::Point( 1,  1, 1));
  vhandle[3] = mesh.add_vertex(MyMesh::Point(-1,  1, 1));
  vhandle[4] = mesh.add_vertex(MyMesh::Point(-1, -1, -1));
  vhandle[5] = mesh.add_vertex(MyMesh::Point( 1, -1, -1));
  vhandle[6] = mesh.add_vertex(MyMesh::Point( 1,  1, -1));
  vhandle[7] = mesh.add_vertex(MyMesh::Point(-1,  1, -1));
	
	std::vector<MyMesh::VertexHandle>  face_vhandles;

	// Generate faces
  face_vhandles.clear();
  face_vhandles.push_back(vhandle[0]);
  face_vhandles.push_back(vhandle[1]);
  face_vhandles.push_back(vhandle[2]);
  face_vhandles.push_back(vhandle[3]);
  mesh.add_face(face_vhandles);
 
  face_vhandles.clear();
  face_vhandles.push_back(vhandle[7]);
  face_vhandles.push_back(vhandle[6]);
  face_vhandles.push_back(vhandle[5]);
  face_vhandles.push_back(vhandle[4]);
  mesh.add_face(face_vhandles);

  face_vhandles.clear();
  face_vhandles.push_back(vhandle[1]);
  face_vhandles.push_back(vhandle[0]);
  face_vhandles.push_back(vhandle[4]);
  face_vhandles.push_back(vhandle[5]);
  mesh.add_face(face_vhandles);

  face_vhandles.clear();
  face_vhandles.push_back(vhandle[2]);
  face_vhandles.push_back(vhandle[1]);
  face_vhandles.push_back(vhandle[5]);
  face_vhandles.push_back(vhandle[6]);
  mesh.add_face(face_vhandles);

  face_vhandles.clear();
  face_vhandles.push_back(vhandle[3]);
  face_vhandles.push_back(vhandle[2]);
  face_vhandles.push_back(vhandle[6]);
  face_vhandles.push_back(vhandle[7]);
  mesh.add_face(face_vhandles);

  face_vhandles.clear();
  face_vhandles.push_back(vhandle[0]);
  face_vhandles.push_back(vhandle[3]);
  face_vhandles.push_back(vhandle[7]);
  face_vhandles.push_back(vhandle[4]);
  mesh.add_face(face_vhandles);

	//Adjust to object's real dimensions
	makeItLonger(length);
	makeItWider(width);
}

LegType::LegType(const LegType& other)
{
}

LegType::~LegType()
{
}

void LegType::makeItLonger(float k)
{
	//to make it longer downwards multiply vertices with y=-1  (down side) by y = y *frame k
	mesh.set_point( vhandle[0], mesh.point(vhandle[0])*MyMesh::Point(1,k,1));
	mesh.set_point( vhandle[1], mesh.point(vhandle[1])*MyMesh::Point(1,k,1));
	mesh.set_point( vhandle[4], mesh.point(vhandle[4])*MyMesh::Point(1,k,1));
	mesh.set_point( vhandle[5], mesh.point(vhandle[5])*MyMesh::Point(1,k,1));
}

void LegType::makeItWider(float w)
{
	//to make it wider multiply all vertices at its x and z coordinate by half the final size
	for(auto vh : vhandle)
		mesh.set_point( vh, mesh.point(vh)*MyMesh::Point(w/2,1,w/2));
}

void LegType::render(std::vector< std::vector< cv::Point > >& lines)
{
	//Recorre las caras y para cada cara recorre los vértices, los proyecta y los copia a lines
  for( MyMesh::FaceIter f_it=mesh.faces_begin(); f_it!=mesh.faces_end(); ++f_it) 
  {
			std::vector< cv::Point > line;
			for(MyMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it); fv_it.is_valid() ; ++fv_it) 
			{
				MyMesh::Point &pp = mesh.point(*fv_it);
				QVec qi = innerModel->project(name, QVec::vec3( pp[0],pp[1],pp[2]), "rgbd");
				line.push_back( cv::Point(qi.x(), qi.y()));
			}
			lines.push_back(line);
	}
}



//Vertical lines
// 	for( int x = -width/2 ; x < width/2; x += step)
// 	{
// 		std::vector< cv::Point > line;
// 		for( int y = 0 ;y > -length; y -= step)
// 		{
// 			QVec qi = innerModel->project(name, QVec::vec3(x,y,0), "rgbd");
// 			line.push_back( cv::Point(qi.x(), qi.y()));
// 			QVec qii = innerModel->project(name, QVec::vec3(width/2,x,y) , "rgbd");
// 			line.push_back( cv::Point(qi.x(), qi.y()));
// 		}
// 		lines.push_back(line);
// 	}