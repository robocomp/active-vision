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

LegType::LegType( const QString &_name, const QVec &_offset, float _length, float _width) : name(_name), offset(_offset),  width(_width), length(_length)
{
}

LegType::LegType(const LegType& other)
{
}

LegType::~LegType()
{
}

void LegType::render(cv::Mat& frame, InnerModel* innerModel, const QString& parent, std::vector< std::vector< cv::Point > >& lines)
{
	int step = 50;
	innerModel->newTransform(name, "static", innerModel->getNode(parent), offset.x(),  offset.y(), offset.z(), 0, 0, 0);
	for( int y = 0 ;y > -length; y -= step)
	{
		std::vector< cv::Point > line;
		for( int x = -width/2 ; x < width/2; x += step)
		{
			QVec qi = innerModel->project(name, QVec::vec3(x,y,-width/2), "rgbd");
			line.push_back( cv::Point(qi.x(), qi.y()));			
			QVec qii = innerModel->project(name, QVec::vec3(x,y,width/2), "rgbd");
			line.push_back( cv::Point(qii.x(), qii.y()));			
		}
		for( int z = -width/2 ; z < width/2; z += step)
		{
			QVec qi = innerModel->project(name, QVec::vec3(width/2,y,z), "rgbd");
			line.push_back( cv::Point(qi.x(), qi.y()));
			QVec qii = innerModel->project(name, QVec::vec3(-width/2,y,z), "rgbd");
			line.push_back( cv::Point(qii.x(), qii.y()));
		}
		lines.push_back(line);
	}
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
}
