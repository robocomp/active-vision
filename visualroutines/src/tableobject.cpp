/*
 * Copyright 2016 pbustos <email>
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

#include "tableobject.h"

TableObject::TableObject() 
{
	tableName = "vtable";
}

TableObject::~TableObject()
{
}

void TableObject::setPose(const QVec& pose)
{
	currentPose = pose;
	innerModel->updateTransformValues("vtable_t", pose.x(), pose.y(), pose.z(), pose.rx(), pose.ry(), pose.rz());
}

RoboCompRGBD::PointSeq TableObject::filterTablePoints(const RoboCompRGBD::PointSeq &points, const Mat &depth, bool addNoise)
{
	RoboCompRGBD::PointSeq lp;
	int lowThreshold=50;
	int ratio = 3;
	int kernel_size = 5;
	
	// filter depthimage
	Mat depthF;
  Canny( depth, depthF, lowThreshold, lowThreshold*ratio, kernel_size );
	Size size = depth.size();
	
	//qDebug() << __FUNCTION__ << "points" << points.size() << size.width << size.height;
	
	for (int i=0; i< size.height; i+=4) 
	{
		for (int j=0; j< size.width; j+=4) 
		{
			//qDebug() << __FUNCTION__ << depth.at<uchar>(i,j) << depthF.at<uchar>(i,j);
			if( depthF.at<uchar>(i,j) > 0 )
			{
				RoboCompRGBD::PointXYZ p = points[j+i*size.width];
				QVec pw = innerModel->transform("world", QVec::vec3(p.x,p.y,p.z),"rgbd"); 			
				//GET convez hull from innermodel!!!
				if( pw.y()>100 and pw.z()<1000 )
				{
					if( addNoise) 
						pw = pw + QVec::uniformVector(3, -30, 30);
					
					RoboCompRGBD::PointXYZ pn = { pw.x(),pw.y(),pw.z(),0 };
					lp.push_back(pn);
				}
			}
		}
	}
	//qDebug() << __FUNCTION__ << lp.size();
	
  //imshow( "canny", depth );
	return lp;
}

