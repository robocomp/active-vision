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

RoboCompRGBD::PointSeq TableObject::renderPose(const QVec& newPose, const RoboCompRGBD::PointSeq &points)
{
	RoboCompRGBD::PointSeq pSeq;
	
	//move the table to pose
	//innerModel->updateTranslationValues(tableName, pose.x(), pose.y(), pose.z());
	
	QVec delta = newPose - currentPose;
	RoboCompRGBD::PointXYZ pw;
	for (auto p: points)
	{
		pw = {p.x + delta.x(), p.y+ delta.y(), p.z + delta.z(), 0};
		pSeq.push_back( pw );
	}
	currentPose = newPose;
	//update InnerModelViewer
	
	//get RGBD frame
	
	//return pSeq
	
	return pSeq;
}
