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

#ifndef TABLEOBJECT_H
#define TABLEOBJECT_H
#include <innermodel/innermodel.h>
#include <RGBD.h>

class TableObject
{
	public:
		TableObject();
		~TableObject();
		void setInnerModel(InnerModel *inner) { innerModel = inner; currentPose = innerModel->transform("world","vtable_t");};
		RoboCompRGBD::PointSeq renderPose( const QVec& newPose, const RoboCompRGBD::PointSeq& points);
		
	private:
		InnerModel *innerModel;
		QString tableName;
		QVec currentPose;
		
};

#endif // TABLEOBJECT_H
