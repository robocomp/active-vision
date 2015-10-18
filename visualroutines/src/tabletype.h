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

#ifndef TABLETYPE_H
#define TABLETYPE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <innermodel/innermodel.h>
#include "legtype.h"
#include "tabletoptype.h"

class TableType
{
	public:
		TableType();
		TableType(const TableType& other);
		~TableType();
		cv::Mat render(cv::Mat& frame, InnerModel* innerModel);
		
		float height, length, width, topThickness, legWidth;
	
	private:
		
		QList< LegType*> legs;
		TabletopType *tabletop;
		
};

#endif // TABLETYPE_H
