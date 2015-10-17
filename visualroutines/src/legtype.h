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

class LegType
{
public:
	LegType(const QString &_name, const QVec &offset, float _length=800, float _width=70);
	LegType(const LegType& other);
	~LegType();
	
	QString name;
	QVec offset;
	float width, length;
	void render(cv::Mat& frame, InnerModel* innerModel, const QString &parent, std::vector< std::vector < cv::Point> > &lines);
};

#endif // LEGTYPE_H
