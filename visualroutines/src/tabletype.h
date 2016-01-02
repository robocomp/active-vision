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
#include "opencv2/imgproc/imgproc.hpp"
#include <innermodel/innermodel.h>
#include <RGBD.h>
#include "legtype.h"
#include "tabletoptype.h"

using namespace cv;

class SpecificWorker;
typedef std::vector<QVec> QPoints;

class TableType
{
	public:
		TableType(QString _name, InnerModel *_innerModel);
		TableType(const TableType& other);
		void update(SpecificWorker *handler);
		~TableType();
		enum class State {INIT, GET_IMAGE, HARRIS, STOP, FILTER_TABLE_HEIGHT, CLUSTER, DRAW_HARRIS, RENDER_TABLE, SELECT_FIRST_CORNER};
		State state = State::INIT;
	
		
		/**
		 * @brief Renders the object on the camera plane using InnerModel
		 * 
		 * @param frame OPenCV image frame where to draw the rendered lines
		 */
		void render(cv::Mat& frame);
		
	private:
		// geometric parameters of the object
		float height, length, width, topThickness, legWidth;
		
		InnerModel *innerModel;
		QString name;
		
		//List of legs objects
		QList< LegType*> legs;
		
		//tabletop object
		TabletopType *tabletop;
		
		std::tuple<int, int> selectFirstCorner(const QPoints& cluster3D);
		
// 		QVec getCornerImage(uint cornerNumber);
// 		cv::Point getCornerImage(uint cornerNumber);
// 		QVec getCornerWorld(uint cornerNumber);
// 		void moveCorner(uint corner, const QVec point);
		
};

#endif // TABLETYPE_H
