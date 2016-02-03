/*
 * Copyright 2016 <copyright holder> <email>
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

#ifndef GLVIEWER_H
#define GLVIEWER_H

#include <QtCore>
#include <QGLViewer/qglviewer.h>
#include <RGBD.h>
#include <qmat/QMatAll>

class GLViewer : public QGLViewer
{
	public:
		GLViewer(QWidget *parent);
		void setCloud(const RoboCompRGBD::PointSeq& points, QVec color);
		void setSensedCloud(const RoboCompRGBD::PointSeq& points, QVec color);
		void draw();
		void initializeGL();

		
	protected:
		RoboCompRGBD::PointSeq ps, sensed;
		QVec col, sensedCol;

	};

#endif // GLVIEWER_H
