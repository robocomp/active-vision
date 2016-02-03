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

#include "glviewer.h"
#include <math.h>

GLViewer::GLViewer(QWidget* parent) : QGLViewer(parent)
{
  restoreStateFromFile();
}

void GLViewer::setCloud(const RoboCompRGBD::PointSeq& points, QVec color)
{
	ps = points;
	col = color;
	
}

void GLViewer::setSensedCloud(const RoboCompRGBD::PointSeq& points, QVec color)
{
	sensed = points;
	sensedCol = color;
	
}

void GLViewer::initializeGL()
{
	
}

void GLViewer::draw( )
{
	//qDebug() << __FUNCTION__;
 	glBegin(GL_POINTS);
	for (auto p: sensed)
  {
  	 glColor3f( sensedCol.x(), sensedCol.y(), sensedCol.z());
		 glVertex3f(p.x/1000.f, p.z/1000.f, p.y/1000.f);
  }
 	for (auto p: ps)
  	{
  		 glColor3f( col.x(), col.y(), col.z());
			 glVertex3f(p.x/1000.f, p.z/1000.f, p.y/1000.f);
  	}
 	glEnd();
}
