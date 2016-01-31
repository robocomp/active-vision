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

#ifndef DEQUE_H
#define DEQUE_H

#include <QtCore>

template <class T> class Deque
{
	public:
		Deque(int mSize=100, int initVal=0)
			{ maxSize = mSize; l.resize(maxSize); l.fill(initVal);};
		~Deque(){};
		void enqueue( const T &d) {l.append(d); if(l.size()>maxSize) l.pop_front(); };
		QVector<T> getVector(){ return l;};
	private:
		int maxSize;
		QVector<T> l;
	
};

#endif // DEQUE_H
