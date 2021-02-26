/*
 *    Copyright (C)2020 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef LOCALNAVIGATOR_H
#define LOCALNAVIGATOR_H

// Ice includes
#include <Ice/Ice.h>
#include <Navigator.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompNavigator;

class LocalNavigatorI : public virtual RoboCompNavigator::LocalNavigator
{
public:
LocalNavigatorI(GenericWorker *_worker);
	~LocalNavigatorI();

	bool setOrientation(const float  angle, const Ice::Current&);
	void setOdometry(const float  x, const float  z, const float  angle, const Ice::Current&);
	string dockingStatus(const Ice::Current&);
	bool goTo(const float  x, const float  z, const float  angle, const Ice::Current&);
	bool goToDockStation(const Ice::Current&);
	void stop(const Ice::Current&);
	void setPathVel(const Trajectory  &path, const float  advVel, const float  rotVel, const Ice::Current&);
	bool goToBasePoint(const Ice::Current&);
	bool isActive(const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
