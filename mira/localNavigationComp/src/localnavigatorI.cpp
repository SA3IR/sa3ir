/*
 *    Copyright (C) 2020 by YOUR NAME HERE
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
#include "localnavigatorI.h"

LocalNavigatorI::LocalNavigatorI(GenericWorker *_worker)
{
	worker = _worker;
}


LocalNavigatorI::~LocalNavigatorI()
{
}

bool LocalNavigatorI::setOrientation(const float  angle, const Ice::Current&)
{
	return worker->setOrientation(angle);
}

void LocalNavigatorI::setOdometry(const float  x, const float  z, const float  angle, const Ice::Current&)
{
	worker->setOdometry(x, z, angle);
}

string LocalNavigatorI::dockingStatus(const Ice::Current&)
{
	return worker->dockingStatus();
}

bool LocalNavigatorI::goTo(const float  x, const float  z, const float  angle, const Ice::Current&)
{
    std::cout << "goto" << std::endl;
	return worker->goTo(x, z, angle);
}

bool LocalNavigatorI::goToDockStation(const Ice::Current&)
{
	return worker->goToDockStation();
}

void LocalNavigatorI::stop(const Ice::Current&)
{
	worker->stop();
}

void LocalNavigatorI::setPathVel(const Trajectory  &path, const float  advVel, const float  rotVel, const Ice::Current&)
{
	worker->setPathVel(path, advVel, rotVel);
}

bool LocalNavigatorI::goToBasePoint(const Ice::Current&)
{
	return worker->goToBasePoint();
}

bool LocalNavigatorI::isActive(const Ice::Current&)
{
	return worker->isActive();
}

