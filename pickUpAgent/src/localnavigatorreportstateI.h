/*
 *    Copyright (C)2021 by YOUR NAME HERE
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
#ifndef LOCALNAVIGATORREPORTSTATE_H
#define LOCALNAVIGATORREPORTSTATE_H

// Ice includes
#include <Ice/Ice.h>
#include <Navigator.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompNavigator;

class LocalNavigatorReportStateI : public virtual RoboCompNavigator::LocalNavigatorReportState
{
public:
LocalNavigatorReportStateI(GenericWorker *_worker);
	~LocalNavigatorReportStateI();

	void reportRobotState(const float  distanceToGoal, const float  angToGoal, const int  timeElapsed,  navigationState  state, const Ice::Current&);
	void reportRobotBatteryLevel(const RobotBatteryLevel  &batteryLevel, const Ice::Current&);
	void reportForkLiftState(const string  &status, const Ice::Current&);
	void reportRobotPose(const float  x, const float  z, const float  angle, const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
