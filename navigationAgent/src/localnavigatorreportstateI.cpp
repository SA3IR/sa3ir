/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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
#include "localnavigatorreportstateI.h"

LocalNavigatorReportStateI::LocalNavigatorReportStateI(GenericWorker *_worker)
{
	worker = _worker;
}


LocalNavigatorReportStateI::~LocalNavigatorReportStateI()
{
}

void LocalNavigatorReportStateI::reportAPTSensor(const float  distance, const Ice::Current&)
{
	worker->reportAPTSensor(distance);
}

void LocalNavigatorReportStateI::reportRobotBatteryLevel(const RobotBatteryLevel  &batteryLevel, const Ice::Current&)
{
	worker->reportRobotBatteryLevel(batteryLevel);
}

void LocalNavigatorReportStateI::reportRobotPose(const float  x, const float  z, const float  angle, const Ice::Current&)
{
	worker->reportRobotPose(x, z, angle);
}

void LocalNavigatorReportStateI::reportRobotState(const float  distanceToGoal, const float  angToGoal, const int  timeElapsed,  navigationState  state, const Ice::Current&)
{
	worker->reportRobotState(distanceToGoal, angToGoal, timeElapsed, state);
}

void LocalNavigatorReportStateI::reportLimitSwitchState( bool  state, const Ice::Current&)
{
	worker->reportLimitSwitchState(state);
}

void LocalNavigatorReportStateI::reportForkLiftState(const string  &status, const Ice::Current&)
{
	worker->reportForkLiftState(status);
}

