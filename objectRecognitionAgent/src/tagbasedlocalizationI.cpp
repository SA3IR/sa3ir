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
#include "tagbasedlocalizationI.h"

TagBasedLocalizationI::TagBasedLocalizationI(GenericWorker *_worker)
{
	worker = _worker;
}


TagBasedLocalizationI::~TagBasedLocalizationI()
{
}

void TagBasedLocalizationI::deliverFinishedPoseInfo(const double  x, const double  z, const double  alpha, const Ice::Current&)
{
	worker->deliverFinishedPoseInfo(x, z, alpha);
}

void TagBasedLocalizationI::newTagBasedPose(const float  x, const float  z, const float  alpha, const Ice::Current&)
{
	worker->newTagBasedPose(x, z, alpha);
}

void TagBasedLocalizationI::pickUpPoseInfo(const double  x, const double  z, const double  alpha, const Ice::Current&)
{
	worker->pickUpPoseInfo(x, z, alpha);
}

void TagBasedLocalizationI::trolleyPoseInfo(const double  x, const double  z, const double  alpha, const TrolleyRectangle  &lines, const Ice::Current&)
{
	worker->trolleyPoseInfo(x, z, alpha, lines);
}

