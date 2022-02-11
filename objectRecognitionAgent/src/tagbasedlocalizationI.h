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
#ifndef TAGBASEDLOCALIZATION_H
#define TAGBASEDLOCALIZATION_H

// Ice includes
#include <Ice/Ice.h>
#include <TagBasedLocalization.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompTagBasedLocalization;

class TagBasedLocalizationI : public virtual RoboCompTagBasedLocalization::TagBasedLocalization
{
public:
TagBasedLocalizationI(GenericWorker *_worker);
	~TagBasedLocalizationI();

	void deliverFinishedPoseInfo(const double  x, const double  z, const double  alpha, const Ice::Current&);
	void newTagBasedPose(const float  x, const float  z, const float  alpha, const Ice::Current&);
	void pickUpPoseInfo(const double  x, const double  z, const double  alpha, const Ice::Current&);
	void trolleyPoseInfo(const double  x, const double  z, const double  alpha, const TrolleyRectangle  &lines, const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
