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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
    gotoStarted =  false;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::compute()
{
	//QMutexLocker locker(mutex);
	//computeCODE
 	try
 	{
        mutex->lock();
        if(gotoStarted) {
            std::cout << "Goto received" << std::endl;
            std::cout << "esperando 6s" << std::endl;
            sleep(6);
            gotoStarted = false;
            mutex->unlock();
            float distanceToGoal = 0;
            float angToGoal = 0;
            int timeElapsed = 0;
            navigationState state = navigationState::REACHED;
            localnavigatorreportstate_proxy->reportRobotState(distanceToGoal, angToGoal, timeElapsed, state);
        }
        else
            mutex->unlock();
 	}
 	catch(const Ice::Exception &e)
 	{
 		std::cout << "Error reading from Camera" << e << std::endl;
 	}
}

bool SpecificWorker::goTo(const float x, const float z, const float angle)
{
    QMutexLocker locker(mutex);
    gotoStarted = true;
    return true;

}


bool SpecificWorker::setOrientation(const float angle)
{
//implementCODE

}

void SpecificWorker::setOdometry(const float x, const float z, const float angle)
{
//implementCODE

}

string SpecificWorker::dockingStatus()
{
//implementCODE
    return "docking";
}



bool SpecificWorker::goToDockStation()
{
//implementCODE

}

void SpecificWorker::stop()
{
//implementCODE

}

void SpecificWorker::setPathVel(const Trajectory &path, const float advVel, const float rotVel)
{
//implementCODE

}

bool SpecificWorker::goToBasePoint()
{
//implementCODE

}

bool SpecificWorker::isActive()
{
//implementCODE

}


bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);


	return true;
}
