/*
 * Copyright (C) 2015 by
 *   MetraLabs GmbH (MLAB), GERMANY
 * and
 *   Neuroinformatics and Cognitive Robotics Labs (NICR) at TU Ilmenau, GERMANY
 * and
 *   University of Málaga
 * 
 * All rights reserved.
 *
 * Contact: info@mira-project.org
 *
 * Commercial Usage:
 *   Licensees holding valid commercial licenses may use this file in
 *   accordance with the commercial license agreement provided with the
 *   software or, alternatively, in accordance with the terms contained in
 *   a written agreement between you and MLAB or NICR.
 *
 * GNU General Public License Usage:
 *   Alternatively, this file may be used under the terms of the GNU
 *   General Public License version 3.0 as published by the Free Software
 *   Foundation and appearing in the file LICENSE.GPL3 included in the
 *   packaging of this file. Please review the following information to
 *   ensure the GNU General Public License version 3.0 requirements will be
 *   met: http://www.gnu.org/copyleft/gpl.html.
 *   Alternatively you may (at your option) use any later version of the GNU
 *   General Public License if such license has been publicly approved by
 *   MLAB and NICR (or its successors, if any).
 *
 * IN NO EVENT SHALL "MLAB" OR "NICR" BE LIABLE TO ANY PARTY FOR DIRECT,
 * INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT OF
 * THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF "MLAB" OR
 * "NICR" HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * "MLAB" AND "NICR" SPECIFICALLY DISCLAIM ANY WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND "MLAB" AND "NICR" HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS OR MODIFICATIONS.
 */

/**
 * @file MiraNavigation.C
 *    Bla
 *
 * @author Blabla
 * @date   2017/02/20
 */

#include <fw/Unit.h>
#include <Ice/Ice.h>
#include <Ice/Application.h>
#include <IceUtil/IceUtil.h>
#include "Navigator.h"
//#include "personFollower.h"
#include "MiraLaser.h"

#include <robot/Odometry.h>
#include <robot/RangeScan.h>
#include <robot/BatteryState.h>
#include <navigation/Task.h>
#include <navigation/tasks/OrientationTask.h>
#include <navigation/tasks/PositionTask.h>
#include <navigation/tasks/PreferredDirectionTask.h>
//#include <navigation/tasks/PersonFollowTask.h>
#include <navigation/tasks/DockingTask.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>

using namespace mira;
using namespace RoboCompNavigator;
//using namespace RoboComppersonFollower;
using namespace RoboCompMiraLaser;
using namespace std;

namespace mira { 

const int MAX_COUNT = 50000;

class MiraNavigation;

class LaserReporterI: public LaserReporter
{
public:
	LaserReporterI(MiraNavigation* instance);
	void toggleLaserReport(bool enable, const Ice::Current& = Ice::Current());
private:
	MiraNavigation* mira_navigation;
};

/*
class PersonFollowerI : public personFollower
{
public:
	PersonFollowerI(MiraNavigation* instance);
	void updatePersonPose(const Pose& p, const Ice::Current& = Ice::Current());
	void personLost(const Ice::Current& = Ice::Current());

private:
	MiraNavigation* mira_navigation;
};

*/
class LocalNavigatorI : public LocalNavigator
{
public:
	LocalNavigatorI(MiraNavigation* instance);
	void stop(const Ice::Current& = Ice::Current());
	bool isActive(const Ice::Current& = Ice::Current());
	bool goTo(float x, float z, float angle, const Ice::Current& = Ice::Current());
	bool goBackWardsTo(float x, float z, float angle, const Ice::Current& = Ice::Current());
	bool goToDockStation(const Ice::Current& = Ice::Current());
	bool rotate(float angle, const Ice::Current& = Ice::Current());
	string dockingStatus(const Ice::Current& = Ice::Current());
	bool goToBasePoint(const Ice::Current& = Ice::Current());
	void setOdometry(float x, float z, float angle, const Ice::Current& = Ice::Current());
	void setPathVel(const Trajectory& path, float advVel, float rotVel, const Ice::Current& = Ice::Current());
	bool setOrientation(float angle, const Ice::Current& = Ice::Current());
	void getGoal(float& goalx, float& goalz, float& subgoalx, float& subgoalz, const Ice::Current& = Ice::Current());
	void forkLiftUp(const Ice::Current& = Ice::Current());
	void forkLiftDown(const Ice::Current& = Ice::Current()); 
private:
	MiraNavigation* mira_navigation;
	
};

///////////////////////////////////////////////////////////////////////////////

class MiraNavigation : public Unit
{
MIRA_OBJECT(MiraNavigation)

public:

	MiraNavigation();

	template<typename Reflector>
	void reflect(Reflector& r)
	{
		Unit::reflect(r);

	//	r.property("PersonFollowMinDistance", mPersonFollowMinDistance, "", 1.0f);
	//	r.property("PersonFollowMaxDistance", mPersonFollowMaxDistance, "", 1.5f);
	//	r.property("PersonFollowOrientationTolerance", mPersonFollowOrientationTolerance, "", deg2rad<float>(5.0f));

		r.property("ResetBumperInterval", mResetBumperInterval, "", Duration::invalid());

	//	r.method("toggleFollow", &MiraNavigation::setFollowPerson, this, "");
	}

protected:
	virtual void initialize();
	virtual void process(const Timer& timer);
	
	void onOdometry(ChannelRead<robot::Odometry2> read);
	void onPilotEvent(ChannelRead<std::string> read);
	void onBumper(ChannelRead<bool> read);
	void onRangeScan(ChannelRead<robot::RangeScan> read);
	void onBatteryState(ChannelRead<robot::BatteryState> data);
	void onDockingStatus(ChannelRead<std::string> status);
	void onForkLiftStatus(ChannelRead<std::string> status);
	void onLimitSwitch(ChannelRead<bool> status);
	void onAPTDistance(ChannelRead<float> distance);
	
//	void setFollowPerson(bool follow);
	
public: 

	// Implementation of LocalNavigator

	void stop();
	bool isActive();
	bool goTo(float x, float z, float angle);
	bool goToDockStation();
	bool goBackWardsTo(float x, float z, float angle);
	bool rotate(float angle);

	bool goToBasePoint();
	string dockingStatus();
	void unDocking();
	void setOdometry(float x, float z, float angle);
	void setPathVel(const Trajectory& path, float advVel, float rotVel);
	bool setOrientation(float angle);
	void getGoal(float& goalx, float& goalz, float& subgoalx, float& subgoalz);

	void forkLiftUp();
	void forkLiftDown(); 

	// Implementation of personFollower
	
//	void updatePersonPose(const Pose& p);
//	void personLost();


	void toggleLaserReport(bool enable);

private:
	bool sendNavigationGoal(const Pose2& goal, float tolT, float tolR, float directionWeight, navigation::PreferredDirectionTask::Direction iDirection = navigation::PreferredDirectionTask::FORWARD);
	void sendNavigationStatus(const navigationState& state);	

	std::string mNavigationService; // Unit that implements the MIRA INavigation interface
	std::string mLocalizationService; // Unit that implements the MIRA ILocalization interface
	std::string mDockingService; // Unit that implements the MIRA IDockingProcess interface
	Channel<std::string> mPilotEventChannel; // Access to channel that contains the status of the navigation in MIRA

	Ice::ObjectAdapterPtr adapter;
	Ice::CommunicatorPtr ic;
	LocalNavigatorReportStatePrx local_navigator_report_state_prx;
	LaserPrx laser_prx;

	boost::optional<Pose2> mCurrentGoal;
	Time mCurrentGoalTime;

//	PersonFollowerI* personFollowerI;
	LocalNavigatorI* localNavigatorI;
	LaserReporterI* laserReporterI;

//	float mPersonFollowMinDistance;
//	float mPersonFollowMaxDistance;
//	float mPersonFollowOrientationTolerance;

	Duration mResetBumperInterval; // invalid --> do not reset motorstop automatically
	Time mLastBumperResetTime;

	std::atomic<bool> powerSupplyPresent;
	std::atomic<bool> charging;
	
	std::string dockingStatusVar;
	std::mutex mutexDocking;

protected:
	bool mReportLaserEnabled;
};

///////////////////////////////////////////////////////////////////////////////

MiraNavigation::MiraNavigation() : Unit(Duration::milliseconds(100))
{
	mCurrentGoal.reset();
	mCurrentGoalTime = Time::invalid();

//	mPersonFollowMinDistance = 1.0f;
//	mPersonFollowMaxDistance = 1.5f;
//	mPersonFollowOrientationTolerance = deg2rad<float>(5.0f);

	mResetBumperInterval = Duration::invalid();
	mLastBumperResetTime = Time::unixEpoch();

	mReportLaserEnabled = false;

	powerSupplyPresent = false;
	charging = false;
}

void MiraNavigation::initialize()
{
	// Subscribe to pilot event channel and get callbacks
	mPilotEventChannel = subscribe<std::string>("/navigation/PilotEvent", &MiraNavigation::onPilotEvent, this);
	subscribe<robot::Odometry2>("/robot/Odometry", &MiraNavigation::onOdometry, this);
	subscribe<bool>("/robot/Bumper", &MiraNavigation::onBumper, this);
	subscribe<robot::RangeScan>("/robot/frontLaser/Laser", &MiraNavigation::onRangeScan, this);
	subscribe<robot::BatteryState>("/robot/charger/Battery", &MiraNavigation::onBatteryState, this);
	subscribe<std::string>("/docking/DockingStatus", &MiraNavigation::onDockingStatus, this);
	subscribe<std::string>("/robot/forklift/State", &MiraNavigation::onForkLiftStatus, this);
	subscribe<bool>("/payload/LimitSwitch", &MiraNavigation::onLimitSwitch, this);
	subscribe<float>("/payload/Distance", &MiraNavigation::onAPTDistance, this);


	// Wait for a MIRA unit publishing the INavigation interface, which we will use later
	mLocalizationService = waitForServiceInterface("ILocalization");
	mNavigationService = waitForServiceInterface("INavigation");
	mDockingService = waitForServiceInterface("IDockingProcess");

	

	// Instantiate ICE
	int nargs = 0;
	char *args[0];
	ic = Ice::initialize(nargs, args);

	try {

		// We create the MiraLaser proxy
		laser_prx = LaserPrx::uncheckedCast(ic->stringToProxy("laser:tcp -h 192.168.0.154 -p 11004"));

		// We create LocalNavigator proxy 
		local_navigator_report_state_prx = LocalNavigatorReportStatePrx::uncheckedCast(ic->stringToProxy("localnavigatorreportstate:tcp -h 192.168.0.154 -p 11001"));
		
		//Initializing localNavigatorI
		localNavigatorI = new LocalNavigatorI(this);

		// Create the ObjectAdapter for LocalNavigationProxy		
		adapter = ic->createObjectAdapterWithEndpoints("LocalNavigatorProxy", "tcp -h 192.168.0.171 -p 11000");
		adapter->add(localNavigatorI, Ice::stringToIdentity("localnavigator"));

		// Lets activate LocalNavigatiorI!
		adapter->activate();

		// Initializing personFollowerI
//		personFollowerI = new PersonFollowerI(this);

		// Create the ObjectAdapter for personFollower
//		adapter = ic->createObjectAdapterWithEndpoints("personFollowerProxy", "tcp -h localhost -p 11003");
//		adapter->add(personFollowerI, Ice::stringToIdentity("personfollower"));

		// Lets activate personFollowerI!
//		adapter->activate();

		// Initializing LaserReporterI
	/*	laserReporterI = new LaserReporterI(this);

		// Create the ObjectAdapter for laserReporter
		adapter = ic->createObjectAdapterWithEndpoints("laserReporterProxy", "tcp -h 192.168.0.154 -p 11007");
		adapter->add(laserReporterI, Ice::stringToIdentity("laserreporter"));

		// Lets activate laserReporter!
		adapter->activate();
*/


		cout << "miraNavigatorComp initizalized!" << endl;

	} 
	catch (const Ice::Exception& e) 
	{
		cerr << e << endl;
        	
	} 
	catch (const char* msg) 
	{
		cerr << msg << endl;
        	
    	}

	publishService(*this);
}

void MiraNavigation::process(const Timer& timer)
{
	// TODO: this method is called periodically with the specified cycle time, so you can perform your computation here.
}

void MiraNavigation::onOdometry(ChannelRead<robot::Odometry2> read)
{
	// Report the robot pose
	const Pose2 robotPoseCorrected = getTransform<Pose2>("/robot/RobotFrame", "/maps/MapFrame", read->timestamp);
	try
	{
		local_navigator_report_state_prx->reportRobotPose(robotPoseCorrected.x(), robotPoseCorrected.y(), robotPoseCorrected.phi());
 	}
	catch(Ice::Exception& e)
	{
		//cout << "ERROR: reportRobotPose... " << e.what() << endl;
	}

	// If navigation is active and robot is driving, report the distance to the goal
	if(isActive())
	{
		sendNavigationStatus(navigationState::DRIVING);
	}	
}

void MiraNavigation::onLimitSwitch(ChannelRead<bool> status)
{
	static bool value = 0;
	if(value != status->value())
	{
		std::cout << "onLimitSwitch: "  << status->value() << std::endl;
		value = status->value();
		try
		{
			local_navigator_report_state_prx->reportLimitSwitchState(value);
		}
		catch(const Exception& ex) 
		{
			//std::cout << "Could not perform reportLimitSwitchState, exception: " << ex.what() << std::endl;
		}
	}
}

void MiraNavigation::onAPTDistance(ChannelRead<float> distance)
{
	static float prev_distance = 0.0f;
	float distance_ = distance->value();
	
	if(fabs(prev_distance - distance_) >=0.002f && distance_ <= 9.0f)
	//if(distance_ <= 9.0f)
	{
		prev_distance = distance_;
		std::cout << "onAPTDistance: "  << distance_ << std::endl;
		try
		{
			local_navigator_report_state_prx->reportAPTSensor(distance_);
		}
		catch(const Exception& ex) 
		{
			//std::cout << "Could not perform reportLimitSwitchState, exception: " << ex.what() << std::endl;
		}
	}

}

void MiraNavigation::onPilotEvent(ChannelRead<std::string> read)
{
	const std::string pilotStatus = read->value();
	
	navigationState state = navigationState::IDLE;
	if(pilotStatus == "GoalReached")
		state = navigationState::REACHED;
	else if(pilotStatus == "PlanAndDrive" || pilotStatus == "PathTemporarilyLost")
		state = navigationState::DRIVING;
	else if(pilotStatus == "Idle")
		state = navigationState::IDLE;
	else 
		state = navigationState::FAILED;

	sendNavigationStatus(state);
}

void MiraNavigation::onBumper(ChannelRead<bool> read)
{
	const bool bumperTriggered = read->value();
	const Time ts = read->timestamp;
	if(!bumperTriggered || !mResetBumperInterval.isValid() || 
		(ts - mLastBumperResetTime) < mResetBumperInterval)
		return;

	mLastBumperResetTime = ts;
	auto ftr = callService<void>("/robot/Robot", "resetMotorStop");
	ftr.wait();
	ftr.get();
}

void MiraNavigation::onRangeScan(ChannelRead<robot::RangeScan> read)
{
	if(!mReportLaserEnabled)
		return;

	float startAngle, deltaAngle;
	const Pose3 laserPose = getTransform<Pose3>(read->frameID, "/robot/RobotFrame", read->timestamp);
	boost::tie(startAngle, deltaAngle) = read->getAnglesForOrientation(laserPose);

	LaserDataT laser;
	const robot::RangeScan& scan = read->value();
	for(std::size_t i = 0; i < scan.range.size(); ++i)
	{
		if(scan.valid[i] != robot::RangeScan::Valid)
			continue;
		
		PointT p;
		p.angle = startAngle + i * deltaAngle;
		p.range = scan.range[i];
		p.x = p.range * cos(p.angle);
		p.y = p.range * sin(p.angle);
		laser.push_back(p);
	}

	try
	{
		laser_prx->reportLaserData(laser);
	}
	catch(const Exception& ex) 
	{
		std::cout << "Could not perform reportLaserData, exception: " << ex.what() << std::endl;
	}
}


// https://www.mira-project.org/MIRA-doc/toolboxes/RobotDataTypes/BatteryState_8h_source.html

void MiraNavigation::onBatteryState(ChannelRead<robot::BatteryState> data)
{
	static int cont = 0;
	
	//Voy a quitar carga de CPU
	if (cont >= MAX_COUNT)
	{
		cont = 0;
	}
	else
	{
		RobotBatteryLevel battery_level;
		battery_level.charging = data->charging;
		battery_level.current = data->current;
		battery_level.cvoltage = data->cellVoltage;
		battery_level.lifePercent = data->lifePercent;
		battery_level.lifeTime = data->lifeTime;
		battery_level.powerSupplyPresent = data->powerSupplyPresent;
		battery_level.voltage = data->voltage;

		powerSupplyPresent =  data->powerSupplyPresent;
		charging = data->charging;

		try
		{
			local_navigator_report_state_prx->reportRobotBatteryLevel(battery_level);		
		}
		catch(Ice::Exception& ex)
		{
			//std::cout << "Could not perform reportBatteryLevel, exception: " << ex.what() << std::endl;
		}
		cont++;
	}
}

void MiraNavigation::onForkLiftStatus(ChannelRead<std::string> status)
{
	std::string forkLiftStatusVar = status->value();
	std::cout << "forkLiftStatus: " << forkLiftStatusVar << std::endl;
	try
	{
		local_navigator_report_state_prx->reportForkLiftState(forkLiftStatusVar);		
	}
	catch(Ice::Exception& ex)
	{
		//std::cout << "Could not perform reportBatteryLevel, exception: " << ex.what() << std::endl;
	}
}

void MiraNavigation::onDockingStatus(ChannelRead<std::string> status)
{

	
	static int cont = 0;	
	std::cout << status->value() << std::endl;		
	mutexDocking.lock();
	dockingStatusVar = status->value(); 
	mutexDocking.unlock();

	/*if(dockingStatusVar == "Failure")
	{
		unDocking();
		cont = 0;
	}
	else if (dockingStatusVar == "Docked")
	{
		if(!cont)
		{
			std::this_thread::sleep_for(std::chrono::seconds(5));		
		}

		if(!powerSupplyPresent)
		{
			std::cout << "PowerSupply not present! Trying docking again..." << std::endl;
			if(!cont)
			{
				goToDockStation();
				++cont;
			}
			else
			{		
				unDocking();		
			}
		}
		else if(powerSupplyPresent && !charging)
		{
			if(!charging)
			{
				std::cout << "Clara is not charging! Trying docking again..." << std::endl;
				if(!cont)
				{
					goToDockStation();
					++cont;
				}			
				else
				{	
					unDocking();		
				}
			}			
			else
			{
				std::cout << "Charging after waiting!" << std::endl;
			}
		}
		else
		{
			std::cout << "Charging!" << std::endl;	
		}
	}
	else if(dockingStatusVar == "UnDocked")
	{
		goToDockStation();
		cont = 0;
	}
	else
	{
		cont = 0;
	}*/
	
}

void MiraNavigation::unDocking()
{
	try
	{
		// Let the robot dock off
		unsigned int station = 0;
		auto rpcFuture = callService<void>(mDockingService, "dockOff", station);

		// Get the result, even though it is void. If an exception has occurred, this will throw as well.
		rpcFuture.get();

//		std::this_thread::sleep_for(std::chrono::seconds(5));		
	}
	catch(const Exception& ex) {
		std::cout << "Could not perform goToDockStation, exception: " << ex.what() << std::endl;
	}

//

}
/*
void MiraNavigation::setFollowPerson(bool follow)
{
	if(!follow)
	{
		std::cout << "Stop following person" << std::endl;	

		stop();
		return;
	}

	std::cout << "Start following person" << std::endl;	
	try 
	{
		boost::shared_ptr<navigation::Task> task(new navigation::Task());
		task->addSubTask(navigation::SubTaskPtr(new navigation::PersonFollowTask(mPersonFollowMinDistance, mPersonFollowMaxDistance, mPersonFollowOrientationTolerance, 0.0)));
		task->addSubTask(navigation::SubTaskPtr(new navigation::PreferredDirectionTask(navigation::PreferredDirectionTask::FORWARD, 0.9)));

		auto rpcAnswerFuture = callService<void>(mNavigationService, "setTask", task);
		rpcAnswerFuture.wait();
		rpcAnswerFuture.get();
	} 
	catch(const Exception& ex) 
	{
		std::cout << "Could not perform person following, exception: " << ex.what() << std::endl;
	}	
}
*/
///////////////////////////////////////////////////////////////////////////////

void MiraNavigation::stop()
{
	auto rpcAnswerFuture = callService<void>(mNavigationService, "setTask", boost::shared_ptr<navigation::Task>());
	rpcAnswerFuture.wait();
	rpcAnswerFuture.get();
}

bool MiraNavigation::isActive()
{
	auto channelRead = mPilotEventChannel.read();

	const Time ts = channelRead->timestamp;
	const std::string status = channelRead->value();

	return (status == "PlanAndDrive");
}

bool MiraNavigation::goTo(float x, float z, float angle)
{
	const float translationTolerance = 0.2f;
//	const float rotationTolerance = deg2rad<float>(22.5f);
	const float rotationTolerance = deg2rad<float>(5.0f);
	const Pose2 goal(x, z, angle);

	//deactivate the ATP sensor
	/*auto rpcAnswerFuture = callService<void>("/payload/APTPayloadSensor", "enable", "false");
	rpcAnswerFuture.wait();
	rpcAnswerFuture.get();
*/

	return sendNavigationGoal(goal, translationTolerance, rotationTolerance, 1.0);
}

bool MiraNavigation::goBackWardsTo(float x, float z, float angle)
{
	const float translationTolerance = 0.2f;
	const float rotationTolerance = deg2rad<float>(5.0f);
	const Pose2 goal(x, z, angle);

	//activate the ATP sensor
	/*auto rpcAnswerFuture = callService<void>("/payload/APTPayloadSensor", "enable", "true");
	rpcAnswerFuture.wait();
	rpcAnswerFuture.get();
*/
	return sendNavigationGoal(goal, translationTolerance, rotationTolerance, 1.0, navigation::PreferredDirectionTask::BACKWARD);
}

bool MiraNavigation::rotate(float angle)
{
	cout << "Executing rotate..." << endl;
	const float translationTolerance = 0.2f;
	const float rotationTolerance = deg2rad<float>(5.0f);
	const Pose2 goal(mCurrentGoal->x(), mCurrentGoal->y(), angle);//be carefull if mCurrentGoal has not been initialized!

	return sendNavigationGoal(goal, translationTolerance, rotationTolerance, 1.0);
}


bool MiraNavigation::goToDockStation()
{
	try
	{
		// Let the robot dock on
		unsigned int station = 0;
		auto rpcFuture = callService<void>(mDockingService, "dockOn", station);

		// Get the result, even though it is void. If an exception has occurred, this will throw as well.
		rpcFuture.get();

//		std::this_thread::sleep_for(std::chrono::seconds(5));		
	}
	catch(const Exception& ex) 
	{
		std::cout << "Could not perform goToDockStation, exception: " << ex.what() << std::endl;
		return false;
	}
	return true;
}

bool MiraNavigation::goToBasePoint()
{
	try
	{
		// Let the robot dock on
		unsigned int station = 0;
		auto rpcFuture = callService<void>(mDockingService, "dockOff", station);

		// Get the result, even though it is void. If an exception has occurred, this will throw as well.
		rpcFuture.get();		
	}
	catch(const Exception& ex) 
	{
		std::cout << "Could not perform goToBasePoint, exception: " << ex.what() << std::endl;
		return false;
	}
	return true;
}

string MiraNavigation::dockingStatus()
{
	mutexDocking.lock();
	string status = dockingStatusVar;
	mutexDocking.unlock();
	return status;
}

void MiraNavigation::setOdometry(float x, float z, float angle)
{
	const float tolT = 0.1;
	const float tolR = 0.1;	
	const PoseCov2 pose(x, z, angle, tolT, tolT, tolR);

	auto ftr = callService<void>(mLocalizationService, "setInitPose", pose);
	ftr.wait();
	ftr.get();
}

void MiraNavigation::setPathVel(const Trajectory& path, float advVel, float rotVel)
{
	MIRA_THROW(XNotImplemented, "setPathVel is currently not implemented");
}

bool MiraNavigation::setOrientation(float angle)
{
	MIRA_THROW(XNotImplemented, "setOrientation is currently not implemented");
	return false;
}


void MiraNavigation::getGoal(float& goalx, float& goalz, float& subgoalx, float& subgoalz)
{
	MIRA_THROW(XNotImplemented, "getGoal is currently not implemented");
}

void MiraNavigation::forkLiftUp()
{
	auto rpcAnswerFuture = callService<void>("/robot/Robot", "forkliftMoveUp");
	rpcAnswerFuture.wait();
	rpcAnswerFuture.get();
}

void MiraNavigation::forkLiftDown()
{
	auto rpcAnswerFuture = callService<void>("/robot/Robot", "forkliftMoveDown");
	rpcAnswerFuture.wait();
	rpcAnswerFuture.get();
}

// Implementation of personFollower
/*
void MiraNavigation::updatePersonPose(const Pose& p)
{
	const Pose2 relativePoseRaw(p.x, p.y, p.orientation);
	publishTransform<Pose2>("/robot/PersonFrame", relativePoseRaw, Time::now());
}

void MiraNavigation::personLost()
{
	stop(); // if we are currently driving, we stop driving
}
*/
void MiraNavigation::sendNavigationStatus(const navigationState& state)
{
	// Calculate distance for translation and rotation based on the relative transformation
	// between the current pose and the goal pose.
	const Pose2 robotPoseCorrected = getTransform<Pose2>("/robot/RobotFrame", "/maps/MapFrame", Time::now());
	const Pose2 relativeTransformToGoal = !mCurrentGoal ? Pose2() : (mCurrentGoal->inverse() * robotPoseCorrected);
	const float distanceT = !mCurrentGoal ? -1 : relativeTransformToGoal.t.norm(); // translation distance in meters
	const float distanceR = !mCurrentGoal ? -1 : relativeTransformToGoal.phi(); // rotation distance in radians
	const int timeElapsed = !mCurrentGoal ? -1 : (Time::now() - mCurrentGoalTime).totalMilliseconds();

	// Call reportRobotState
	try
	{
		//navigationState { IDLE, DRIVING, REACHED, FAILED };
		local_navigator_report_state_prx->reportRobotState(distanceT, distanceR, timeElapsed, state);		
	}		
	catch(Ice::Exception& e)
	{
		//cout << "ERROR: reportRobotState... " << e.what() << endl;
	}
}



bool MiraNavigation::sendNavigationGoal(const Pose2& goal, float tolT, float tolR, float directionWeight, navigation::PreferredDirectionTask::Direction iDirection)
{
	cout << "Executing goTo..." << endl;
	try {
		boost::shared_ptr<navigation::Task> task(new navigation::Task());
		task->addSubTask(navigation::SubTaskPtr(new navigation::PositionTask(goal.t, tolT)));
		task->addSubTask(navigation::SubTaskPtr(new navigation::OrientationTask(goal.phi(), tolR)));
		task->addSubTask(navigation::SubTaskPtr(new navigation::PreferredDirectionTask(iDirection, directionWeight)));

		auto rpcAnswerFuture = callService<void>(mNavigationService, "setTask", task);
		rpcAnswerFuture.wait();
		rpcAnswerFuture.get();
	} catch(const Exception& ex) {
		std::cout << "Could not perform goTo, exception: " << ex.what() << std::endl;
		return false;
	}

	// Save current goal for checking the robot state later
	mCurrentGoal.reset(goal);
	mCurrentGoalTime = Time::now();

	return true;
}

void MiraNavigation::toggleLaserReport(bool enable)
{
	mReportLaserEnabled = enable;
}

// personFollowerI interface implementation
/*

PersonFollowerI::PersonFollowerI(MiraNavigation* instance)
{
	mira_navigation = instance;
}
void PersonFollowerI::updatePersonPose(const Pose& p, const Ice::Current&)
{
	mira_navigation->updatePersonPose(p);
}

void PersonFollowerI::personLost(const Ice::Current&)
{
	mira_navigation->personLost();
}
*/

// LocalNavigatorI interface implementation


LocalNavigatorI::LocalNavigatorI(MiraNavigation* instance)
{
	mira_navigation = instance;
}

void LocalNavigatorI::stop(const Ice::Current&)
{
	mira_navigation->stop();
}

bool LocalNavigatorI::isActive(const Ice::Current&)
{
	return mira_navigation->isActive();
}

bool LocalNavigatorI::goTo(float x, float z, float angle, const Ice::Current&)
{
	return mira_navigation->goTo(x, z, angle);
}

bool LocalNavigatorI::goBackWardsTo(float x, float z, float angle, const Ice::Current&)
{
	return mira_navigation->goBackWardsTo(x, z, angle);
}

bool LocalNavigatorI::goToDockStation(const Ice::Current&)
{
	return mira_navigation->goToDockStation();
}

string LocalNavigatorI::dockingStatus(const Ice::Current&)
{
	return mira_navigation->dockingStatus();
}

bool LocalNavigatorI::goToBasePoint(const Ice::Current&)
{
	return mira_navigation->goToBasePoint();
}

void LocalNavigatorI::setOdometry(float x, float z, float angle, const Ice::Current&)
{
	mira_navigation->setOdometry(x, z, angle);
}

void LocalNavigatorI::setPathVel(const Trajectory& path, float advVel, float rotVel, const Ice::Current&)
{
	mira_navigation->setPathVel(path, advVel, rotVel);
}

bool LocalNavigatorI::setOrientation(float angle, const Ice::Current&)
{
	return mira_navigation->setOrientation(angle);
}

void LocalNavigatorI::getGoal(float& goalx, float& goalz, float& subgoalx, float& subgoalz, const Ice::Current&)
{
	mira_navigation->getGoal(goalx, goalz, subgoalx, subgoalz);
}

void LocalNavigatorI::forkLiftUp(const Ice::Current&)
{
	mira_navigation->forkLiftUp();
}

void LocalNavigatorI::forkLiftDown(const Ice::Current&)
{
	mira_navigation->forkLiftDown();
}

bool LocalNavigatorI::rotate(float angle, const Ice::Current&)
{
	return mira_navigation->rotate(angle);
}

// LaserReporterI implementation


LaserReporterI::LaserReporterI(MiraNavigation* instance)
{
	mira_navigation = instance;
}

void LaserReporterI::toggleLaserReport(bool enable, const Ice::Current&)
{
	mira_navigation->toggleLaserReport(enable);
}

} //End MIRA namespace

MIRA_CLASS_SERIALIZATION(mira::MiraNavigation, mira::Unit );
