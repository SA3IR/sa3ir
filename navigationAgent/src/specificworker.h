/*
 *    Copyright (C) 2017 by the University of Málaga
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


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <thread>
#include <chrono>
#include <limits> 

const int PATH_NOT_FOUND_TIMEOUT = 10;
const string GOAL_POSITIONS_FILE = "/home/robocomp/robocomp/components/sa3ir/etc/goals/goalPositions.txt";

struct Pose2D
{ 
	float x;
	float z;
	float angle;
};



class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	bool reloadConfigAgent();
	bool activateAgent(const ParameterMap &prs);
	bool setAgentParameters(const ParameterMap &prs);
	ParameterMap getAgentParameters();
	void killAgent();
	int uptimeAgent();
	bool deactivateAgent();
	StateStruct getAgentState();
	void reportRobotState(const float distanceToGoal, const float angToGoal, const int timeElapsed, const navigationState &state);
	void reportRobotPose(const float x, const float z, const float angle);
	void structuralChange(const RoboCompAGMWorldModel::World &w);
	void edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modification);
	void edgeUpdated(const RoboCompAGMWorldModel::Edge &modification);
	void symbolUpdated(const RoboCompAGMWorldModel::Node &modification);
	void symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modification);
	void reportRobotBatteryLevel(const RobotBatteryLevel &batteryLevel);
    void reportForkLiftState(const string &status);
    void reportAPTSensor(const float distance);
    void reportLimitSwitchState(const bool &state);
	void modifyingNavigationEdge(bool& model_modified);

private:
	void sendGoalToMira(const float goalx, const float goalz,const float angle, bool moving_fordwards = true) const;
	void localizeRobot() const;
	void readGoalPosition(string label);
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);
	void moveRobotToDockStation() const;
	void unDocking() const;
	string dockingStatus() const;
	bool isValidLabel(const string& label) const;
	bool publishModelModification();
    void checkAbortAction();
    bool existsLink(int source, int target, const std::string& name);
    void setGoal();
    void setNearestGoalPosition(const std::string& link);
    float distanceToGoal(const std::string& label);
    void readGoalPosition();
    void setObjectDetectionPose2();
    void forkUp();
    void pickUpTrolley(bool& model_modified);

public slots:
	void compute(); 

private: //AGM attributes and methods
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	InnerModel *innerModel;
	bool active;
    enum ForkingStateT
    {
        IDLE,
        FORKINGUP,
        FORWKINDOWN,
        FINISHED
    } forkingState;
	
private: 
    QWaitCondition waitGotoResult;
    bool gotoFinished;
	bool worldChanged;
	Pose2D currentRobotPose;
	navigationState navigationCurrentState;
	bool batteryLevelChanged;
	bool goToDockStation;
#ifdef MIRON_TOOLS
	string batteryLevel;
#else
    int batteryPercentLevel;
#endif
	Pose2D goalPosition;
    std::string goal;
	bool sendGoal;
    bool flagStop;
	bool doingUnDocking;
	bool robotCharging;
    std::thread abortActionThread;
    std::map<std::string, Pose2D> goalPositions;
    bool movingFordwards;
	
		
};

#endif

