/*
 *    Copyright (C) 2020 by the University of Málaga
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

typedef struct {
    double x;
    double z;
    double angle;
}Robot2DPoint;

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
	void structuralChange(const RoboCompAGMWorldModel::World &w);
	void edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modification);
	void edgeUpdated(const RoboCompAGMWorldModel::Edge &modification);
	void symbolUpdated(const RoboCompAGMWorldModel::Node &modification);
	void symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modification);
    
    //AprilBAsedLocalization interface
    void newTagBasedPose(const float x, const float z, const float alpha);
    void pickUpPoseInfo(const double& x, const double& z, const double& alpha);
    void deliverFinishedPoseInfo(const double& x, const double& z, const double& alpha);
    void trolleyPoseInfo(const double &x, const double &z, const double &alpha, const TrolleyRectangle &lines);

public slots:
	void compute(); 	

private:
    bool worldChanged;
    bool flagStop;
    bool objectRecognitionFinished;
    bool startObjectRecognition;
    bool readingFromLaser;
    bool new_data_available;
    
    enum DetectingLaserStateT
    {
        IDLE,
        DETECTING,
        FINISHED
    } detectingLaserState;
    
    Robot2DPoint robotGoalFromCamera, pickUpPoint, pickUpPointLaser, deliverFinishedPoint;
    double angle_correction;
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	InnerModel *innerModel;
	bool active;
    std::chrono::time_point<std::chrono::system_clock> startTime, startLaserDetectionTime;
    std::chrono::time_point<std::chrono::system_clock> endTime, endLaserDetectionTime;
    bool timerStarted;
    bool timerLaserDetectionStarted;
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);
    void publishModelModification();
    
    Robot2DPoint calculaPoseGlobalMIRASA3IR(const Robot2DPoint& poseRobot, const Robot2DPoint& poseLocal);
    void laserDetection(bool& model_modified);
	
};

#endif

