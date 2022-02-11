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

/**
       \brief
       @author ARomero
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/xml_parsing.h>
#include <list>

#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "args.hpp"
#include "skill_action.hpp"
#include "variant_action.hpp"
#include "deliver_object.hpp"
#include "pick_object.hpp"
#include "detect_object.hpp"
#include "get_position.hpp"
#include "move_roboter_position.hpp"
#include "move_roboter_position2.hpp"
#include "approachObject.hpp"
#include "dock.hpp"
#include "pickingRequests.hpp"
#include "validEstimatedPosition.hpp"
#include "doNothing.hpp"
#include "moveToStart.hpp"
#include "rotateRight.hpp"
#include "rotateLeft.hpp"
#include "moveToDeliverArea.hpp"
#include "moveToDefaultDeliverArea.hpp"

typedef enum ActionResult{
    SUCCESS, FAILURE, RUNNING        
} ActionRes;

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

	void startNavigationAction(const std::string& action, bool docking = false);
    void startApproachAction();
    void startRecognitionAction();
    void startDeliverAction();
    void startPickUpAction();
    
    ActionRes checkNavigationResult();
    ActionRes checkRecognitionResult();
    ActionRes checkDeliverResult();
    ActionRes checkPickUpResult();
    
    bool startPickDeliverStart();
    bool robotAtGoodPositionFromMark();
    
public slots:
	void compute(); 	

protected:
     struct NavigationInfo{
        string x, z, angle, docking;
    } navigationInfo;
    
private:
	std::mutex m;
	std::condition_variable cond_var;
	bool notified;
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	InnerModel *innerModel;
	bool active;  
    ActionRes action_result;
   // const std::string BT_FILE = "./bin/use_case_extended_p2.xml";
    const std::string BT_FILE = "./bin/use_case_sacir_rot.xml";
    
private:   
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel); 
	void initBehaviorTree();
    bool publishModelModification();
    void startAction(const std::string& name, int source, int target,const std::string& tagRem, const std::string& tagAdd);
    void startNavigation(const std::string& name, int source, int target, const std::string& tagRem, const std::string& tagAdd, const std::string& dock);
    ActionRes checkResult(const std::string& name, int source, int target, const std::string& tagRem, const std::string& tagAdd);
    bool existsLink(int source, int target, const std::string& name);
    
	
};

#endif

