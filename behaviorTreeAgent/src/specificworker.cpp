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
#include "specificworker.h"

using namespace BT;

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	timer.setSingleShot(true); //compute method is executed only once
	notified = false;
	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	
	try
	{
		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
		structuralChange(w);
	}
	catch(...)
	{
		printf("The executive is probably not running, waiting for first AGM model publication...");
	}

	return true;
}

void SpecificWorker::compute()
{
    std::this_thread::sleep_for (std::chrono::seconds(10));
    std::cout << "Initializing behavior tree..." << std::endl;
    initBehaviorTree();
}


void SpecificWorker::startNavigationAction(const std::string& action, bool docking)
{
    std::string docking_str;
    if(docking)
        docking_str = "1";
    else
        docking_str = "0";
    startNavigation(action, 3, 5, "stopped", "start", docking_str);
}

void SpecificWorker::startApproachAction()
{
    startAction("approach", 3, 5, "stopped", "start");
}

void SpecificWorker::startRecognitionAction()
{
    startAction("objectRecognition", 3, 6, "not", "start");
}

void SpecificWorker::startDeliverAction()
{
    startAction("deliver", 3, 8, "not", "start");
}

void SpecificWorker::startPickUpAction()
{
    startAction("pickUp", 3, 7, "not", "start");
}

bool SpecificWorker::startPickDeliverStart()
{
    bool result_ = false;
    bool has_trolley_exists_ = false;
    bool requested_trolley_exists_ = false;
    bool delivered_trolley_exists_ = false;
    for (unsigned int i = 0; i < 4; i++)
    {
        if(existsLink (3, 13+i, "has_trolley"))
            has_trolley_exists_ = true;
        else if(existsLink (3, 13+i, "request_trolley"))
            requested_trolley_exists_ = true;
        else if(existsLink (3, 13+i, "deliver_trolley"))
            delivered_trolley_exists_ = true;
    }
    
#ifdef SINGLE_TROLLEY     
    if(has_trolley_exists_)
        result_ = false;
    else if(requested_trolley_exists_ || delivered_trolley_exists_)
        result_ = true;
#else
    if(requested_trolley_exists_ || delivered_trolley_exists_)
        result_ = true;
#endif    
    return result_;
    
}

bool SpecificWorker::robotAtGoodPositionFromMark()
{
    bool result = false;
    try
    {
        //Reading robot position
        float x_robot_ = str2float(worldModel->getSymbol(5)->getAttribute("x"));
        float z_robot_ = str2float(worldModel->getSymbol(5)->getAttribute("z"));;
        float angle_robot_ = str2float(worldModel->getSymbol(5)->getAttribute("angle"));
        //Reading navigation goal (from apriltags and ObjectRecognitionAgent)
        float x_estimate_ = str2float(worldModel->getSymbol(9)->getAttribute("x"));
        float z_estimate_ = str2float(worldModel->getSymbol(9)->getAttribute("z"));
        float angle_estimate_ = str2float(worldModel->getSymbol(9)->getAttribute("angle"));
        
        float distance_error_ = sqrt(pow(x_estimate_-x_robot_,2) + pow(z_estimate_-z_robot_,2));
        float angle_error_ = abs(angle_estimate_-angle_robot_);
        
        std::cout << x_robot_ << " " << z_robot_ << " "<< angle_robot_ << ", " << 
        x_estimate_ << " " << z_estimate_ << " " << angle_estimate_ << " : " << distance_error_ << " " << angle_error_ << endl;
        
        result = ((distance_error_ < 0.10) || (angle_error_ < 0.10));
        
    } 
    catch(...){}
    return result;
}

bool SpecificWorker::existsLink(int source, int target, const std::string& name)
{
    bool result = false;
    try
    {
        worldModel->getEdgeByIdentifiers(source, target, name);
        result = true;
    }
    catch(...){}
    return result;
}

void SpecificWorker::startNavigation(const std::string& name, int source, int target, const std::string& tagRem, const std::string& tagAdd, const std::string& docking)
{
    std::unique_lock<std::mutex> lock(m);
    std::cout << name <<  " " << docking << std::endl;
    bool model_modified_ = false;
    try
    {
        worldModel->removeEdgeByIdentifiers(source, target, tagRem); //ya estoy buscando el objeto
        worldModel->addEdgeByIdentifiers(source, target, tagAdd); //ya estoy buscando el objeto
        worldModel->getSymbol(5)->setAttribute("docking", docking);
        worldModel->getSymbol(3)->setAttribute("lastAction", name);
        model_modified_ = true;
    }
    catch(...)
    {
        std::cout << name << ": ERROR" << std::endl;    
    }
    while(!model_modified_ || !publishModelModification())
    {
        model_modified_ = false;
        cond_var.wait(lock);
        try{
            worldModel->removeEdgeByIdentifiers(source, target, tagRem); //ya estoy buscando el objeto
            worldModel->addEdgeByIdentifiers(source, target, tagAdd); //ya estoy buscando el objeto
            worldModel->getSymbol(5)->setAttribute("docking", docking);
            worldModel->getSymbol(3)->setAttribute("lastAction", name);
            model_modified_ = true;
        }
        catch(...){}
        //std::cout << name << " finished" << std::endl;
    } 
   
}

void SpecificWorker::startAction(const std::string& name, int source, int target, const std::string& tagRem, const std::string& tagAdd)
{
    std::unique_lock<std::mutex> lock(m);
    std::cout << name << std::endl;
    //bool published = false;
    bool expected_label = true;
    bool model_published = false;
    
    while(!model_published)
    {
        try {
            worldModel->removeEdgeByIdentifiers(source, target, tagRem);
        }
        catch(...) {
            try {
                worldModel->removeEdgeByIdentifiers(source, target, "failed"); 
            } catch(...) {
                expected_label = false;
            }
        }
            
        if (expected_label) {
            try
            {
                worldModel->addEdgeByIdentifiers(source, target, tagAdd); 
                worldModel->getSymbol(3)->setAttribute("lastAction", name);
                while(!publishModelModification())
                {
                    cond_var.wait(lock);
                    try {
                        worldModel->removeEdgeByIdentifiers(source, target, tagRem);
                    }
                    catch(...) {
                        try {
                            worldModel->removeEdgeByIdentifiers(source, target, "failed"); 
                        } catch(...) {
                            expected_label = false;
                        }
                    }
                    worldModel->addEdgeByIdentifiers(source, target, tagAdd); 
                    worldModel->getSymbol(3)->setAttribute("lastAction", name);
                }
                model_published = true;
                std::cout << name << " finished" << std::endl;
                
            } 
            catch(...)
            {
                std::cout << name << ": this should not happen!" << std::endl;    
                cond_var.wait(lock);
            }
        }
        else {
                std::cout << name << ": ERROR" << std::endl;    
                cond_var.wait(lock);
        }
    }
}


ActionRes SpecificWorker::checkNavigationResult()
{
   return checkResult("waitForNavigationAction", 3, 5, "finish", "stopped"); 
}

ActionRes SpecificWorker::checkRecognitionResult()
{
    return checkResult("waitForRecognitionAction", 3, 6, "finish", "not"); 
}

ActionRes SpecificWorker::checkDeliverResult()
{
    return checkResult("waitForDeliverAction", 3, 8, "finish", "not"); 
}

ActionRes SpecificWorker::checkPickUpResult()
{
    return checkResult("waitForPickUpAction", 3, 7, "finish", "not"); 
}


ActionRes SpecificWorker::checkResult(const std::string& name, int source, int target, const std::string& tagRem, const std::string& tagAdd)
{
    std::unique_lock<std::mutex> lock(m);
    
    ActionRes result = ActionResult::RUNNING;
    try
    {
        worldModel->removeEdgeByIdentifiers(source, target, tagRem); 
        worldModel->addEdgeByIdentifiers(source, target, tagAdd); 
        if(publishModelModification())
            result = ActionResult::SUCCESS;
    } 
    catch(...) {
        try
		{
			worldModel->getEdgeByIdentifiers(source,target,"failed");
            result = ActionResult::FAILURE;
			
		}
		catch(...)
		{}
    }
    
    return result;
}

/*
void SpecificWorker::waitForAction(const std::string& name, int source, int target, const std::string& tagRem, const std::string& tagAdd)
{
    std::cout << name << std::endl;
    std::unique_lock<std::mutex> lock(m);
    try
    {
        worldModel->removeEdgeByIdentifiers(source, target, tagRem);
        worldModel->addEdgeByIdentifiers(source, target, tagAdd); 
        publishModelModification();
    } 
    catch(...)
    {
        bool modelChanged = false;
        while (!notified || !modelChanged)
        {  
            cond_var.wait(lock);
            try
            {
                worldModel->removeEdgeByIdentifiers(source, target, tagRem); //ya estoy buscando el objeto
                worldModel->addEdgeByIdentifiers(source, target, tagAdd); //ya estoy buscando el objeto
                modelChanged = publishModelModification();
                std::cout << name << " finished" << std::endl;
            }
            catch(...)
            {
                std::cout << name << ": ERROR" << std::endl;        
            }
        }  
        notified = false;
    } 
}
*/
bool SpecificWorker::publishModelModification()
{
	bool finished = false;
    bool published = false;
	do
	{
		try
		{
			AGMMisc::publishModification(worldModel, agmexecutive_proxy, "behaviorTreeAgent");
            finished = true;
            published = true;
			
		}
		catch(RoboCompAGMExecutive::Locked& e)
		{
			qDebug() << e.what() << ". Lo vuelvo a intentar en 300s";
			std::this_thread::sleep_for(std::chrono::milliseconds(300));
		}
		catch(RoboCompAGMExecutive::OldModel& e) 
		{
			qDebug() << e.what();
			finished = true; //si el modelo es antiguo damos por finalizado el método. Llegará un nuevo modelo con el que poder trabajar
		}
		catch(...)
		{
			qDebug() << "Error en el método AGM::publishModification. Lo vuelvo a intentar en 500ms";
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
	}
	while(!finished);
    
    return published;
}

void SpecificWorker::initBehaviorTree()
{
    
    struct Arguments{
        std::string tree_file;
        std::string IP;
    };
    
    
    Arguments arg;
    
    arg.tree_file = BT_FILE.c_str();
    //arg.tree_file = "./bin/use_case.xml";
    
    arg.IP = "localhost";
    
    zmq::context_t zmq_context(1);

    BehaviorTreeFactory factory;

    // Register moveRoboterPosition Action
    TreeNodeManifest manif;
    manif.type = NodeType::ACTION;
    manif.registration_ID = "moveRoboterPosition";
    manif.ports = BT::PortsList{
    BT::InputPort<std::string>("angle"),
	BT::InputPort<std::string>("x"),
	BT::InputPort<std::string>("z")};
    auto move_roboter_position = [this](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new MoveRoboterPosition(this, name, config));
    };
    factory.registerBuilder(manif, move_roboter_position);
    
    manif.type = NodeType::ACTION;
    manif.registration_ID = "moveRoboterPosition2";
    auto move_roboter_position2 = [this](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new MoveRoboterPosition2(this, name, config));
    };
    factory.registerBuilder(manif, move_roboter_position2);

    manif.type = NodeType::ACTION;
    manif.registration_ID = "DeliverObject";
    auto deliver_object = [this](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new DeliverObject(this, name, config));
    };
    factory.registerBuilder(manif, deliver_object);

    manif.type = NodeType::ACTION;
    manif.registration_ID = "approachObject";
    auto approach_object = [this](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new ApproachObject(this, name, config));
    };
    factory.registerBuilder(manif, approach_object);
    
    manif.type = NodeType::ACTION;
    manif.registration_ID = "dock";
    auto dock_ = [this](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new Dock(this, name, config));
    };
    factory.registerBuilder(manif, dock_);
    
    manif.type = NodeType::ACTION;
    manif.registration_ID = "DetectObject";
    auto detect_oject = [this](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new DetectObject(this, name, config));
    };
    factory.registerBuilder(manif, detect_oject);

    manif.type = NodeType::ACTION;
    manif.registration_ID = "PickObject";
    auto pick_object = [this](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new PickObject(this, name, config));
    };
    factory.registerBuilder(manif, pick_object);

    manif.type = NodeType::ACTION;
    manif.registration_ID = "pickingRequests";
    auto pick_request_object = [this](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new PickingRequests(this, name, config));
    };
    factory.registerBuilder(manif, pick_request_object);

    manif.type = NodeType::ACTION;
    manif.registration_ID = "validEstimatedPosition";
    auto valid_estimated_position = [this](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new ValidEstimatedPosition(this, name, config));
    };
    factory.registerBuilder(manif, valid_estimated_position);
    
    manif.type = NodeType::ACTION;
    manif.registration_ID = "DoNothing";
    auto do_nothing = [this](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new DoNothing(this, name, config));
    };
    factory.registerBuilder(manif, do_nothing);
    
    manif.type = NodeType::ACTION;
    manif.registration_ID = "moveToStart";
    auto move_start = [this](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new MoveToStart(this, name, config));
    };
    factory.registerBuilder(manif, move_start);
    
    manif.type = NodeType::ACTION;
    manif.registration_ID = "moveToDeliverArea";
    auto move_deliver = [this](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new MoveToDeliverArea(this, name, config));
    };
    factory.registerBuilder(manif, move_deliver);
    
    manif.type = NodeType::ACTION;
    manif.registration_ID = "moveToDefaultDeliverArea";
    auto move_default_deliver = [this](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new MoveToDefaultDeliverArea(this, name, config));
    };
    factory.registerBuilder(manif, move_default_deliver);
    
    
    manif.type = NodeType::ACTION;
    manif.registration_ID = "VariantAction";
    manif.ports = BT::PortsList{BT::OutputPort<std::string>("value")};
    auto creator = [&zmq_context, &arg](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new VariantAction(name, config, 
            arg.IP.c_str(), zmq_context));
    };
    factory.registerBuilder(manif, creator);

    for (const auto& model : factory.manifests())
    {
        std::cout << model.first << std::endl;
    }

    auto tree = factory.createTreeFromFile(arg.tree_file);

    // add loggers
    StdCoutLogger logger_cout(tree);
    FileLogger logger_file(tree, "ulm_trace.fbl");
    PublisherZMQ publisher_zmq(tree);

    //------------------------------------------------------
    // Execute the tree
    NodeStatus status = NodeStatus::RUNNING;
    // Keep on ticking until you get either a SUCCESS or FAILURE state
    while( status == NodeStatus::RUNNING)
    {
        status = tree.tickRoot();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } 
}



bool SpecificWorker::reloadConfigAgent()
{
	return true;
}

bool SpecificWorker::activateAgent(const ParameterMap &prs)
{
	bool activated = false;
	if (setParametersAndPossibleActivation(prs, activated))
	{
		if (not activated)
		{
			return activate(p);
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool SpecificWorker::setAgentParameters(const ParameterMap &prs)
{
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

ParameterMap SpecificWorker::getAgentParameters()
{
	return params;
}

void SpecificWorker::killAgent()
{

}

int SpecificWorker::uptimeAgent()
{
	return 0;
}

bool SpecificWorker::deactivateAgent()
{
	return deactivate();
}

StateStruct SpecificWorker::getAgentState()
{
	StateStruct s;
	if (isActive())
	{
		s.state = Running;
	}
	else
	{
		s.state = Stopped;
	}
	s.info = p.action.name;
	return s;
}

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::World &w)
{
	std::unique_lock<std::mutex> lock(m);
 	AGMModelConverter::fromIceToInternal(w, worldModel);
    notified = true;
    cond_var.notify_one();
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modification)
{
	std::unique_lock<std::mutex> lock(m);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
    notified = true;
    cond_var.notify_one();
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	std::unique_lock<std::mutex> lock(m);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
    notified = true;
    cond_var.notify_one();
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	std::unique_lock<std::mutex> lock(m);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
    notified = true;
    cond_var.notify_one();
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modification)
{
	std::unique_lock<std::mutex> lock(m);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
    notified = true;
    cond_var.notify_one();
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}



bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	printf("<<< setParametersAndPossibleActivation\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
	}

	try
	{
		action = params["action"].value;
		std::transform(action.begin(), action.end(), action.begin(), ::tolower);
		//TYPE YOUR ACTION NAME
		if (action == "actionname")
		{
			active = true;
		}
		else
		{
			active = true;
		}
	}
	catch (...)
	{
		printf("exception in setParametersAndPossibleActivation %d\n", __LINE__);
		return false;
	}

	// Check if we should reactivate the component
	if (active)
	{
		active = true;
		reactivated = true;
	}

	printf("setParametersAndPossibleActivation >>>\n");

	return true;
}
void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
	try
	{
		AGMModelPrinter::printWorld(newModel);
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "behaviorTreeAgent");
	}
	catch(...)
	{
		exit(1);
	}
}




