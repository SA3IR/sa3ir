/*
 *    Copyright (C) 2017 by YOUR NAME HERE
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
	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	worldModel->version = -1;
	innerModel = new InnerModel();
	
	batteryLevelChanged = false;
	goToDockStation = false;
	worldChanged = false;
	navigationCurrentState = navigationState::IDLE;
	sendGoal = false;
    flagStop = false;
	doingUnDocking = false;
	robotCharging = false;
	
	goalPosition.x = 0; 
	goalPosition.z = 0;
	goalPosition.angle = 0;
    
    movingFordwards = true;
    
    forkingState = IDLE;
#ifndef MIRON_TOOLS
    batteryPercentLevel = 0;
#endif
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	abortActionThread.join();
}

void SpecificWorker::compute()
{
    
    static bool first = true;
    if(first)
    {
        readGoalPosition();
        cout << "sleeping 10 seconds..." << endl;
        sleep(10);
        first = false;
        cout << "starting..." << endl;
    }
    
	QMutexLocker locker(mutex);
	

    bool model_modified = false;
    
    
    try
    {
        worldModel->removeEdgeByIdentifiers(3, 5, "abort"); 
        std::cout << "Parando el navegador" << std::endl;
        worldModel->addEdgeByIdentifiers(3, 5, "stopped"); 
        model_modified = true;
        flagStop = true;
    }
    catch(...)
    {
        pickUpTrolley(model_modified);
        if(worldChanged)
        {
            
            modifyingNavigationEdge(model_modified);
            worldChanged = false;
        }
            
        if(doingUnDocking && (dockingStatus()=="UnDocked" || dockingStatus() == "Failure"))
        {
            cout << "doingUndocking a false" << endl;
            doingUnDocking = false;
            modifyingNavigationEdge(model_modified);
        }
            
        // Si ha terminado de navegar cambiamos el (1, "navigation", 19) por  (1, "finished", 19)
        if(navigationCurrentState == navigationState::REACHED)
        {
            try
            { 
                worldModel->removeEdgeByIdentifiers(3, 5, "navigating"); 
                model_modified = true;
                worldModel->addEdgeByIdentifiers(3, 5, "finish"); 
                model_modified = true;	
                worldModel->getSymbol(5)->setAttribute("result", "OK");
                model_modified = true;
                worldModel->getSymbol(9)->setAttribute("place", goal);
                model_modified = true;
            }
            catch(...)
            {
            }
            
            if(goToDockStation)
            {
                moveRobotToDockStation();
                goToDockStation = false;
            }
        } 
        else if(navigationCurrentState == navigationState::FAILED) //si ha fallado le envío de nuevo el objetivo
        {
            try
            {
                std::cout << "Objetivo no accesible!! Espero " << PATH_NOT_FOUND_TIMEOUT << " segundos..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(PATH_NOT_FOUND_TIMEOUT));

            /*   std::cout << "Enviando de nuevo el objetivo al navegador..." << std::endl;
                const float x = str2float(worldModel->getSymbol(5)->getAttribute("x"));
                const float z = str2float(worldModel->getSymbol(5)->getAttribute("z"));
                const float angle = str2float(worldModel->getSymbol(5)->getAttribute("angle"));
                sendGoalToMira(x,z,angle);*/
                worldModel->getSymbol(5)->setAttribute("result", "FAILED");
                goalPosition.x = str2float(worldModel->getSymbol(5)->getAttribute("x"));	
                goalPosition.z = str2float(worldModel->getSymbol(5)->getAttribute("z"));	
                goalPosition.angle = str2float(worldModel->getSymbol(5)->getAttribute("angle"));
                goToDockStation = (str2int(worldModel->getSymbol(5)->getAttribute("docking"))? true : false); 
                model_modified = true;
                sendGoal = true;
            }
            catch(...)
            {
            }
        }
        
        // Vamos actualizando la posición del robot en el modelo del mundo
        if (navigationCurrentState != navigationState::IDLE)
        {
            try
            {
                worldModel->getSymbol(9)->setAttribute("x", float2str(currentRobotPose.x));
                worldModel->getSymbol(9)->setAttribute("z", float2str(currentRobotPose.z));
                worldModel->getSymbol(9)->setAttribute("angle", float2str(currentRobotPose.angle));
                model_modified = true;
            }
            catch(...)
            {
            }	
        }
        
        if(batteryLevelChanged)
        {
            int32_t battery_symbol = worldModel->getIdentifierByType("battery");
            if(battery_symbol!=-1)
            {
                try
                {
#ifdef MIRON_TOOLS                    
                    worldModel->getSymbol(battery_symbol)->setAttribute("level", batteryLevel);
#else
                    worldModel->getSymbol(battery_symbol)->setAttribute("level", int2str(batteryPercentLevel));
#endif
                    model_modified = true;
                }
                catch(...)
                {
                }
            }
        }
    }
     // Miramos si hay que publicar el modelo del mundo
    if(model_modified && publishModelModification())
    {
        navigationCurrentState = navigationState::IDLE;
    }
}

void SpecificWorker::modifyingNavigationEdge(bool& model_modified)
{
    try
    {
        worldModel->getEdgeByIdentifiers(3, 5, "navigating"); //ya estoy navegando
       // navigationCurrentState = navigationState::REACHED; //ROQME: Objetivo alcanzado
    }
    catch(...)
    {
        try
        {
            worldModel->getEdgeByIdentifiers(3, 5, "finish");
        }
        catch(...)
        {
            if(robotCharging && !doingUnDocking)
            {
                unDocking();
                cout << "Vamos a hacer el undocking" << endl;
                doingUnDocking = true;
            }

            if(!doingUnDocking)
            {
                try
                {
                    worldModel->removeEdgeByIdentifiers(3, 5, "start");
                    worldModel->addEdgeByIdentifiers(3, 5,"navigating");
                    model_modified = true;
                    worldModel->getSymbol(5)->setAttribute("result", "");
                    setGoal();
                    goalPosition.x = str2float(worldModel->getSymbol(5)->getAttribute("x"));	
                    goalPosition.z = str2float(worldModel->getSymbol(5)->getAttribute("z"));	
                    goalPosition.angle = str2float(worldModel->getSymbol(5)->getAttribute("angle"));
                    goToDockStation = (str2int(worldModel->getSymbol(5)->getAttribute("docking"))? true : false); 
                    std::cout << "GotoDocking: " << worldModel->getSymbol(5)->getAttribute("docking") << std::endl;
                    sendGoal = true;
                }
                catch(...)
                {
                }
            }
        }
    }
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



void SpecificWorker::setGoal()
{
    try
    {    
        std::string currentAction_ = worldModel->getSymbol(3)->getAttribute("lastAction");
        std::cout << currentAction_ << std::endl;
        if(currentAction_ != "approach" && currentAction_ != "pickUp")
        {
            if(currentAction_ == "moveToDock")
            {
                readGoalPosition("dock");
            }
            else if(currentAction_ == "moveToStart")
            {
                readGoalPosition("start_a");
            }
            else if(currentAction_ == "move")
            {
                setNearestGoalPosition("deliver_trolley");
            }
            else if (currentAction_ == "move2") //robot does not see the apriltTag, move to second position
            {
                setObjectDetectionPose2();
            }
            else if (currentAction_ == "moveToDeliverArea")
            {
                setNearestGoalPosition("request_trolley");
            }
            else if (currentAction_ == "moveToDefaultDeliverArea")
            {
                readGoalPosition("defaultDeliverArea");
            }
            worldModel->getSymbol(5)->setAttribute("goal", goal);
            worldModel->getSymbol(5)->setAttribute("x", float2str(goalPosition.x));
            worldModel->getSymbol(5)->setAttribute("z", float2str(goalPosition.z));
            
            if(currentAction_ == "rotateLeft")
            {
                worldModel->getSymbol(5)->setAttribute("angle", float2str(goalPosition.angle+0.39));
            }
            else if(currentAction_ == "rotateRight")
            {
                worldModel->getSymbol(5)->setAttribute("angle", float2str(goalPosition.angle-0.79));
            }
            else
            {   
                worldModel->getSymbol(5)->setAttribute("angle", float2str(goalPosition.angle));
            }
        }
        if(worldModel->getSymbol(5)->getAttribute("move") == "backwards")
            movingFordwards = false;
        else 
            movingFordwards = true;
        
     /*   try
        {
            worldModel->getEdgeByIdentifiers(3, 7, "is");
            Pose2D goalPosition;
            goalPosition.x = str2float(worldModel->getSymbol(7)->getAttribute("x"));	
            goalPosition.z = str2float(worldModel->getSymbol(7)->getAttribute("z"));	
            goalPosition.angle = str2float(worldModel->getSymbol(7)->getAttribute("angle"));
            worldModel->getSymbol(5)->setAttribute("x", float2str(goalPosition.x));
            worldModel->getSymbol(5)->setAttribute("z", float2str(goalPosition.z));
            worldModel->getSymbol(5)->setAttribute("angle", float2str(goalPosition.angle));
            
        }
        catch(...){}
       */ 
        try
        {
            worldModel->getEdgeByIdentifiers(3, 8, "is"); //haciendo el delivering
            Pose2D goalPosition;
            goalPosition.x = str2float(worldModel->getSymbol(8)->getAttribute("x"));	
            goalPosition.z = str2float(worldModel->getSymbol(8)->getAttribute("z"));	
            goalPosition.angle = str2float(worldModel->getSymbol(8)->getAttribute("angle"));
            worldModel->getSymbol(5)->setAttribute("x", float2str(goalPosition.x));
            worldModel->getSymbol(5)->setAttribute("z", float2str(goalPosition.z));
            worldModel->getSymbol(5)->setAttribute("angle", float2str(goalPosition.angle));
            
        }
        catch(...){}
            
    }
    catch(...)
    {
    }
}

void SpecificWorker::setObjectDetectionPose2()
{
    if( existsLink(3, 13, "deliver_trolley") )
    {
        readGoalPosition("pick1_b");                
    }
    else if ( existsLink(3, 14, "deliver_trolley" ) )
    {
        readGoalPosition("pick2_b");
    }
    else if ( existsLink(3, 15, "deliver_trolley" ) )
    {
        readGoalPosition("pick3_b");
    }
    else if ( existsLink(3, 16, "deliver_trolley" ) )
    {
        readGoalPosition("pick4_b");
    }
    else
    {
        readGoalPosition("start_b");
    }  
}

void SpecificWorker::setNearestGoalPosition(const std::string& link)
{
    
   
    /*std::string trolleyFull = worldModel->getSymbol(3)->getAttribute("trolleyFull");	
    if (link == "request_trolley" && trolleyFull == "true")
    {
        std::string label_goal = "deliver";
        goalPosition.x = goalPositions[label_goal].x;
        goalPosition.z = goalPositions[label_goal].z;
        goalPosition.angle = goalPositions[label_goal].angle;
        goal = label_goal;
    }
    else
    {*/
        float h = std::numeric_limits<float>::max();
        bool link_exists = false;
        for (unsigned int i = 0; i < 4; i++)
        {
            if (existsLink(3, 13 + i, link))
            {
                link_exists = true;
                std::stringstream ss;
                if(link == "deliver_trolley")
                    ss << "pick" << i+1 << "_a";
                else if(link == "request_trolley")
                    ss << "deliverArea" << i+1;
                    
                float distance = distanceToGoal(ss.str());
                cout << ss.str() << " : " << distance << " " << h << std::endl;
                if (distance < h )
                {
                    h = distance;
                    goalPosition.x = goalPositions[ss.str()].x;
                    goalPosition.z = goalPositions[ss.str()].z;
                    goalPosition.angle = goalPositions[ss.str()].angle;
                    goal = ss.str();
                }
            }
        }
        if(!link_exists)
        {
            std::string label_goal;
            if(link == "deliver_trolley")
                label_goal = "start_a";
            else if (link == "request_trolley")
                label_goal = "defaultDeliverArea";
            goalPosition.x = goalPositions[label_goal].x;
            goalPosition.z = goalPositions[label_goal].z;
            goalPosition.angle = goalPositions[label_goal].angle;
            goal = label_goal;
        }
   // }
}

void SpecificWorker::pickUpTrolley(bool& model_modified)
{
    try
    {
        std::string pickUpAction = worldModel->getSymbol(7)->getAttribute("action");
        if(pickUpAction == "forkingup" && forkingState == IDLE)
        {
            forkUp();
        }
        else if(forkingState == FINISHED)
        {
            worldModel->getSymbol(7)->setAttribute("action","finished");
            model_modified = true;
            forkingState = IDLE;
        }
    }  
    catch(...){}
}


void SpecificWorker::forkUp()
{
    try
    {
        forkingState = FORKINGUP;
        localnavigator_proxy->forkLiftUp();
    }
    catch(...){}
}

float SpecificWorker::distanceToGoal(const std::string& label)
{
    float distance = -1;
    try
    {
        float x0 = str2float(worldModel->getSymbol(9)->getAttribute("x"));
        float z0 =  str2float(worldModel->getSymbol(9)->getAttribute("z"));
        
        float x1 = goalPositions[label].x;
        float z1 = goalPositions[label].z;
        
        distance = abs(x0-x1)+abs(z0-z1);
    }
    catch(...){}
    
    return distance;
       
}

void SpecificWorker::readGoalPosition()
{
    ifstream infile(GOAL_POSITIONS_FILE);
	if(infile.good())
	{
		string label_;
		float x_, z_, angle_;
	
		while(infile >> label_ >> x_ >> z_ >> angle_)
		{
            Pose2D pose;
            pose.x = x_;
            pose.z = z_;
            pose.angle = angle_;
            goalPositions[label_] = pose; 
		}
		infile.close();
	}
}

void SpecificWorker::readGoalPosition(string label)
{
	goalPosition.x = goalPositions[label].x;
	goalPosition.z = goalPositions[label].z;;
	goalPosition.angle = goalPositions[label].angle;;
    goal = label;
}


std::string SpecificWorker::dockingStatus() const
{
	string res;
	try
	{
		res = localnavigator_proxy->dockingStatus();
	}
	catch(Ice::Exception& e)
	{
		qDebug() << "Local navigation component not available!: " << e.what();
	}	

	return res;
}


bool SpecificWorker::isValidLabel(const string& label) const
{
	return (label == "pick1" || label == "pick2"  || label == "dock" || label == "start" || label == "deliver");
}

bool SpecificWorker::publishModelModification()
{
	bool finished = false;
    bool published = false;
	do
	{
		try
		{
            //AGMModelPrinter::printWorld(worldModel);
			AGMMisc::publishModification(worldModel, agmexecutive_proxy, "navigationAgent");
			if(sendGoal)
			{
				qDebug("Enviando objetivo a mira");
				sendGoalToMira(goalPosition.x, goalPosition.z, goalPosition.angle, movingFordwards);
                sendGoal = false;
                qDebug("Enviando objetivo a mira terminado");                    
			}
			if (flagStop)
            {
                try{
                    localnavigator_proxy->stop();
                    flagStop = false;
                }catch(...){}
                
            }
			//qDebug("Modelo publicado correctamente");
			finished = true;
			batteryLevelChanged = false;
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

//Implementation of Navigation interface

void SpecificWorker::reportRobotState(const float distanceToGoal, const float angToGoal, const int timeElapsed, const navigationState &state)
{
	QMutexLocker locker(mutex);
	//cout << "(" << distanceToGoal << ", " << angToGoal << ", " << timeElapsed << ", " << state << ")" << endl;
    if(!flagStop)
    {
        navigationCurrentState = state;
        if(state == navigationState::REACHED)
            std::cout << "REACHED" << std::endl;
        else if(state == navigationState::FAILED)
            std::cout << "FAILED" << std::endl;
        else if(state == navigationState::IDLE)
            std::cout << "IDLE" << std::endl;
    }
	//else
	//	std::cout << "DRIVING" << std::endl;
    /*if(navigationCurrentState == navigationState::REACHED || navigationCurrentState == navigationState::FAILED )
    {
        gotoFinished = true;
        waitGotoResult.wakeAll();
    }*/
    
}

void SpecificWorker::reportRobotPose(const float x, const float z, const float angle)
{
	QMutexLocker locker(mutex);
	//cout << "(" << x << ", " << z << ", " << angle << ")" << endl;
	currentRobotPose.x = x;
	currentRobotPose.z = z;
	currentRobotPose.angle = angle;
}

void SpecificWorker::reportForkLiftState(const string &status)
{
    std::cout << "forklift state: " << status << std::endl;
    if(status == "Up" || status == "Down")
        forkingState = FINISHED;
}

void SpecificWorker::reportAPTSensor(const float distance)
{
    QMutexLocker locker(mutex);
    if(!movingFordwards && distance <= 0.15 && !flagStop)
    {
        std::cout << "reportAPTSensor: " << distance << std::endl;
        navigationCurrentState = navigationState::REACHED;
        flagStop = true;
    }
}

void SpecificWorker::reportLimitSwitchState(const bool &state)
{
    QMutexLocker locker(mutex);
    if(!movingFordwards && state && !flagStop)
    {
        std::cout << "reportLimitSwitchState: " << std::endl;
        navigationCurrentState = navigationState::REACHED;
        flagStop = true;
    }
}

// AGM methods

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::World &w)
{
	mutex->lock();
	if(w.version > worldModel->version)
	{
		AGMModelConverter::fromIceToInternal(w, worldModel);
	
		delete innerModel;
		innerModel = AGMInner::extractInnerModel(worldModel);
		
		// En nuestro caso las actualizaciones del mundo siempre son estructurales
		worldChanged = true;
	}
	mutex->unlock();
}

void SpecificWorker::reportRobotBatteryLevel(const RobotBatteryLevel &batteryInfo)
{
       QMutexLocker locker(mutex);
	
       robotCharging = batteryInfo.charging;
    
#ifdef MIRON_TOOLS  
        static float voltage = 0.0f;
        if(fabs(voltage - batteryInfo.voltage) >= 0.05f)
        {
            voltage = batteryInfo.voltage; 

            cout  << "charging: " << batteryInfo.charging 
                    << ", voltage: " << batteryInfo.voltage 
                    << ", current: " << batteryInfo.current 
                    << ", lifePercent: " << batteryInfo.lifePercent 
                    << ", powerSuppyPresent: " << batteryInfo.powerSupplyPresent 
                    << ", lifeTime: " << batteryInfo.lifeTime;
            
            cout << ", cellVoltage: ";
            int i = 0;
            for (auto iter = batteryInfo.cvoltage.begin(); iter!= batteryInfo.cvoltage.end(); iter++)
            {
                    cout << "[cell " << i++ << ": " << *iter << "]";
            }
            cout << std::flush;
                            
            static string previousBatteryLevel  = "";
    
            if(batteryInfo.voltage >= 26.0f)
            {
                    batteryLevel = "high";
            }
            else if(batteryInfo.voltage >= 25.0f && batteryInfo.voltage < 26.0f)
            {
                    batteryLevel = "medium";
            }
            else if(batteryInfo.voltage < 25.0f && batteryInfo.voltage > 24.5f)
            {
                    batteryLevel = "low";
            }
            else
            {
                    batteryLevel = "verylow";
            }
    
            if(previousBatteryLevel != batteryLevel)
            {
                    previousBatteryLevel = batteryLevel;
                    batteryLevelChanged = true;             
                    cout << "voltage: " << batteryInfo.voltage << ", level: " << batteryLevel << endl;                      
            }
        }
#else
        if(batteryPercentLevel != batteryInfo.lifePercent)
        {
            batteryPercentLevel = batteryInfo.lifePercent;
            batteryLevelChanged = true; 
            cout << "lifepercent: " << batteryPercentLevel << endl;   
        }       
#endif
              
}


void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 	worldChanged = true;

	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	worldChanged = true;
 
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	worldChanged = true;
 
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	worldChanged = true;
 
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
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "navigationagentAgent");
	}
	catch(...)
	{
		exit(1);
	}
}

void SpecificWorker::moveRobotToDockStation() const
{
       try
       {
               localnavigator_proxy->goToDockStation();
       }
       catch(Ice::Exception& e)
       {
               cout << "Local navigation component not available!: " << e.what() << endl;
       }       
}

void SpecificWorker::unDocking() const
{
	try
	{
		localnavigator_proxy->goToBasePoint();
	}
	catch(Ice::Exception& e)
	{
		cout << "Local navigation component not available!: " << e.what() << endl;
	}  
}

void SpecificWorker::sendGoalToMira(const float goalx, const float goalz,const float angle, bool moving_fordwards) const
{
	try
	{
		cout << "(" << goalx << ", " << goalz << ", " << angle << ")" << endl;
        if(moving_fordwards)
            localnavigator_proxy->goTo(goalx, goalz, angle);
        else
            localnavigator_proxy->goBackWardsTo(goalx, goalz, angle);
	}
	catch(Ice::Exception& e)
	{
		cout << "Local navigation component not available!: " << e.what() << endl;
	}		
}

void SpecificWorker::localizeRobot() const
{
	try
	{
		localnavigator_proxy->setOdometry(0, 0, 0);
	}
	catch(Ice::Exception& e)
	{
		cout << "Local navigation component not available!: " << e.what() << endl;
	}
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

	
	//abortActionThread = std::thread(&SpecificWorker::checkAbortAction, this);
    
	return true;
}


void SpecificWorker::checkAbortAction()
{
    std::cout << "checkAbortAction" << std::endl;
    while(1) //TODO: use a finish flag
    {
        bool flag_stop = false;
        try
        {
           	QMutexLocker locker(mutex);
            worldModel->removeEdgeByIdentifiers(3, 5, "abort"); 
            std::cout << "Parando el navegador" << std::endl;
            worldModel->addEdgeByIdentifiers(3, 5, "stopped"); 
            publishModelModification();
            flag_stop = true;
        }
        catch(...)
        {
        }
        if(flag_stop)
        {
            try{
                localnavigator_proxy->stop();
            }catch(...){}
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }    
}

