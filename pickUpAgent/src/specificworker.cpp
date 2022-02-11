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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();
    worldChanged = false;
    flagStop = false;
    pickUpFinished = false;
    pickUpFinishedTmp = false;
    startPickUpProcess = true;
    prev_pick_status = pickUpStatus::IDLE;
    prev_pick_status_tmp = pickUpStatus::IDLE;;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

void SpecificWorker::compute()
{
    QMutexLocker locker(mutex);
   // std::cout << __FILE__ << " " << __LINE__ << std::endl;
    bool modelModified = false;
    
    try
    {
        worldModel->removeEdgeByIdentifiers(3, 7, "abort"); 
        std::cout << "Stopping action..." << std::endl;
        worldModel->addEdgeByIdentifiers(3, 7, "not"); 
        modelModified = true;
        flagStop = true;
    }
    catch(...)
    {
    
        try
        {
            worldModel->getEdgeByIdentifiers(3, 7, "is"); //ya estoy haciendo el pickup
            try{
                std::string forking_action = worldModel->getSymbol(7)->getAttribute("action");
                if(forking_action == "finished")
                {
                   try
                    {
                        worldModel->removeEdgeByIdentifiers(3, 7, "is");
                        worldModel->addEdgeByIdentifiers(3, 7, "finish"); 
                        worldModel->getSymbol(7)->setAttribute("result", "OK"); //TODO: contemplar el caso de que no haya ido bien
                        worldModel->getSymbol(7)->setAttribute("action", "");
                        removeHasAndDeliverLinks(modelModified);
                        modelModified = true;
                    }
                    catch(...){}   
                }
                else if(forking_action == "detection_finished")
                {
                    try
                    {
                        std::cout << "detection_finished" << std::endl;
                        worldModel->removeEdgeByIdentifiers(3, 5, "finish");  //la navegación ha terminado
                        worldModel->addEdgeByIdentifiers(3, 5, "start"); //le indicamos a la navegación un nuevo objetivo
                        worldModel->getSymbol(7)->setAttribute("action", "approaching");
                        if(prev_pick_status_tmp == pickUpStatus::WAITING_FOR_DETECTION)
                        {
                            std::cout << "desactivando laser" << std::endl;
                            laserreporter_proxy->toggleLaserReport(false);
                        }
                        std::cout << "Guardando en AGM: moviendo hacia atrás" << std::endl;
                        moveTrolleyBackwards();
                        modelModified = true;
                        prev_pick_status_tmp = pickUpStatus::FORKINGUP;
                    }
                    catch(...){}
                }
            }
            catch(...){}
            
            if (prev_pick_status == pickUpStatus::ROTATING)
            {
                try
                {
                     //Indicamos al navegador que se mueva
                    std::cout << "Guardando en AGM: rotateTrolley..." << std::endl;
                    worldModel->removeEdgeByIdentifiers(3, 5, "stopped");
                    worldModel->addEdgeByIdentifiers(3, 5, "start"); 
                    worldModel->getSymbol(7)->setAttribute("action", "rotating");
                    rotateTrolley(); //ponemos el objetivo en el AGM (nodo pickup)        
                    modelModified = true;
                    prev_pick_status_tmp = pickUpStatus::DETECTING;
                }
                catch(...){}
            }
            else if (prev_pick_status == pickUpStatus::DETECTING)
            {
                try
                {
                    worldModel->getEdgeByIdentifiers(3, 5, "finish");  //la navegación ha terminado
                    worldModel->getSymbol(7)->setAttribute("action", "waiting_for_detection");
                    if(prev_pick_status_tmp == pickUpStatus::DETECTING)
                    {
                        laserreporter_proxy->toggleLaserReport(true);
                    }
                    std::cout << "Guardando en AGM: DETECTING" << std::endl;
                    
                    modelModified = true;
                    prev_pick_status_tmp = pickUpStatus::WAITING_FOR_DETECTION;
                }
                catch(...){}
            }
            else if (prev_pick_status == pickUpStatus::FORKINGUP)
            {
                try
                {
                    worldModel->removeEdgeByIdentifiers(3, 5, "finish");  //la navegación ha terminado
                    worldModel->addEdgeByIdentifiers(3, 5, "stopped");
                    worldModel->getSymbol(5)->setAttribute("move", "forwards");
                    worldModel->getSymbol(7)->setAttribute("action", "forkingup");
                    modelModified = true;
                    std::cout << "Guardando en AGM: elevando el carrito" << std::endl;
                    prev_pick_status_tmp = pickUpStatus::IDLE;
                }
                catch(...){}
            }
        }
        catch(...)
        {
            try
            {
                worldModel->getEdgeByIdentifiers(3, 7, "finish"); //ya ha terminado, no hago nada
            }
            catch(...)
            {
                try
                {
                    worldModel->removeEdgeByIdentifiers(3, 7, "start"); //el BT indica que empiece el proceso
                    worldModel->addEdgeByIdentifiers(3, 7, "is");
                    worldModel->getSymbol(7)->setAttribute("action", "");
                    worldModel->getSymbol(7)->setAttribute("result", "");
                    modelModified = true;
                    prev_pick_status_tmp = pickUpStatus::ROTATING;
                }
                catch(...){}       
            }
        }
    }
     // Miramos si hay que publicar el modelo del mundo
    if(modelModified)
    {
        publishModelModification();
    }
}

void SpecificWorker::forkUp()
{
    try
    {
        localnavigator_proxy->forkLiftUp();
    }
    catch(...){}
}

void SpecificWorker::rotateTrolley()
{
    Robot2DPoint robotGlobalPose;
    robotGlobalPose.x = str2float(worldModel->getSymbol(9)->getAttribute("x"));;
    robotGlobalPose.z =  str2float(worldModel->getSymbol(9)->getAttribute("z"));;
    robotGlobalPose.angle = str2float(worldModel->getSymbol(9)->getAttribute("angle"));;
    
    Robot2DPoint robotGoal;
    robotGoal.x = robotGlobalPose.x;
    robotGoal.z = robotGlobalPose.z;
    robotGoal.angle = robotGlobalPose.angle + 3.14;
    
    std::cout << "robotGlobalPose: " << robotGlobalPose.x << " " << robotGlobalPose.z << " " << robotGlobalPose.angle << std::endl;
    std::cout << "robotGoal: " << robotGoal.x << " " << robotGoal.z << " " << robotGoal.angle << std::endl;
    
    updateGoal(robotGoal);
}

void SpecificWorker::moveTrolleyBackwards()
{
    Robot2DPoint robotGoal;
    robotGoal.x = str2float(worldModel->getSymbol(7)->getAttribute("x"));
    robotGoal.z =  str2float(worldModel->getSymbol(7)->getAttribute("z"));
    robotGoal.angle = str2float(worldModel->getSymbol(7)->getAttribute("angle"));
    updateGoal(robotGoal);
}

void SpecificWorker::updateGoal(const Robot2DPoint& robotGoal)
{
    std::cout << "robotGoal: (x = " << robotGoal.x  <<  ",z = " << robotGoal.z << ", angle = " << robotGoal.angle  << ")" << std::endl;
    //Ponemos el goal en el nodo navegación 
    worldModel->getSymbol(5)->setAttribute("x", float2str(robotGoal.x));
    worldModel->getSymbol(5)->setAttribute("z", float2str(robotGoal.z));
    worldModel->getSymbol(5)->setAttribute("angle", float2str(robotGoal.angle));
    worldModel->getSymbol(5)->setAttribute("move", "backwards");
    /*try{
        localnavigator_proxy->goBackWardsTo(robotGoal.x, robotGoal.z, robotGoal.angle);
    }
    catch(...){}
    */
    
}

void SpecificWorker::removeHasAndDeliverLinks(bool& modelModified)
{
    try
    {
        std::string goal = worldModel->getSymbol(5)->getAttribute("goal");
        if( goal == "pick1_a" || goal == "pick1_b")
        {
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 13, "has_trolley");
            }catch(...){}
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 13, "deliver_trolley");
            }catch(...){}
            // worldModel->addEdgeByIdentifiers(3, 13, "idle");
            modelModified = true;
        }
        else if ( goal == "pick2_a" || goal == "pick2_b")
        {
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 14, "has_trolley");
            }catch(...){}
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 14, "deliver_trolley");
            }catch(...){}
            //worldModel->addEdgeByIdentifiers(3, 14, "idle");
            modelModified = true;
        }
        else if ( goal == "pick3_a" || goal == "pick3_b")
        {
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 15, "has_trolley");
            }catch(...){}
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 15, "deliver_trolley");
            }catch(...){}
            //worldModel->addEdgeByIdentifiers(3, 14, "idle");
            modelModified = true;
        }
        else if ( goal == "pick4_a" || goal == "pick4_b")
        {
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 16, "has_trolley");
            }catch(...){}
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 16, "deliver_trolley");
            }catch(...){}
            //worldModel->addEdgeByIdentifiers(3, 14, "idle");
            modelModified = true;
        }
    }
    catch(...){}
}

void SpecificWorker::publishModelModification()
{
	bool finished = false;
	do
	{
		try
		{
			AGMMisc::publishModification(worldModel, agmexecutive_proxy, "pickUpAgent");
            std::cout << __FILE__ << " " << __LINE__ << std::endl;
           // if(prev_pick_status ==  pickUpStatus::APPROACHING))
            //{
            //    prev_pick_status_tmp = pickUpStatus::FORKINGUP;
           // }
            prev_pick_status = prev_pick_status_tmp;
            finished = true;
			
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
}

void SpecificWorker::pickUpProcess()
{
    std::cout << "Cogiendo objeto ..." << std::endl;
    pickUpFinished = true;    
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
	mutex->lock();
 	AGMModelConverter::fromIceToInternal(w, worldModel);
 
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
	mutex->unlock();
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 
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
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "pickUpAgent");
	}
	catch(...)
	{
		exit(1);
	}
}



bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{


//	THE FOLLOWING IS JUST AN EXAMPLE for AGENTS
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("NameAgent.InnerModel") ;
//		if( QFile(QString::fromStdString(par.value)).exists() == true)
//		{
//			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
//			innerModel = new InnerModel(par.value);
//			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file read OK!" ;
//		}
//		else
//		{
//			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file " << QString::fromStdString(par.value) << " does not exists";
//			qFatal("Exiting now.");
//		}
//	}
//	catch(std::exception e)
//	{
//		qFatal("Error reading config params");
//	}

	
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


// ----------------------------------------------------------
// PARAMETERS:
// poseRobot: X,Z,angle (radians) of the robot in global's coordinates
// poseLocal: X,Z,angle (radians) of the target pose in robot's coordinates
// RESULT:
// A poseMIRA struct containing X,Z,angle (radians) of the target pose in global's coordinates
// ----------------------------------------------------------
Robot2DPoint SpecificWorker::calculaPoseGlobalMIRASA3IR(const Robot2DPoint& poseRobot, const Robot2DPoint& poseLocal)
{
	Robot2DPoint poseGlobal;

	// Directly taken from:
	// Transform_global = Transform_global_robot * Transform_local_target
	// where a transform matrix is:
	//	c(phi)	-s(phi)	Tx
	//	s(phi)	c(phi)	Tz
	//	0		0		1

	poseGlobal.x = cos(poseRobot.angle)*poseLocal.x - sin(poseRobot.angle)*poseLocal.z + poseRobot.x;
	poseGlobal.z = sin(poseRobot.angle)*poseLocal.x + cos(poseRobot.angle)*poseLocal.z + poseRobot.z;
	poseGlobal.angle = atan2( cos(poseRobot.angle)*sin(poseLocal.angle) + sin(poseRobot.angle)*cos(poseLocal.angle), cos(poseRobot.angle)*cos(poseLocal.angle) - sin(poseRobot.angle)*sin(poseLocal.angle) );

	return poseGlobal;
}

