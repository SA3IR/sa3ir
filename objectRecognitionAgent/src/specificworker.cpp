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
    objectRecognitionFinished = false;
    startObjectRecognition = false;
    timerStarted = false;
    readingFromLaser = false;
    new_data_available = false;
    detectingLaserState = IDLE;
    timerLaserDetectionStarted = false;
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
    
    bool modelModified = false;
    
    try
    {
        worldModel->removeEdgeByIdentifiers(3, 6, "abort"); 
        std::cout << "Parando el navegador" << std::endl;
        worldModel->addEdgeByIdentifiers(3, 6, "not"); 
        modelModified = true;
        flagStop = true;
    }
    catch(...)
    {
        
        try
        {
            worldModel->getEdgeByIdentifiers(3, 6, "is"); //ya estoy buscando el objeto
            if(!timerStarted) {
                startTime = std::chrono::system_clock::now();
                timerStarted = true;
            }
            else {
                endTime = std::chrono::system_clock::now();
                if(std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() >= 5000) { //we abort after 5 seconds
                    timerStarted = false;
                    try
                    {
                        worldModel->removeEdgeByIdentifiers(3, 6, "is");
                        modelModified = true;
                        worldModel->addEdgeByIdentifiers(3, 6, "failed"); //TODO: eliminar este enlace y usar el atributo en su lugar
                        modelModified = true;
                        worldModel->getSymbol(6)->setAttribute("result", "FAILED");
                    }
                    catch(...)
                    {
                            
                    }   
                }
                else  if(objectRecognitionFinished)
                {
                    try
                    {
                        worldModel->removeEdgeByIdentifiers(3, 6, "is");
                        modelModified = true;
                        worldModel->addEdgeByIdentifiers(3, 6, "finish"); 

                        modelModified = true;
                        
                        worldModel->getSymbol(6)->setAttribute("result", "OK");
                        
                        //TODO: refactorizar todo esto!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        
                        //*********************** Goal position for navigation process ***************
                        Robot2DPoint robotGlobalPose;
                        robotGlobalPose.x = str2float(worldModel->getSymbol(9)->getAttribute("x"));;
                        robotGlobalPose.z =  str2float(worldModel->getSymbol(9)->getAttribute("z"));;
                        robotGlobalPose.angle = str2float(worldModel->getSymbol(9)->getAttribute("angle"));;
                        
                        Robot2DPoint robotLocalPose;
                        robotLocalPose.x = robotGoalFromCamera.z;
                        robotLocalPose.z = -robotGoalFromCamera.x;
                        robotLocalPose.angle = -robotGoalFromCamera.angle;
                       
                        Robot2DPoint robotGoal = calculaPoseGlobalMIRASA3IR (robotGlobalPose, robotLocalPose); 
                        std::cout << "robotGoal: (x = " << robotGoal.x  <<  ",z = " << robotGoal.z << ", angle = " << robotGoal.angle  << ")" << std::endl;
                        
                        //Ponemos el goal en el nodo navegación 
                        worldModel->getSymbol(5)->setAttribute("x", float2str(robotGoal.x));
                        worldModel->getSymbol(5)->setAttribute("z", float2str(robotGoal.z));
                        worldModel->getSymbol(5)->setAttribute("angle", float2str(robotGoal.angle));
                        worldModel->getSymbol(5)->setAttribute("angle_correction", float2str(angle_correction));
                        //*******************************************************************************
                        
                        
                        // *********************  Goal position for the pickUp process ********************
                        robotLocalPose.x = pickUpPoint.z;
                        robotLocalPose.z = -pickUpPoint.x;
                        robotLocalPose.angle = -pickUpPoint.angle;
                       
                        robotGoal = calculaPoseGlobalMIRASA3IR (robotGlobalPose, robotLocalPose); 
                        std::cout << "robotGoalPickUp: (x = " << robotGoal.x  <<  ",z = " << robotGoal.z << ", angle = " << robotGoal.angle  << ")" << std::endl;
                        
                        //Ponemos el goal en el pickUp 
                        worldModel->getSymbol(7)->setAttribute("x", float2str(robotGoal.x));
                        worldModel->getSymbol(7)->setAttribute("z", float2str(robotGoal.z));
                        worldModel->getSymbol(7)->setAttribute("angle", float2str(robotGoal.angle));
                        //****************************************************
                       
                        // *********************  Goal position for the delivery finished process ********************
                        robotLocalPose.x = deliverFinishedPoint.z;
                        robotLocalPose.z = -deliverFinishedPoint.x;
                        robotLocalPose.angle = -deliverFinishedPoint.angle;
                       
                        robotGoal = calculaPoseGlobalMIRASA3IR (robotGlobalPose, robotLocalPose); 
                        std::cout << "deliverFinishedPoint: (x = " << robotGoal.x  <<  ",z = " << robotGoal.z << ", angle = " << robotGoal.angle  << ")" << std::endl;
                        
                        //Ponemos el goal para finalizar el delivery 
                        worldModel->getSymbol(8)->setAttribute("x", float2str(robotGoal.x));
                        worldModel->getSymbol(8)->setAttribute("z", float2str(robotGoal.z));
                        worldModel->getSymbol(8)->setAttribute("angle", float2str(robotGoal.angle));
                        //****************************************************
                        
                        
                        modelModified = true;
                        timerStarted = false;
                    }
                    catch(...)
                    {
                            
                    }   
                }
            }
        }
        catch(...)
        {
            
            try
            {
                worldModel->getEdgeByIdentifiers(3, 6, "finish"); //ya ha terminado, no hago nada
            }
            catch(...)
            {
                try
                {
                    worldModel->removeEdgeByIdentifiers(3, 6, "start"); //el BT indica que empiece el proceso, se elimina y se pone un "is"
                    modelModified = true;
                    worldModel->addEdgeByIdentifiers(3, 6, "is"); 
                    modelModified = true;
                    worldModel->getSymbol(6)->setAttribute("result", "");
                }
                catch(...)
                {
                        
                }       
            }
            
            //**** Estoy hay que probarl
            laserDetection(modelModified);
        }
    }
    // Miramos si hay que publicar el modelo del mundo
    if(modelModified)
    {
        publishModelModification();
    }
}

void SpecificWorker::laserDetection(bool& model_modified)
{
    try
    {
        std::string pickUpAction = worldModel->getSymbol(7)->getAttribute("action");
        if(pickUpAction == "waiting_for_detection" && detectingLaserState == IDLE)
        {
            readingFromLaser = true;
            detectingLaserState = DETECTING;
            if(!timerLaserDetectionStarted){
                std::cout << "Iniciando timer" << std::endl;
                startLaserDetectionTime = std::chrono::system_clock::now();
            }
        }
        else if(detectingLaserState == DETECTING)
        {
            endLaserDetectionTime = std::chrono::system_clock::now();
            if(std::chrono::duration_cast<std::chrono::milliseconds>(endLaserDetectionTime - startLaserDetectionTime).count() >= 2000) { //we abort after 5 seconds
                timerLaserDetectionStarted = false;
                std::cout << "Finalizando timer" << std::endl;
                detectingLaserState = FINISHED;
            }
        }

        if(detectingLaserState == FINISHED)
        {
            worldModel->getSymbol(7)->setAttribute("action","detection_finished");
            model_modified = true;
            std::cout << "vamos a ver si hay datos" << std::endl;
            if(new_data_available) {
                //*********************  Goal position for the pickUp process ********************
                Robot2DPoint robotGlobalPose;
                robotGlobalPose.x = str2float(worldModel->getSymbol(9)->getAttribute("x"));;
                robotGlobalPose.z =  str2float(worldModel->getSymbol(9)->getAttribute("z"));;
                robotGlobalPose.angle = str2float(worldModel->getSymbol(9)->getAttribute("angle"));;
                
                Robot2DPoint robotLocalPose;
                robotLocalPose.x = -pickUpPointLaser.z;
                robotLocalPose.z = -pickUpPointLaser.x;
                robotLocalPose.angle = -pickUpPointLaser.angle;
                
                Robot2DPoint robotGoal = calculaPoseGlobalMIRASA3IR (robotGlobalPose, robotLocalPose); 
                std::cout << "robotGlobalPoseUppppppppppppppppp: (x = " << robotGlobalPose.x  <<  ",z = " << robotGlobalPose.z << ", angle = " << robotGlobalPose.angle  << ")" << std::endl;
                std::cout << "robotLocalPoseUppppppppppppppppp: (x = " << robotLocalPose.x  <<  ",z = " << robotLocalPose.z << ", angle = " << robotLocalPose.angle  << ")" << std::endl;
                std::cout << "robotGoalPickUppppppppppppppppp: (x = " << robotGoal.x  <<  ",z = " << robotGoal.z << ", angle = " << robotGoal.angle  << ")" << std::endl;
                std::cout << "PickUppppppppppppppppp: (x = " << pickUpPointLaser.x  <<  ",z = " << pickUpPointLaser.z << ", angle = " << pickUpPointLaser.angle << ")" << std::endl;
                
                //Ponemos el goal en el pickUp 
                worldModel->getSymbol(7)->setAttribute("x", float2str(robotGoal.x));
                worldModel->getSymbol(7)->setAttribute("z", float2str(robotGoal.z));
                worldModel->getSymbol(7)->setAttribute("angle", float2str(robotGoal.angle));
                                
                model_modified = true;
                readingFromLaser = false;
                new_data_available = false;
                detectingLaserState = IDLE;
            }
        }
    }  
    catch(...)
    {
        if(detectingLaserState != IDLE)
        {
            detectingLaserState = IDLE;
        }
    }
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

void SpecificWorker::newTagBasedPose(const float x, const float z, const float alpha)
{
    QMutexLocker locker(mutex);
    if(timerStarted) {
        robotGoalFromCamera.x = x;
        robotGoalFromCamera.z = z;
        robotGoalFromCamera.angle = alpha;
        std::cout << "newTagBasedPose: " << x << " " << z << " " << alpha << std::endl; 
        objectRecognitionFinished = true;
    }
}

void SpecificWorker::pickUpPoseInfo(const double& x, const double& z, const double& alpha)
{
    QMutexLocker locker(mutex);
    if(timerStarted || readingFromLaser) {
        if(readingFromLaser){
            std::cout << "recibiendo lecturas del laser" << std::endl;
            new_data_available = true;
            pickUpPointLaser.x = x;
            pickUpPointLaser.z = z;
            pickUpPointLaser.angle = alpha;
        }
        else
        {
            pickUpPoint.x = x;
            pickUpPoint.z = z;
            pickUpPoint.angle = alpha;
        }
    }
}

void SpecificWorker::deliverFinishedPoseInfo(const double& x, const double& z, const double& alpha)
{
    QMutexLocker locker(mutex);
    if(timerStarted) {
        deliverFinishedPoint.x = x;
        deliverFinishedPoint.z = z;
        deliverFinishedPoint.angle = alpha;
    }
}

void SpecificWorker::trolleyPoseInfo(const double& x, const double& z, const double& alpha, const TrolleyRectangle& lines)
{
    QMutexLocker locker(mutex);
   if(timerStarted) {
        robotGoalFromCamera.x = x;
        robotGoalFromCamera.z = z;
        robotGoalFromCamera.angle = alpha;
        std::cout << "trolleyPoseInfo: " << x << " " << z << " " << alpha << std::endl; 
        objectRecognitionFinished = true;
        
        if(lines.size() == 4) //four lines means that the robot sees all the wheels
        {
            double angle1 = lines[2].angle;
            double angle2 = lines[3].angle;
            double angle1_aux, angle2_aux;
            //std::cout << angle1 << " " << angle2 << std::endl;
            
            //TODO: Another solution: test taking the line with lower length
            
            if(angle1 >= 0)
                angle1_aux = 90 - angle1;
            else
                angle1_aux = abs(angle1)-90 ;
            
            
            if(angle2 >= 0)
                angle2_aux = 90 - angle2;
            else
                angle2_aux = abs(angle2)-90 ;
            
            angle_correction = (angle1_aux+angle2_aux)/2;
            
        }
        else if(lines.size() == 2) //three lines means that the robot sees only three the wheels
        {
            //tengo que ver de donde viene
            double angle = lines[1].angle;
            //std::cout << angle << std::endl;
            if(angle >= 0)
                angle_correction = 90 - angle;
            else
                angle_correction = abs(angle)-90 ;
            
        }
    }
}

void SpecificWorker::publishModelModification()
{
	bool finished = false;
	do
	{
		try
		{
			AGMMisc::publishModification(worldModel, agmexecutive_proxy, "objectRecognitionAgent");
            objectRecognitionFinished = false;
            finished = true;
            //if (flagStop) ...
			
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
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "objectRecognitionAgent");
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

