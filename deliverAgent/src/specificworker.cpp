/*
 *    Copyright (C) 2020 by YOUR NAME HERE
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
    deliverFinished = false;
    flagStop = false;
    startDeliverProcess = true;
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
        worldModel->removeEdgeByIdentifiers(3, 8, "abort"); 
        std::cout << "Stopping delivering task.." << std::endl;
        worldModel->addEdgeByIdentifiers(3, 8, "not"); 
        modelModified = true;
        flagStop = true;
    }
    catch(...)
    {
        try
        {
            worldModel->getEdgeByIdentifiers(3, 8, "is"); //ya estoy buscando el objeto
            if(deliverFinished)
            {
                try
                {
                    worldModel->removeEdgeByIdentifiers(3, 8, "is");
                    worldModel->addEdgeByIdentifiers(3, 8, "finish"); 
                    worldModel->getSymbol(8)->setAttribute("result", "OK"); //TODO: suponemos que va bien. Si falla, poner "FAILED"
                    worldModel->getSymbol(3)->setAttribute("trolleyFull", "false");	
                    //Si va bien se añade el enlace "pickedUp", sino otro que lo indique.
                    /*try{
                        worldModel->removeEdgeByIdentifiers(3, 10, "pickedUp");
                    }catch(...){}
                    worldModel->addEdgeByIdentifiers(3, 10, "delivered"); */
                    removeRequestTrolley(modelModified);
                    deliverFinished = false;
                }
                catch(...)
                {
                        
                }   
            }
        }
        catch(...)
        {
            
            try
            {
            worldModel->getEdgeByIdentifiers(3, 8, "finish"); //ya ha terminado, no hago nada
            }
            catch(...)
            {
                try
                {
                    worldModel->removeEdgeByIdentifiers(3, 8, "start"); //el BT indica que empiece el proceso
                    worldModel->addEdgeByIdentifiers(3, 8, "is"); 
                    worldModel->getSymbol(8)->setAttribute("result", "");
                    startDeliverProcess = true;
                    modelModified = true;
                }
                catch(...)
                {}       
            }
        }
    }
    // Miramos si hay que publicar el modelo del mundo
    if(modelModified)
    {
        publishModelModification();
    }
}


void SpecificWorker::removeRequestTrolley(bool& modelModified)
{
    try
    {
        std::string goal = worldModel->getSymbol(5)->getAttribute("goal");
        if( goal == "pick1_a" || goal == "pick1_b" || goal == "deliverArea1")
        {
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 13, "request_trolley");
            }catch(...){}
            worldModel->addEdgeByIdentifiers(3, 13, "has_trolley");
            
        }
        else if (goal == "pick2_a" || goal == "pick2_b" || goal == "deliverArea2")
        {
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 14, "request_trolley");
            }catch(...){}
            worldModel->addEdgeByIdentifiers(3, 14, "has_trolley");
            
        }
        else if (goal == "pick3_a" || goal == "pick3_b" || goal == "deliverArea3")
        {
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 15, "request_trolley");
            }catch(...){}
            worldModel->addEdgeByIdentifiers(3, 15, "has_trolley");
            
        }
        else if (goal == "pick4_a" || goal == "pick4_b" || goal == "deliverArea4")
        {
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 16, "request_trolley");
            }catch(...){}
            worldModel->addEdgeByIdentifiers(3, 16, "has_trolley");
            
        }
        modelModified = true;
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
			AGMMisc::publishModification(worldModel, agmexecutive_proxy, "deliverAgent");
            if(startDeliverProcess)
            {
                deliverProcess();
                startDeliverProcess = false;
                //if (flagStop) ...
            }
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

    
void SpecificWorker::deliverProcess()
{
    std::cout << "Soltando objeto ..." << std::endl;
    try
    {
        //implementación
        std::cout << "OK!" << std::endl;
        deliverFinished = true;    
    }catch(...){} 
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
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "deliverAgent");
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

