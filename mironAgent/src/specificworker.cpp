 /*
 *    Copyright (C)2020 by University of Málaga
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
    timer.setSingleShot(true); //compute method is executed only once
	active = false;
	dataAvailable = false;
    finishRecv= false;
    newModelAvailable = false;
    worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();
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
	
    zmq::context_t context;
  	zmq::socket_t replay_socket(context, ZMQ_REP);

	replay_socket.bind("tcp://127.0.0.1:5556");

    while(!finishRecv) 
    {
        zmq::message_t request;
        //  Wait for next request from client
        try
        {
            bool not_timeout = replay_socket.recv (&request);
            if( not_timeout )
            {
                //  Send reply back to client TODO should be used to eval if the request is valid
                zmq::message_t ack;
                replay_socket.send(ack);
                std::string msg = std::string(static_cast<char*>(request.data()), request.size());     
                std::cout << "mensaje recibido: " << msg << std::endl;
                abortAction();
            } 
            else 
            {
                std::cout << "timeout!" << std::endl; 
            }
        }
        catch(zmq::error_t& err)
        {
            std::cout<<"ELSE: Error reading from socket "<<std::endl;
            finishRecv = true;
        }   
    }
}

void SpecificWorker::abortAction() 
{
    QMutexLocker locker(mutex);

    bool modelModified = false;

    //Deliver
    try
    {
        try {
            worldModel->removeEdgeByIdentifiers(3, 8, "start");
            worldModel->addEdgeByIdentifiers(3, 8, "abort"); 
            modelModified = true;
        }catch(...){}
        try{
            worldModel->removeEdgeByIdentifiers(3, 8, "is");
            worldModel->addEdgeByIdentifiers(3, 8, "abort"); 
            modelModified = true;
        }catch(...){}
        try{
            worldModel->removeEdgeByIdentifiers(3, 8, "finish");
            worldModel->addEdgeByIdentifiers(3, 8, "abort"); 
            modelModified = true;
        }catch(...){}
    }
    catch(...){}
    
    //Navigation
    try
    {
         try {
            worldModel->removeEdgeByIdentifiers(3, 5, "start");
            worldModel->addEdgeByIdentifiers(3, 5, "abort"); 
            modelModified = true;
        }catch(...){}
        try{
            worldModel->removeEdgeByIdentifiers(3, 5, "navigating");
            worldModel->addEdgeByIdentifiers(3, 5, "abort"); 
            modelModified = true;
        }catch(...){}
        try{
            worldModel->removeEdgeByIdentifiers(3, 5, "finish");
            worldModel->addEdgeByIdentifiers(3, 5, "abort"); 
            modelModified = true;
        }catch(...){}
    }
    catch(...){}
    
    //ObjectRecognition
    try
    {
        try {
            worldModel->removeEdgeByIdentifiers(3, 6, "start");
            worldModel->addEdgeByIdentifiers(3, 6, "abort"); 
            modelModified = true;
        }catch(...){}
        try{
            worldModel->removeEdgeByIdentifiers(3, 6, "is");
            worldModel->addEdgeByIdentifiers(3, 6, "abort"); 
            modelModified = true;
        }catch(...){}
        try{
            worldModel->removeEdgeByIdentifiers(3, 6, "finish");
            worldModel->addEdgeByIdentifiers(3, 6, "abort"); 
            modelModified = true;
        }catch(...){}
       
    }
    catch(...){}
    
    //PickUp
    try
    {
        try {
            worldModel->removeEdgeByIdentifiers(3, 7, "start");
            worldModel->addEdgeByIdentifiers(3, 7, "abort"); 
            modelModified = true;
        }catch(...){}
        try{
            worldModel->removeEdgeByIdentifiers(3, 7, "is");
            worldModel->addEdgeByIdentifiers(3, 7, "abort"); 
            modelModified = true;
        }catch(...){}
        try{
            worldModel->removeEdgeByIdentifiers(3, 7, "finish");
            worldModel->addEdgeByIdentifiers(3, 7, "abort"); 
            modelModified = true;
        }catch(...){}        
    }
    catch(...){}
    
    if(modelModified)
    {
        sendModificationProposal(worldModel, worldModel);
    }
}

bool SpecificWorker::reloadConfigAgent()
{
    //implementCODE
	return true;
}

bool SpecificWorker::activateAgent(const ParameterMap &prs)
{
//implementCODE
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
//implementCODE
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

ParameterMap SpecificWorker::getAgentParameters()
{
//implementCODE
	return params;
}

void SpecificWorker::killAgent()
{
//implementCODE

}

int SpecificWorker::uptimeAgent()
{
//implementCODE
	return 0;
}

bool SpecificWorker::deactivateAgent()
{
//implementCODE
	return deactivate();
}

StateStruct SpecificWorker::getAgentState()
{
//implementCODE
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
//subscribesToCODE
	QMutexLocker lockIM(mutex);
    newModelAvailable = true;
    waitNewModel.wakeAll();
 	AGMModelConverter::fromIceToInternal(w, worldModel);
 
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
//subscribesToCODE
	QMutexLocker lockIM(mutex);
    newModelAvailable = true;
    waitNewModel.wakeAll();
	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);
	}

}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
//subscribesToCODE
	QMutexLocker locker(mutex);
    newModelAvailable = true;
    waitNewModel.wakeAll();
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);

}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
//subscribesToCODE
	QMutexLocker locker(mutex);
    newModelAvailable = true;
    waitNewModel.wakeAll();
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{
//subscribesToCODE
	QMutexLocker l(mutex);
    newModelAvailable = true;
    waitNewModel.wakeAll();
	for (auto modification : modifications)
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

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
	bool finished = false;
	do
	{
		try
		{
			AGMMisc::publishModification(newModel, agmexecutive_proxy, "mironAgent");
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
			//Esperamos a que llegue un nuevo modelo con el que poder abortar
            newModelAvailable = false;
            while(!newModelAvailable)
                waitNewModel.wait(mutex);
            abortAction();
            finished = true;
		}
		catch(...)
		{
			qDebug() << "Error en el método AGM::publishModification. Lo vuelvo a intentar en 500ms";
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
	}
	while(!finished);
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

