/*
 *    Copyright (C)2021 by University of Málaga
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
   // timer.setSingleShot(true); //compute method is executed only once
	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();
    errorSent = false;
    pickers_action.reserve(4);
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
    QMutexLocker locker(mutex);
    
    //mensajes si todo ha ido como lo esperado
    checkExpectedLinks();
    
    //comprobar si algo fue mal
    checkActionErrors();
    
    //consultamos la base de datos por si hay peticiones pendientes   
    static int cont = 0;
    if(!cont)
    {
        //TODO: Mover a un thread independiente?
        //std::cout << "Vamos a hacer la petición al API REST..." << std::endl;
        if(!httpGetRequest())
        {
        //  std::cout << "Ahora vamos a leer el json y modificar el AGM..." << std::endl;
            readJsonFile();
            for (auto it=pickersActions.begin(); it!=pickersActions.end(); ++it)
            {
                std::cout << it->first << " => " << it->second << '\n';
                if(updateAGM(it->first, it->second))
                {
                    std::stringstream uri;
                    uri << "/api/picker" << it->first << "_remove_action";
                    httpPostRequest(uri.str());
                }
            }
        }
        errorSent = false;
    }
    cont = (cont + 1)%10;
}

bool SpecificWorker::checkExpectedLinks()
{
    bool res = false;
    for(unsigned int i = 0; i < 4; i++)
    {
        if(hasLink(3, 13+i, has_trolley))
        {
            if(pickers_action[i] != has_trolley)
            {
                pickers_action[i] = has_trolley;
                std::stringstream uri;
                uri << "/api/picker" << i+1 << "_" << has_trolley;
                //eliminar status de la base de datos
                httpPostRequest(uri.str());
                res = true;
            }
        } 
        else if(hasLink(3, 13+i, deliver_trolley))
        {
            pickers_action[i] = deliver_trolley;
        }
        else if(!hasLink(3, 13+i, deliver_trolley))
        {
            if(pickers_action[i] == deliver_trolley)
            {
                pickers_action[i] = "";
                 std::stringstream uri;
                uri << "/api/picker" << i+1 << "_" << deliver_trolley;
                //eliminar status de la base de datos
                httpPostRequest(uri.str());
                res = true;
            }
        }
    }
    
    return res;
}

bool SpecificWorker::checkActionErrors()
{
    int i = 1;
    bool error_ = false;
   
    if(!errorSent)
    {
        while(!error_ && i <= 4)
        {
            if(actionDeliverArea(i))
            {
                //std::cout << "actionDeliverArea" << i << std::endl;
                std::stringstream uri;
                uri << "/api/picker" << i << "_";
                if(actionFailed(5))//Navigation failed
                {
                   // std::cout << "navigation" << std::endl;
                    uri << "navigation_error";
                    httpPostRequest(uri.str()); 
                }
                else if(actionFailed(8))//deliver
                {
                    //std::cout << "deliver" << std::endl;
                    uri << "deliver_error";
                    httpPostRequest(uri.str());
                }
                error_ = true;
                errorSent = true;
            }
            else if(actionPickingArea(i))
            {
                //std::cout << "actionPickingArea" << i << std::endl;
                std::stringstream uri;
                uri << "/api/picker" << i << "_";
                if(actionFailed(5))//Navigation failed
                {
                    //std::cout << "Navigation" << std::endl;
                    uri << "navigation_error";
                    httpPostRequest(uri.str()); 
                    
                }
                else if(actionFailed(6))//object recognition
                {
                    //std::cout << "object" << std::endl;
                    uri << "object_error";
                    httpPostRequest(uri.str()); 
                    
                }
                else if(actionFailed(7))//pickUp
                {
                    //std::cout << "pickUp" << std::endl;
                    uri << "pickup_error";
                    httpPostRequest(uri.str()); 
                    
                }
                error_ = true;
                errorSent = true;
            }
            i++;
        }
    }
    
    return error_;
}

bool SpecificWorker::actionDeliverArea(unsigned int id)
{
    bool result = false;
    try
    {
        std::stringstream ss;
        ss << "deliverArea" << id;
        const std::string& goal = worldModel->getSymbol(5)->getAttribute("goal");
        if(goal == ss.str())
        {
            result = true;
        }
    }
    catch(...) {}
    
    return result;
    
}

bool SpecificWorker::actionPickingArea(unsigned int id)
{
    bool result = false;
    try
    {
        std::stringstream ss_a, ss_b;
        ss_a << "pick" << id << "_a";
        ss_b << "pick" << id << "_b";
        const std::string& goal = worldModel->getSymbol(5)->getAttribute("goal");
        if(goal == ss_a.str() || goal == ss_b.str())
        {
            result = true;
        }
    }
    catch(...) {}
    
    return result;
}

bool SpecificWorker::actionFailed(unsigned int nodeId)
{
    bool result = false;
    try
    {
        const std::string& action_res = worldModel->getSymbol(nodeId)->getAttribute("result");
        if(action_res == "FAILED")
            result = true;
    }
    catch(...){}
    
    return result;
}

bool SpecificWorker::hasLink(unsigned int source, unsigned int target, const std::string& link)
{
    bool result = false;
    try
    {
        worldModel->getEdgeByIdentifiers(source, target, link);
        result = true;
    }
    catch(...){}
    
    return result;
}

int SpecificWorker::readJsonFile()
{
    pickersActions.clear();
    ifstream ifs("rest_result.json");
	Json::Reader reader;
	Json::Value root;
	if(reader.parse(ifs, root)){
		for(unsigned int i=0; i<root.size(); i++)
		{
			auto element = root[i];
            std::string pickerID = element["pickerId"].asString();
            std::string action = element["action"].asString();
           // std::cout << "PickerID: " << pickerID << std::endl;
			//std::cout << "Action: " << action << std::endl;
            if(action != "")
                pickersActions[pickerID] = action;
		}
	}
    std::ofstream ofs ("rest_result.json", std::ofstream::out | std::ofstream::trunc); //TODO: something is wrong with the JSON file. Check it!
    ofs.close();
	return 0;
}




int SpecificWorker::httpGetRequest()
{
    return httpRequest(methods::GET, "/api/pickers");
}

int SpecificWorker::httpPostRequest(const std::string& uri)
{
    return httpRequest(methods::POST, uri);
}

int SpecificWorker::httpRequest(const method &mtd, const std::string& uri)
{
    auto fileStream = std::make_shared<concurrency::streams::ostream>();

    // Open stream to output file.
    pplx::task<void> requestTask = concurrency::streams::fstream::open_ostream(U("rest_result.json")).then([=](concurrency::streams::ostream outFile)
    {
        *fileStream = outFile;

        // Create http_client to send the request.
        http_client client(U("http://localhost:8080/"));//TODO: move IP to config file
        //http_client client(U("http://192.168.0.126:8080/"));//TODO: move IP to config file

        // Build request URI and start the request.
        uri_builder builder(U(uri));
        return client.request(mtd, builder.to_string());
    })

    // Handle response headers arriving.
    .then([=](http_response response)
    {
        printf("Received response status code:%u\n", response.status_code());

        // Write response body into the file.
        return response.body().read_to_end(fileStream->streambuf());
    })

    // Close the file stream.
    .then([=](size_t)
    {
       // std::cout << "closing fileStream" << std::endl;
       // fileStream->seek(0); 
        return fileStream->close();
    });

    // Wait for all the outstanding I/O to complete and handle any exceptions
    try
    {
        requestTask.wait();
    }
    catch (const std::exception &e)
    {
        printf("Error exception:%s\n", e.what());
        return 1;
    }
    
   
    
    return 0;
}


bool SpecificWorker::updateAGM(const std::string& pickerID, const::string& action)
{
    
    bool res = false;
    
    QMutexLocker locker(mutex);
    bool model_modified = false;
    unsigned int source = 3, target;
    
    if(pickerID == "1")  target = 13;
    else if(pickerID == "2") target = 14;
    else if(pickerID == "3") target = 15;
    else /*if(pikerID == "4")*/ target = 16;
    
    
    
    if(action == "deliver_trolley")
    {
        try
        {
            worldModel->removeEdgeByIdentifiers(source, target, "has_trolley");
            worldModel->addEdgeByIdentifiers(source, target, action);
            model_modified = true;
        }catch(...){}
    }
    else if (action == "deliver_trolley_full")
    {
        try
        {
            worldModel->removeEdgeByIdentifiers(source, target, "has_trolley");
            worldModel->addEdgeByIdentifiers(source, target, "deliver_trolley");
            worldModel->getSymbol(3)->setAttribute("trolleyFull", "true");	
            model_modified = true;
        }catch(...){}
    }
    else if( action == "request_trolley")
    {
        try
        {
            worldModel->addEdgeByIdentifiers(source, target, action);
            model_modified = true;
        }catch(...){} 
    }
    
    if (model_modified)
         res =  publishModelModification();
    else
        std::cout <<"no se modificó el modelo" << std::endl;
    return res;

}

/*

void SpecificWorker::compute()
{


    
    
    int op;
    do
    {
        std::cout << "Introduce una opción: " << std::endl;   
        std::cout << "1) Pick1: request_trolley" << std::endl;   
        std::cout << "2) Pick1: has_trolley" << std::endl;   
        std::cout << "3) Pick1: deliver_trolley" << std::endl;  
        std::cout << "4) Pick1: deliver_trolley - full" << std::endl;  
        std::cout << "5) Pick2: request_trolley" << std::endl;   
        std::cout << "6) Pick2: has_trolley" << std::endl;   
        std::cout << "7) Pick2: deliver_trolley" << std::endl;
        std::cout << "8) Pick2: deliver_trolley - full" << std::endl;  
        std::cout << "9) Pick3: request_trolley" << std::endl;   
        std::cout << "10) Pick3: has_trolley" << std::endl;   
        std::cout << "11) Pick3: deliver_trolley" << std::endl;   
        std::cout << "12) Pick3: deliver_trolley - full" << std::endl;  
        std::cout << "13) Pick4: request_trolley" << std::endl;   
        std::cout << "14) Pick4: has_trolley" << std::endl;   
        std::cout << "15) Pick4: deliver_trolley" << std::endl;   
        std::cout << "16) Pick4: deliver_trolley - full" << std::endl;  
        std::cout << "0) Salir" << std::endl;   
        std::cout << ">" << std::endl;
        
        std::cin >> op;
        
        QMutexLocker locker(mutex);
        bool model_modified = false;
        if(op == 1)
        {
            
            try
            {
                worldModel->addEdgeByIdentifiers(3, 13, "request_trolley");
                model_modified = true;
            }catch(...){}
        }
        else if (op == 2)
        {
            try
            {
                worldModel->addEdgeByIdentifiers(3, 13, "has_trolley");
                model_modified = true;
            }catch(...){}
        }
        else if (op == 3)
        {
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 13, "has_trolley");
                worldModel->addEdgeByIdentifiers(3, 13, "deliver_trolley");
                model_modified = true;
            }catch(...){}
        }
        else if (op == 4)
        {
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 13, "has_trolley");
                worldModel->addEdgeByIdentifiers(3, 13, "deliver_trolley");
                worldModel->getSymbol(3)->setAttribute("trolleyFull", "true");	
                model_modified = true;
            }catch(...){}
        }
        else if (op == 5)
        {
            try
            {
                worldModel->addEdgeByIdentifiers(3, 14, "request_trolley");
                model_modified = true;
            }catch(...){}
        }
        else if (op == 6)
        {
            try
            {
                worldModel->addEdgeByIdentifiers(3, 14, "has_trolley");
                model_modified = true;
            }catch(...){}
        }
        else if (op == 7)
        {
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 14, "has_trolley");
                worldModel->addEdgeByIdentifiers(3, 14, "deliver_trolley");
                model_modified = true;
            }catch(...){std::cout << "algo fue mal" << std::endl;}
        }
        else if (op == 8)
        {
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 14, "has_trolley");
                worldModel->addEdgeByIdentifiers(3, 14, "deliver_trolley");
                worldModel->getSymbol(3)->setAttribute("trolleyFull", "true");	
                model_modified = true;
            }catch(...){std::cout << "algo fue mal" << std::endl;}
        }
        else if (op == 9)
        {
            try
            {
                worldModel->addEdgeByIdentifiers(3, 15, "request_trolley");
                model_modified = true;
            }catch(...){}
        }
        else if (op == 10)
        {
            try
            {
                worldModel->addEdgeByIdentifiers(3, 15, "has_trolley");
                model_modified = true;
            }catch(...){}
        }
        else if (op == 11)
        {
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 15, "has_trolley");
                worldModel->addEdgeByIdentifiers(3, 15, "deliver_trolley");
                model_modified = true;
            }catch(...){std::cout << "algo fue mal" << std::endl;}
        }
        else if (op == 12)
        {
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 15, "has_trolley");
                worldModel->addEdgeByIdentifiers(3, 15, "deliver_trolley");
                worldModel->getSymbol(3)->setAttribute("trolleyFull", "true");	
                model_modified = true;
            }catch(...){std::cout << "algo fue mal" << std::endl;}
        }
        else if (op == 13)
        {
            try
            {
                worldModel->addEdgeByIdentifiers(3, 16, "request_trolley");
                model_modified = true;
            }catch(...){}
        }
        else if (op == 14)
        {
            try
            {
                worldModel->addEdgeByIdentifiers(3, 16, "has_trolley");
                model_modified = true;
            }catch(...){}
        }
        else if (op == 15)
        {
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 16, "has_trolley");
                worldModel->addEdgeByIdentifiers(3, 16, "deliver_trolley");
                model_modified = true;
            }catch(...){std::cout << "algo fue mal" << std::endl;}
        }
        else if (op == 16)
        {
            try
            {
                worldModel->removeEdgeByIdentifiers(3, 16, "has_trolley");
                worldModel->addEdgeByIdentifiers(3, 16, "deliver_trolley");
                worldModel->getSymbol(3)->setAttribute("trolleyFull", "true");	
                model_modified = true;
            }catch(...){std::cout << "algo fue mal" << std::endl;}
        }
        if(model_modified)
                publishModelModification();
    }while(op!=0);
                        
                    
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
            //AGMModelPrinter::printWorld(worldModel);
			AGMMisc::publishModification(worldModel, agmexecutive_proxy, "navigationAgent");
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
 	AGMModelConverter::fromIceToInternal(w, worldModel);
 
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
//subscribesToCODE
	QMutexLocker lockIM(mutex);
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
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);

}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
//subscribesToCODE
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{
//subscribesToCODE
	QMutexLocker l(mutex);
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
	try
	{
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "webInterfaceAgentAgent");
	}
/*	catch(const RoboCompAGMExecutive::Locked &e)
	{
	}
	catch(const RoboCompAGMExecutive::OldModel &e)
	{
	}
	catch(const RoboCompAGMExecutive::InvalidChange &e)
	{
	}
*/
	catch(const Ice::Exception& e)
	{
		exit(1);
	}
}



bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }




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
