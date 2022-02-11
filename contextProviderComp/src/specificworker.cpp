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
	roqmeOut.start();
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
	if (worldChanged)
	{
		//std::cout << "WorldChanged == true" << std::endl;
		worldChanged = false;
        currentAction();
		batteryLevel();
        objectIdentified();
        pickedUp();
        delivered();
        goalReached();
        peopleDetected();
	}
}

void SpecificWorker::objectIdentified()
{
    static std::string prev;
    checkResult(6, "ObjectIdentified", prev);
}
 
void SpecificWorker::pickedUp()
{
    static std::string prev;
    if(checkResult(7, "ObjectPickedUp", prev))
    {
        trolleyFull();
    }
}
 
void SpecificWorker::delivered()
{
    static std::string prev;
    if(checkResult(8, "ObjectDelivered", prev))
    {
        trolleyFull();
    }
}

void SpecificWorker::goalReached()
{
    static std::string prev;
    checkResult(5, "GoalReached", prev);
}

bool SpecificWorker::checkResult(int symbolID, std::string contextName, std::string& prev)
{
    bool result = false;
	try
	{
        std::string current = worldModel->getSymbol(symbolID)->getAttribute("result");
		if ( current != prev )
		{
            prev = current;
            if ( current == "OK" ) {
                sendBool (contextName, true);
                result = true;
            } else if ( current == "FAILED" ) {
                sendBool (contextName, false);
            }
		}	
	} 
	catch(...) {} 
	
	return result;
}

void SpecificWorker::currentAction()
{
    try
	{
        static std::string prevAction;
        currentAction_ = worldModel->getSymbol(3)->getAttribute("lastAction");
		if ( currentAction_ != prevAction )
		{
            prevAction = currentAction_;
            sendEnum ("Action", currentAction_);    
		}	
	} 
	catch(...) {} 
}

void SpecificWorker::trolleyFull()
{
    try
    {
        static std::string prev_full;
        const std::string& full = worldModel->getSymbol(3)->getAttribute("trolleyFull");
        if(prev_full != full)
        {
           prev_full = full;
           if(full == "true")
                sendBool ("TrolleyFull", true);
           else
                sendBool ("TrolleyFull", false);
        }
      
    }catch(...){}
}

void SpecificWorker::batteryLevel() 
{
	// Battery Level
	int32_t battery_symbol = worldModel->getIdentifierByType("battery");
	if(battery_symbol!=-1)
	{
		try
		{
			static string previousBatteryLevel;					
			string batteryLevel = worldModel->getSymbol(battery_symbol)->getAttribute("level");  //high medium low verylow
			if ( previousBatteryLevel != batteryLevel ) 
			{
				previousBatteryLevel = batteryLevel;
#ifdef MIRON_TOOLS
				if ( batteryLevel == "high" ) {
					sendEnum ( "BatteryLevel", "HIGH" ); //NO hay VERY_HIGH
                } else if ( batteryLevel == "medium" )  {
					sendEnum ( "BatteryLevel", "HALF" );
                } else if ( batteryLevel == "low" ) {
					sendEnum ( "BatteryLevel", "LOW" );
                } else {// if ( batteryLevel == "verylow" ) 
					sendEnum ( "BatteryLevel", "VERYLOW" );	
                }
#else
            sendInt("BatteryLevel", str2int(batteryLevel));       
#endif
			}
		}
		catch(...)
		{
		}
	}
}


void SpecificWorker::peopleDetected()
{
	static int npersons_prev = -1;
    static std::string prev_enum_value = "";
	try
	{
        int npersons = str2int(worldModel->getSymbol(12)->getAttribute("number"));
		if ( npersons != npersons_prev )
		{
            npersons_prev = npersons;
#ifdef MIRON_TOOLS
            string value;
            if ((npersons > 3) && (prev_enum_value != "FULL")) {
                value = "FULL";
                prev_enum_value = value;
                sendEnum ("PeopleInRoom", value);
            } else if ((npersons >= 1) && (prev_enum_value != "HALF")) {
                value = "HALF";
                prev_enum_value = value;
                sendEnum ("PeopleInRoom", value);
            } else if ((npersons == 0) && (prev_enum_value != "EMPTY")) {
                value = "EMPTY";
                prev_enum_value = value;
                sendEnum ("PeopleInRoom", value);
            }
#else
            sendInt("PeopleInRoom", npersons);
#endif
		
		}	
	} 
	catch(...) {}
}


void SpecificWorker::sendInt(const std::string& name, int value)
{
	//std::cout << "Enviando enumerado: " << name << ": " << value << std::endl;
	RoqmeDDSTopics::RoqmeIntContext intCtx;
	intCtx.name(name);
	intCtx.value().push_back(value);
    std::stringstream  ss;
    ss << value;
	roqmeOut.roqmeDebug(Roqme::RoqmeDebug::ContextType::INT, name, ss.str());
	intWriter.write(intCtx);
}


void SpecificWorker::sendEnum(const std::string& name, const std::string& value)
{
	//std::cout << "Enviando enumerado: " << name << ": " << value << std::endl;
	RoqmeDDSTopics::RoqmeEnumContext enumCtx;
	enumCtx.name(name);
	enumCtx.value().push_back(value);
	roqmeOut.roqmeDebug(Roqme::RoqmeDebug::ContextType::ENUM, name, value);
	enumWriter.write(enumCtx);
}

void SpecificWorker::sendEvent(const std::string& name, const std::string& value)
{
	//std::cout << "Enviando evento: " << name << ": " << value << std::endl;
	RoqmeDDSTopics::RoqmeEventContext eventCtx;
	eventCtx.name(name);
	eventCtx.value().push_back(value);
	roqmeOut.roqmeDebug(Roqme::RoqmeDebug::ContextType::EVENT, name, value);
	eventWriter.write(eventCtx);
}

void SpecificWorker::sendBool(const std::string& name, const bool& value)
{
	//std::cout << "Enviando booleano: " << name << ": " << value << std::endl;
	RoqmeDDSTopics::RoqmeBoolContext boolCtx;
	boolCtx.name(name);
	boolCtx.value().push_back(value);
	if ( value == true ) 
		roqmeOut.roqmeDebug(Roqme::RoqmeDebug::ContextType::BOOLEAN, name, "true");
	else
		roqmeOut.roqmeDebug(Roqme::RoqmeDebug::ContextType::BOOLEAN, name, "false");
	boolWriter.write(boolCtx);
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
	worldChanged = true;
	mutex->unlock();
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 	delete innerModel;
	worldChanged = true;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	delete innerModel;
	worldChanged = true;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 	delete innerModel;
	worldChanged = true;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	delete innerModel;
	worldChanged = true;
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
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "contextProviderAgent");
	}
	catch(...)
	{
		exit(1);
	}
}




