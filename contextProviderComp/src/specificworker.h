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
#include <RoqmeWriterImpl.h>
#include <RoqmeDebug.h>

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

public slots:
	void compute(); 	

private:
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	InnerModel *innerModel;
	bool worldChanged;
	bool active;
	std::string currentAction_;

	Roqme::RoqmeEventWriter eventWriter;
    Roqme::RoqmeIntWriter intWriter;
	Roqme::RoqmeEnumWriter enumWriter;
	Roqme::RoqmeBoolWriter boolWriter;
	Roqme::RoqmeDebug roqmeOut;

	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);
    
    void currentAction();
	void batteryLevel();
    void peopleDetected();
    void goalReached();
    void objectIdentified();
    void pickedUp();
    void delivered();
    bool checkResult(int symbolID, std::string contextName, std::string& prev);
    void trolleyFull();
	

    void sendInt(const std::string& name, int value);
	void sendEvent(const std::string& name, const std::string& value);
	void sendEnum(const std::string& name, const std::string& value);
	void sendBool(const std::string& name, const bool& value);
	
};

#endif

