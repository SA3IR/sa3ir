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

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <thread>
#include <memory>
#include <sstream>
#include <ostream>

#include <cpprest/http_client.h>
#include <cpprest/filestream.h>

#include <fstream>
#include <jsoncpp/json/json.h>

using namespace utility;                    // Common utilities like string conversions
using namespace web;                        // Common features like URIs.
using namespace web::http;                  // Common HTTP functionality
using namespace web::http::client;          // HTTP client features
using namespace concurrency::streams;       // Asynchronous streams


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
	void edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications);
	void edgeUpdated(const RoboCompAGMWorldModel::Edge &modification);
	void symbolUpdated(const RoboCompAGMWorldModel::Node &modification);
	void symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications);

public slots:
	void compute();

private:
	InnerModel *innerModel;
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	bool active;
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);
    bool publishModelModification();    
    int readJsonFile();
    int httpGetRequest();
    int httpPostRequest(const std::string& path);
    int httpRequest(const method &mtd, const std::string& uri);
    bool updateAGM(const std::string& pickerID, const::string& action);
    bool hasLink(unsigned int source, unsigned int target, const std::string& link);
    bool actionFailed(unsigned int nodeId);
    bool actionDeliverArea(unsigned int id);
    bool actionPickingArea(unsigned int id);
    bool checkExpectedLinks();
    bool checkActionErrors();
    bool modelModified =false;
    bool errorSent;
    std::map<std::string, std::string> pickersActions;
    std::vector<std::string> pickers_action;
    const std::string has_trolley = "has_trolley";
    const std::string deliver_trolley = "deliver_trolley";
};

#endif
