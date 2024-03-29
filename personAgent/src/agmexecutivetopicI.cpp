/*
 *    Copyright (C) 2022 by YOUR NAME HERE
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
#include "agmexecutivetopicI.h"

AGMExecutiveTopicI::AGMExecutiveTopicI(GenericWorker *_worker)
{
	worker = _worker;
}


AGMExecutiveTopicI::~AGMExecutiveTopicI()
{
}

void AGMExecutiveTopicI::structuralChange(const RoboCompAGMWorldModel::World  &w, const Ice::Current&)
{
	worker->structuralChange(w);
}

void AGMExecutiveTopicI::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence  &modifications, const Ice::Current&)
{
	worker->edgesUpdated(modifications);
}

void AGMExecutiveTopicI::edgeUpdated(const RoboCompAGMWorldModel::Edge  &modification, const Ice::Current&)
{
	worker->edgeUpdated(modification);
}

void AGMExecutiveTopicI::symbolUpdated(const RoboCompAGMWorldModel::Node  &modification, const Ice::Current&)
{
	worker->symbolUpdated(modification);
}

void AGMExecutiveTopicI::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence  &modifications, const Ice::Current&)
{
	worker->symbolsUpdated(modifications);
}

