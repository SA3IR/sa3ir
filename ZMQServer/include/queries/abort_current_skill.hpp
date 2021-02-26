/*
    Copyright (C) 2020  University of Extremadura, University of Málaga, Blue Ocean Robotics.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

    Author: Renan Freitas, renan028@gmail.com
            Antonio Bandera, ajbandera@uma.es
    Maintainer: Renan Freitas, renan028@gmail.com
*/

#pragma once

#include "json.hpp"
#include "query.hpp"

namespace zmqserver
{

struct ApproachDist{
  std::unordered_map<std::string, double> values;

  ApproachDist(double dist) {
    values.insert(std::make_pair("1", dist));
  }
  ApproachDist() {
    values.insert(std::make_pair("1", 100));
  }
};

class AbortCurrentSkill : public ChangeParameter<ApproachDist>
{
public:
  AbortCurrentSkill(int id, ApproachDist value) : 
    ChangeParameter<ApproachDist>(id, value) {
      _query["type"] = "abort-current-skill";
      _query["ParameterSetRepository"] = "CommNavigationObjects";
      _query["ParameterSet"] = "CdlParameter";
      _query["Component"] = "SmartCdlServer";
      _query["Parameter"] = "approachdist";
  }
  ~AbortCurrentSkill(){};

  std::string dump() override
  {
    nlohmann::json values;

    values["1"] = _values.values["1"];
    _query["values"] = values;
    return Query::dump();
  }
};

} // namespace zmqserver
