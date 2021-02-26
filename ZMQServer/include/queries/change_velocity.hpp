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
    Maintainer: Renan Freitas, renan028@gmail.com
*/

#pragma once

#include "json.hpp"
#include "change_parameter.hpp"

namespace zmqserver
{

struct Velocity
{
  std::unordered_map<std::string, double> values;

  Velocity(double vmin, double vmax) {
    values.insert(std::make_pair("1", vmin));
    values.insert(std::make_pair("2", vmax));
  }
  Velocity() {
    values.insert(std::make_pair("1", 0.0));
    values.insert(std::make_pair("2", 0.0));
  }
};

class ChangeVelocity : public ChangeParameter<Velocity>
{
public:
  ChangeVelocity(int id, Velocity values) : ChangeParameter<Velocity>(id, values)
  {
    _query["ParameterSetRepository"] = "CommNavigationObjects";
    _query["ParameterSet"] = "CdlParameter";
    _query["Component"] = "SmartCdlServer";
    _query["Parameter"] = "transvel";
  }
  ~ChangeVelocity(){};

  std::string dump() override
  {
    nlohmann::json values;

    values["1"] = _values.values["1"];
    values["2"] = _values.values["2"];
    _query["values"] = values;
    return Query::dump();
  }
};

} // namespace zmqserver