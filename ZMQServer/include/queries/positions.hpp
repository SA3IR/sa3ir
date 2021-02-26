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
#include "query.hpp"

namespace zmqserver{

class Positions : public Query
{
    public:
        Positions(int id) : 
            Query(id)
        {
            _query["type"] = "get-all-positions";
        }
        ~Positions(){};
};

}