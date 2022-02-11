/* 
 * Copyright 2022 University of Málaga <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef _TROLLEYDATYPES_
#define _TROLLEYDATYPES_


#include <TagBasedLocalization.h>
#include <MiraLaser.h>

namespace sa3ir 
{
/**
 * @brief Rotation/Translation matrix
 * 
 */
struct tag_pose_t{
    double r[9];
    double t[3];
};

/**
 * @brief Point2D 
 * 
 */
/*
struct PointT
{
    float x;
    float y;
    float angle;
    float range;

    std::tuple<const float&, const float&, const float&, const float&> tuple() const
    {
        return std::tie(x, y, angle, range);
    }
};
*/
/**
 * @brief Object line with length and angle
 * 
 */
/*
struct LineEstimation
{
    ::std::string id;
    double distance;
    double angle;

    std::tuple<const ::std::string&, const double&, const double&> tuple() const
    {
        return std::tie(id, distance, angle);
    }
};*/

enum ObjectClassType
{
    TROLLEY,
    WHEEL,
    PERSON
};

/**
 * @brief Laser info from Mira
 * 
 */
//using LaserDataT = std::vector<PointT>;

/**
 * @brief Trolley rectangle created from wheels positions
 * 
 */
//using TrolleyRectangle = std::vector<LineEstimation>;

}
#endif