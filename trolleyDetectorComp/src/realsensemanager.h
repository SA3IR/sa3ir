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

#ifndef _REALSENSEMANAGER_
#define _REALSENSEMANAGER_

#include <thread>
//#include <samples/ocv_common.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/highgui.hpp>
#include <librealsense2/rsutil.h> 
#include <librealsense2/rs.hpp> 
#include "trolleydatatypes.h" 

using namespace RoboCompTagBasedLocalization;
using namespace RoboCompMiraLaser;

namespace sa3ir
{

/**
 * @brief RealSenseManager. 
 * This class allow you to use the RealSense depth camera to obtain the goal position from the wheels detected previously
 */
class RealSenseManager
{
public:
    /**
     * @brief Get the Distance And Coords object
     * 
     * @param selection rs2 pipeline profile
     * @param color_frame  rs2 color frame
     * @param depth_frame  rs2 depth frame
     * @param objects objects detected previously (for instance, the wheels of the trolley). Its a vector of pair<class type, coordinates>.
     * @return std::vector<std::pair<float, cv::Point2f>>  retuns a vector of pair<distance to the wheel, coordinates>.
     */
	std::vector<std::pair<float, cv::Point2f>> 
    getDistanceAndCoords(rs2::pipeline_profile selection,
                        rs2::video_frame& color_frame, 
                        rs2::depth_frame& depth_frame, 
                        const std::vector<std::pair<int,cv::Point2f>>& objects);

    unsigned int getNumberOfPeople(const std::vector<std::pair<int,cv::Point2f>>& objects);

     /**
      * @brief Obtains the position of the imaginary mark from which the robot must be placed.
      * 
      * @param tag Tag estimated info (distance and coordinates). It will be located between the nearest two wheels.
      * @param data Info about the wheels (distance and coordinates)
      * @return true Whether it was possible to create the tag
      * @return false It was not possible to create the tag (for instance, if there system did not detect at least two wheels)
      */
    bool getTagPose(std::pair<float, cv::Point2f>& tag, 
                    std::vector<std::pair<float, cv::Point2f>>& data);

    /**
     * @brief It creates a perfect rectangle that contains all the detected wheels.
     * 
     * @param data Wheels info (distance and coordinates)
     */
    void boundedRectangle(std::vector<std::pair<float, cv::Point2f>>& data);

    /**
     * @brief Get the Goal From the current position
     * 
     * @param poseMarca Current tag located between two wheels 
     * @param angulo_local Angle respect to the tag
     * @param x_local X value respect to the tag
     * @param z_local  Z value respect to the tag
     * @return tag_pose_t Target pose respect to the current position
     */
    tag_pose_t getGoalFromPose(tag_pose_t poseMarca, double angulo_local, double x_local, double z_local);

    /**
     * @brief Create a Tag Matrix object
     * 
     * @param tag Current tag
     * @return tag_pose_t Matrix usefull for transformations
     */
    tag_pose_t createTagMatrix(const std::pair<float, cv::Point2f>& tag);

    /**
     * @brief It is used to obtain the current goal using the laser info
     * 
     * @param data Laser data info
     * @return tag_pose_t Target position
     */
    tag_pose_t reportLaserData(const LaserDataT &data); 

     /**
     * @brief Rectangle obtained from the wheels detected
     * 
     * @param info Wheels info
     * @return TrolleyRectangle Rectangle 
     */
    TrolleyRectangle rectangleEstimation(std::vector<std::pair<float, cv::Point2f>> & info);

private:   
    /**
     * @brief Laser info
     * 
     */
    LaserDataT laser_data;
    
    // Not needed
    const float width = 1280;
    const float height = 720;
    const float fov_h = 87;
    const float fov_v = 58;
    //--

    /**
     * @brief Distance between two points
     * 
     * @param p1 Point A
     * @param p2 Point B
     * @return float  Distance
     */
    float distance(cv::Point2f& p1, cv::Point2f& p2);

    /**
     * @brief Angle between two points (line)
     * 
     * @param p1 Point A
     * @param p2 Poit B
     * @return float Angle
     */
    float angle(cv::Point2f& p1, cv::Point2f& p2);

    /**
     * @brief Angle between two points(line) in radians
     * 
     * @param p1 Point A
     * @param p2 Point B
     * @return float Angle in radians
     */
    float angleRadians(cv::Point2f& p1, cv::Point2f& p2);

    /**
     * @brief It generates the curent goal from the laser data
     * 
     * @return tag_pose_t Target position
     */
    tag_pose_t processLaserData();
};

}

#endif
