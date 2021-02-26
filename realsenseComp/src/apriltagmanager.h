/*
 *    Copyright (C)2021 by the University of Málaga
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

#ifndef APRILTAGMANAGER_H
#define APRILTAGMANAGER_H

#include <iostream>

//#include <monitors/presenter.h>
#include <samples/ocv_common.hpp>
#include <samples/slog.hpp>


#include <librealsense2/rs.hpp> 
extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
#include "tag36h11.h"
#include "common/getopt.h"
}


using namespace InferenceEngine;


//------------APRIL and RealSense -----------


using namespace std;
using namespace cv;

class AprilTagPose {
public:
    AprilTagPose() {
        valid = false;
    }
    AprilTagPose(float x, float z, float angle) {
        this->x = x;
        this->z = z;
        this->angle = angle;
        valid = true;
    }
    void setPose(float x, float z, float angle) {
        this->x = x;
        this->z = z;
        this->angle = angle;
        valid = true;
    }
    bool isValid() {
        return valid;
    }
     
    float getX() {
        return x;
    }
    float getZ() {
        return z;
    }
    float getAngle() {
        return angle;
    }
     
private:
     float x, z, angle;   
     bool valid;
};


class AprilTagManager
{
public:
    AprilTagManager();
    ~AprilTagManager();
    void estimateTagPose(apriltag_detection_t *det, rs2::pipeline_profile& pprof, AprilTagPose& tagPose, unsigned int tagID, float& angle);
    AprilTagPose detectTag(rs2::video_frame vf, cv::Mat& frame, rs2::pipeline_profile& pprof);
    apriltag_pose_t calculaObjetivoSA3IR(apriltag_pose_t poseMarca, double angulo_local, double X_local, double Z_local);
    
private:
    apriltag_family_t *tf;
    apriltag_detector_t *td;
    //const float TAG_SIZE = 0.157; //grande
    const float TAG_SIZE = 0.087; //mediana
    //const float TAG_SIZE = 0.053;//pequeña
    
    void printAprilTagPoseT(apriltag_pose_t& pose);
    void robotPoseFromTagID(unsigned int tagID, float& x, float& z, float& angle);
};


#endif // APRILTAGMANAGER_H
