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


#include "apriltagmanager.h"


AprilTagManager::AprilTagManager()
{
    tf = tag36h11_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    
    td->quad_decimate = 1.0;
    td->quad_sigma    = 0.0;
    td->nthreads      = 1;
    td->debug         = 0;
    td->refine_edges  = 1;
}

AprilTagManager::~AprilTagManager()
{
	apriltag_detector_destroy(td);
	tag36h11_destroy(tf);
}

void AprilTagManager::estimateTagPose(apriltag_detection_t *det, rs2::pipeline_profile& pprof, AprilTagPose& tagPose, unsigned int tagID, float& prevAngle)
{
	// First create an apriltag_detection_info_t struct using your known parameters.
	apriltag_detection_info_t info;
	info.det = det;
	info.tagsize = TAG_SIZE;
	info.fx = pprof.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics().fx;
	info.fy = pprof.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics().fy;
	info.cx = pprof.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics().ppx;
	info.cy = pprof.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics().ppy;

	// Then call estimate_tag_pose.
	apriltag_pose_t pose;
	/*double err =*/ estimate_tag_pose(&info, &pose);
    
    float current_angle = acos(pose.R->data[0]);
    std::cout << prevAngle << " " << current_angle  << std::endl;
    if(current_angle < prevAngle) 
    {
        prevAngle = current_angle;
        // Calculate robot goal from AprilTag ID
        float angle, x, z;
        robotPoseFromTagID(tagID, x, z, angle);

        // Estimate robot goal
        apriltag_pose_t pose_res = calculaObjetivoSA3IR(pose, angle, x, z);

       // printAprilTagPoseT(pose);
       // printAprilTagPoseT(pose_res);

        std::cout << "Resultado = ( x = " <<  pose_res.t->data[0] << ", z = " << pose_res.t->data[2] << ", angle = " << atan2 ( pose_res.R->data[2], pose_res.R->data[0]) << " )"  << endl;

        tagPose.setPose(pose_res.t->data[0], pose_res.t->data[2], atan2 ( pose_res.R->data[2], pose_res.R->data[0]));
    }
}

void AprilTagManager::robotPoseFromTagID(unsigned int tagID, float& x, float& z, float& angle)
{
    switch (tagID) {
        case 0 : x = 0; z = -0.9; angle = 0; 
                break;
        case 1 : x = 1.30; z = 0.33; angle = -1.57; 
                break;
        case 2 : x = 0; z = 1.70; angle = 3.14; 
                break;
        case 3 : x = -1.30; z = 0.33; angle = 1.57; 
                break;
        default: break;        
    }
}

apriltag_pose_t AprilTagManager::calculaObjetivoSA3IR(apriltag_pose_t poseMarca, double angulo_local, double X_local, double Z_local)
{
	apriltag_pose_t result;

	matd_t *result_rotation = matd_create(3,3);
	matd_t* result_translation = matd_create(3,1);
	result.R = result_rotation;
	result.t = result_translation;

	// Cálculo de las componentes de la matriz final
	// T(final) = T(poseMarca) X T(local)
	// Donde Tlocal:
	//	c(phi)	0		s(phi)	Tx
	//	0		1		0		0
	//	-s(phi)	0		c(phi)	Tz
	//	0		0		0		1

	result.R->data[0] = poseMarca.R->data[0]*cos(angulo_local) - poseMarca.R->data[2]*sin(angulo_local);
	result.R->data[1] = poseMarca.R->data[1];
	result.R->data[2] = poseMarca.R->data[0]*sin(angulo_local) + poseMarca.R->data[2]*cos(angulo_local);

	result.R->data[3] = poseMarca.R->data[3]*cos(angulo_local) - poseMarca.R->data[5]*sin(angulo_local);
	result.R->data[4] = poseMarca.R->data[4];
	result.R->data[5] = poseMarca.R->data[3]*sin(angulo_local) + poseMarca.R->data[5]*cos(angulo_local);

	result.R->data[6] = poseMarca.R->data[6]*cos(angulo_local) - poseMarca.R->data[8]*sin(angulo_local);
	result.R->data[7] = poseMarca.R->data[7];
	result.R->data[8] = poseMarca.R->data[6]*sin(angulo_local) + poseMarca.R->data[8]*cos(angulo_local);

	result.t->data[0] = poseMarca.R->data[0]*X_local + poseMarca.R->data[2]*Z_local + poseMarca.t->data[0];
	result.t->data[1] = poseMarca.R->data[3]*X_local + poseMarca.R->data[5]*Z_local + poseMarca.t->data[1];
	result.t->data[2] = poseMarca.R->data[6]*X_local + poseMarca.R->data[8]*Z_local + poseMarca.t->data[2];

	return result;
}


AprilTagPose AprilTagManager::detectTag(rs2::video_frame vf, cv::Mat& frame, rs2::pipeline_profile& pprof)
{
    AprilTagPose tagPose;
    //bool tagDetected = false;
    
	cv::Mat gray;
	cvtColor(frame, gray, COLOR_BGR2GRAY);
	
	//imshow("Imagen", gray);
	//char c = (char)waitKey(25);
	
	image_u8_t im = { 	.width = gray.cols,
                        .height = gray.rows,
                        .stride = gray.cols,
                        .buf = gray.data };

	zarray_t *detections = apriltag_detector_detect(td, &im);	
    
    //TODO: Seleccionar el mejor 
    int nDetections = zarray_size(detections);
    if(nDetections > 0) {
      //  tagDetected = true;
        float angle = 3.14;
        for (int i = 0; i < nDetections; i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            stringstream ss;
            ss << det->id;
            std::cout << "Frame: "<< vf.get_frame_number() <<": tag detectado con ID = " << ss.str() << std::endl;
            estimateTagPose(det, pprof, tagPose, det->id, angle);
        }
    }
    
    //std::cout << tagPose.getX() << " " << tagPose.getZ() << " " << tagPose.getAngle() << std::endl;
    
    apriltag_detections_destroy(detections);
    
    return tagPose;
}


void AprilTagManager::printAprilTagPoseT(apriltag_pose_t& pose)
{
    std::cout << "Vector R:" << std::endl;

	for (unsigned int i = 0; i < (pose.R->nrows)*(pose.R->ncols); i++)
	{
		std::cout << pose.R->data[i] << " ";
		std::cout.flush();
		static int j = 0;
		j++;			
		if(j == 3) {
			std::cout << std::endl;
			j = 0;
		}
	}

	std::cout << std::endl;

	std::cout << "Vector t: " << std::endl;

	for (unsigned int i = 0; i < (pose.t->nrows)*(pose.t->ncols); i++)
	{
		std::cout << pose.t->data[i] << " ";
		std::cout.flush();
	}

	std::cout << std::endl;
}

