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

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <thread>
//#include "apriltagmanager.h"
#include <samples/ocv_common.hpp>
#include <samples/slog.hpp>
#include <librealsense2/rsutil.h> 
#include <librealsense2/rs.hpp> 
#include "openvinomanager.h"
#include "realsensemanager.h"


using namespace sa3ir;
using namespace std;
using namespace cv;
using namespace InferenceEngine;

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	void compute();

private:   
    
    LaserDataT laser_data;
    tag_pose_t pose_from_laser;
    
   // AprilTagManager apriltagm;
    OpenVinoManager openvinom;

    RealSenseManager realsensem;
              
    Mat image;
    Mat imgHSV;
    Mat OutputImage;
    rs2::pipeline_profile selection;
    
    const float width = 1280;
    const float height = 720;
    const float fov_h = 87;
    const float fov_v = 58;

    unsigned int npeople;
    
   
    std::vector<std::pair<float, cv::Point2f>> obtenerCoordenadasYDistancia(rs2::video_frame& color_frame, 
                                                                            rs2::depth_frame& depth_frame, 
                                                                            const std::vector<std::pair<int,cv::Point2f>>& objects);
    
    bool posicionMarca(std::pair<float, cv::Point2f>& tag, std::vector<std::pair<float, cv::Point2f>>& data);
    std::pair<float, cv::Point2f> posicionMarca(cv::Point2f& p1, cv::Point2f& p2);
    vector<std::pair<float, cv::Point2f>> posicionMarcaDesdeLateral(std::vector<std::pair<float, cv::Point2f>>& data);
    
    tag_pose_t calculaObjetivoSA3IR(tag_pose_t poseMarca, double angulo_local, double x_local, double z_local);
    tag_pose_t construirMatrizMarca(const std::pair<float, cv::Point2f>& tag);
    
    float distanciaPuntos(cv::Point2f& p1, cv::Point2f& p2);
    float angulo(cv::Point2f& p1, cv::Point2f& p2);
    float anguloRadianes(cv::Point2f& p1, cv::Point2f& p2);
    //std::vector<std::pair<double, double>> estimarRectangulo(std::vector<std::pair<float, cv::Point2f>> & info);
    RoboCompTagBasedLocalization::TrolleyRectangle estimarRectangulo(std::vector<std::pair<float, cv::Point2f>> & info);
    float pendienteRecta(cv::Point2f& p1, cv::Point2f& p2);
    void alphaFromCamera(const std::vector<std::pair<int,cv::Point2f>>& objects);
    void printObjectsDistance(rs2::video_frame& color_frame, 
                              rs2::depth_frame& depth_frame, 
                              const std::vector<std::pair<int,cv::Point2f>>& objects);
    
    void sendTagToAgent(double x, double z, double angle);
    void sendTrolleyEstimationToAgent(double x, double z, double angle, const RoboCompTagBasedLocalization::TrolleyRectangle& rectangle);
    void sendPickUpPoseEstimationToAgent(double x, double z, double angle);
    void sendDeliverFinishedPoseEstimationToAgent(double x, double z, double angle);
    void pickUpPoseInfo(const double& x, const double& z, const double& alpha);
    void boundedRectangle(std::vector<std::pair<float, cv::Point2f>>& data);
    
    int test();
    
    void processLaserData();
    
    //Laser interface 
    void reportLaserData(const LaserDataT &data);
    
};

#endif
