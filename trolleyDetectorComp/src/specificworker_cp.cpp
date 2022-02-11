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

#include "specificworker.h"
#include <iomanip>

using namespace std;
using namespace cv;

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
    timer.setSingleShot(true); //compute method is executed only once
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::compute()
{
    
   // test();
    
    try {
        
        rs2::pipeline pipe;
        
        //Create a configuration for configuring the pipeline with a non default profile
        rs2::config cfg;    

    
        //Add desired streams to configuration
        cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);
        
        //Instruct pipeline to start streaming with the requested configuration
        selection = pipe.start(cfg); //for color
        
         // Camera warmup - dropping several first frames to let auto-exposure stabilize
        rs2::frameset frames;

        for (int i = 0; i < 30; i++)
        {
            //Wait for all configured streams to produce a frame
            frames = pipe.wait_for_frames();
        }

        openvinom.loadAndCreateInfer();
        
        while (waitKey(1) < 0) 
        { 
            frames = pipe.wait_for_frames();    // Wait for next set of frames from the camera
            auto color_frame = frames.get_color_frame();
            auto depth_frame = frames.get_depth_frame();

            if (!depth_frame || !color_frame)
                continue;
            
            cv::Mat color(Size(color_frame.get_width(),color_frame.get_height()),CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
            if (color.empty())
            {
                cerr << "image was not generated !" << endl;
                continue;
            }

            std::vector<std::pair<int, cv::Point2f>> objects = openvinom.detectObjects(color);
            std::vector<std::pair<float, cv::Point2f>> coords_info = obtenerCoordenadasYDistancia(color_frame, depth_frame, objects);
        
           
            if(coords_info.size() >= 3)
            {
                if(coords_info.size() == 4) //si ve menos ruedas el algoritmo puede no funcionar bien. TODO
                {
                    boundedRectangle(coords_info);
                }
                std::pair<float, cv::Point2f> tag_pose;
                if(posicionMarca(tag_pose, coords_info))
                {
                    double x, z, angle;
                    tag_pose_t tag_matrix = construirMatrizMarca(tag_pose);
                    
                    tag_pose_t pose_fr = calculaObjetivoSA3IR(tag_matrix, 3.14, 0, 1.0);
                    x = pose_fr.t[0];
                    z = pose_fr.t[2];
                    angle = atan2 (pose_fr.r[2], pose_fr.r[0]);
                    //std::cout << "Tag = ( x = " <<  x << ", z = " << z << ", angle = " << angle << " )"  << endl;
                    //std::cout << "Goal = ( x = " <<  pose_fr.t[0] << ", z = " << pose_fr.t[2] << ", angle = " << atan2 ( pose_fr.r[2], pose_fr.r[0]) << " )"  << endl;
                    sendTrolleyEstimationToAgent(x, z, angle, estimarRectangulo(coords_info));
                    
                    pose_fr = calculaObjetivoSA3IR(tag_matrix, 0, 0, -0.35);
                    x = pose_fr.t[0];
                    z = pose_fr.t[2];
                    angle = atan2 (pose_fr.r[2], pose_fr.r[0]);
                    //std::cout << "Goal pickup = ( x = " <<  pose_fr.t[0] << ", z = " << pose_fr.t[2] << ", angle = " << atan2 ( pose_fr.r[2], pose_fr.r[0]) << " )"  << endl;
                    sendPickUpPoseEstimationToAgent(x,z,angle);
                    
                    pose_fr = calculaObjetivoSA3IR(tag_matrix, 0, 0, 1);
                    x = pose_fr.t[0];
                    z = pose_fr.t[2];
                    angle = atan2 (pose_fr.r[2], pose_fr.r[0]);
                    //std::cout << "Goal deliver finished = ( x = " <<  pose_fr.t[0] << ", z = " << pose_fr.t[2] << ", angle = " << atan2 ( pose_fr.r[2], pose_fr.r[0]) << " )"  << endl;
                    sendDeliverFinishedPoseEstimationToAgent(x,z,angle);
                }
            }
            namedWindow("Display Image", WINDOW_AUTOSIZE);
            imshow("Display Image", color);    
        }        
    }
    catch (const std::exception& error) {
        std::cerr << "[ ERROR ] " << error.what() << std::endl;
    }
    catch (...) {
        std::cerr << "[ ERROR ] Unknown/internal exception happened." << std::endl;
    }
    std::cout << "Compute finished!" << std::endl;
}



void SpecificWorker::sendTrolleyEstimationToAgent(double x, double z, double angle, const RoboCompTagBasedLocalization::TrolleyRectangle& rectangle)
{
    static unsigned int cont = 0;
    //if (!(cont++%2)) 
    {
        try {
            //std::cout << "sendTrolleyEstimationToAgent: " << rectangle.size() << std::endl;
            tagbasedlocalization_proxy->trolleyPoseInfo(x, z, angle, rectangle);
        }catch(...){ 
            //std::cout << "Proxy error: ObjectDetectionAgent not available" << std::endl; 
        }
    }
}
void SpecificWorker::sendPickUpPoseEstimationToAgent(double x, double z, double angle)
{
    static unsigned int cont = 0;
   // if (!(cont++%2)) 
    {
        try {
            tagbasedlocalization_proxy->pickUpPoseInfo(x, z, angle);
        }catch(...){ 
           // std::cout << "Proxy error: ObjectDetectionAgent not available" << std::endl; 
        }
    }
}

void SpecificWorker::sendDeliverFinishedPoseEstimationToAgent(double x, double z, double angle)
{
    static unsigned int cont = 0;
   // if (!(cont++%2)) 
    {
        try {
            tagbasedlocalization_proxy->deliverFinishedPoseInfo(x, z, angle);
        }catch(...){ 
          //  std::cout << "Proxy error: ObjectDetectionAgent not available" << std::endl; 
        }
    }
}


void SpecificWorker::processLaserData()
{
    std::sort(laser_data.begin(), laser_data.end(), [] (PointT& p1,PointT& p2) { return p1.range < p2.range; });
    
   /* cout << "************************************" << endl;
	for(unsigned int i = 0; i < laser_data.size(); ++i)
	{ 
		cout << laser_data[i].x << ", " << laser_data[i].y << ", " << laser_data[i].angle << ", " << laser_data[i].range << endl; 
	}
	cout << "************************************" << endl;
    */
    //std::pair<float, cv::Point2f> point_a, point_b;
    
    
    PointT point_a, point_b;
    
    if(laser_data.size())
    {
        point_a = laser_data[0];
    }

    bool point_b_initialized = false;

    for(unsigned int i = 1; i < laser_data.size(); i++)
    {
        if(fabs(point_a.angle - laser_data[i].angle) <=  0.15 && fabs(point_a.range - laser_data[i].range) <= 0.15)
        {
            //me quedo con el más cercano
            if(point_a.range > laser_data[i].range)
            {
                point_a = laser_data[i];
            }
        }
        else if (!point_b_initialized)
        {
            point_b_initialized = true;
            point_b = laser_data[i];
        }
        else if(fabs(point_b.angle - laser_data[i].angle) <=  0.15 && fabs(point_b.range - laser_data[i].range) <= 0.15)
        {
            //me quedo con el más cercano
            if(point_b.range > laser_data[i].range)
            {
                point_b = laser_data[i];
            }
        }
    }
    float x, y, angle;
    if(point_a.y < point_b.y)    
    {
        y = (point_a.x + point_b.x)/2;
        x = (point_a.y + point_b.y)/2;
        //angle = (atan2f(point_a.y - point_b.y, point_a.x - point_b.x) * 180)/3.14159265;
        angle = -atan2f(point_a.x - point_b.x, point_a.y - point_b.y);
    }
    else
    {        
        y = (point_b.x + point_a.x)/2;
        x = (point_b.y + point_a.y)/2;
        //angle = (atan2f(point_b.y - point_a.y, point_b.x - point_a.x) * 180)/3.14159265;
        angle = -atan2f(point_b.x - point_a.x, point_b.y - point_a.y);
    }

    std::cout << "point_a: ( " << point_a.x << ", " << point_a.y << ", " << point_a.angle << ")" << std::endl;
    std::cout << "point_b: ( " << point_b.x << ", " << point_b.y << ", " << point_b.angle << ")" << std::endl;
    std::cout << "( " << x << ", " << y << ", " << angle << ")" << std::endl;

    
    std::pair<float, cv::Point2f> tag = std::make_pair(angle, cv::Point2f(x, y));
    tag_pose_t tag_matrix = construirMatrizMarca(tag);
    
   
    tag_pose_t pose_from_laser = calculaObjetivoSA3IR(tag_matrix, 0.0, 0, 0.65);

    float goal_x = pose_from_laser.t[0];
    float goal_y = pose_from_laser.t[2];
    float goal_angle = atan2 (pose_from_laser.r[2], pose_from_laser.r[0]);
    std::cout << "Tag = ( x = " <<  goal_x << ", y = " << goal_y << ", angle = " << goal_angle << " )"  << endl;
    // std::cout << "Goal = ( x = " <<  pose_from_laser.t[0] << ", z = " << pose_from_laser.t[2] << ", angle = " << atan2 ( pose_from_laser.r[2], pose_from_laser.r[0]) << " )"  << endl;
    
    
    //TODO: OJO a los interbloqueos!!!
   // sendPickUpPoseEstimationToAgent(goal_x,goal_x,goal_angle);
    try {
        tagbasedlocalization_proxy->pickUpPoseInfo(goal_x, goal_y, goal_angle);
    }catch(...){ 
        // std::cout << "Proxy error: ObjectDetectionAgent not available" << std::endl; 
    }

}

void SpecificWorker::reportLaserData(const LaserDataT &data)
{    
    laser_data.clear(); //TODO: check if it is needed
    laser_data.reserve(data.size());
    std::copy(data.begin(), data.end(), back_inserter(laser_data)); 
    processLaserData();    
}

tag_pose_t SpecificWorker::calculaObjetivoSA3IR(tag_pose_t poseMarca, double angulo_local, double x_local, double z_local)
{
    tag_pose_t result;
	// Cálculo de las componentes de la matriz final
	// T(final) = T(poseMarca) X T(local)
	// Donde Tlocal:
	//	c(phi)	0		s(phi)	Tx
	//	0		1		0		0
	//	-s(phi)	0		c(phi)	Tz
	//	0		0		0		1


	result.r[0] = poseMarca.r[0]*cos(angulo_local) - poseMarca.r[2]*sin(angulo_local);
	result.r[1] = poseMarca.r[1];
	result.r[2] = poseMarca.r[0]*sin(angulo_local) + poseMarca.r[2]*cos(angulo_local);

	result.r[3] = poseMarca.r[3]*cos(angulo_local) - poseMarca.r[5]*sin(angulo_local);
	result.r[4] = poseMarca.r[4];
	result.r[5] = poseMarca.r[3]*sin(angulo_local) + poseMarca.r[5]*cos(angulo_local);

	result.r[6] = poseMarca.r[6]*cos(angulo_local) - poseMarca.r[8]*sin(angulo_local);
	result.r[7] = poseMarca.r[7];
	result.r[8] = poseMarca.r[6]*sin(angulo_local) + poseMarca.r[8]*cos(angulo_local);

	result.t[0] = poseMarca.r[0]*x_local + poseMarca.r[2]*z_local + poseMarca.t[0];
	result.t[1] = poseMarca.r[3]*x_local + poseMarca.r[5]*z_local + poseMarca.t[1];
	result.t[2] = poseMarca.r[6]*x_local + poseMarca.r[8]*z_local + poseMarca.t[2];

	return result;
}

tag_pose_t SpecificWorker::construirMatrizMarca(const std::pair<float, cv::Point2f>& tag)
{
    tag_pose_t tag_m;
    
    float angle = (tag.first * 3.14159265)/180;
    
    tag_m.r[0] = cos(angle);
	tag_m.r[1] = 0;
	tag_m.r[2] = sin(angle);
	tag_m.r[3] = 0;
	tag_m.r[4] = 1;
	tag_m.r[5] = 0;
	tag_m.r[6] = -sin(angle);
	tag_m.r[7] = 0;
	tag_m.r[8] = cos(angle);
    
    tag_m.t[0] = tag.second.x;
    tag_m.t[1] = 0;
    tag_m.t[2] = tag.second.y;
    
    return tag_m;
}

void SpecificWorker::boundedRectangle(std::vector<std::pair<float, cv::Point2f>>& data)
{
      //Estimando rectangulo
    
    std::vector<cv::Point2f> points;
    
    for(unsigned int i = 0; i < data.size(); i++)
    {
        points.push_back(data[i].second);
    }
    
    // compute the rotated bounding rect of the points
    cv::RotatedRect boundingBox = cv::minAreaRect(points);
    // one thing to remark: this will compute the OUTER boundary box, so maybe you have to erode/dilate if you want something between the ragged lines
    
    cv::Mat drawing(1200, 1400, CV_8UC3, Scalar(10, 100, 150));
    
    
    // draw the rotated rect
    cv::Point2f corners[4];
    boundingBox.points(corners);
    //std::cout << "rectángulo: " << corners[0] << " " << corners[1] << " " << corners[2] << " " << corners[3] << std::endl;
    
    cv::Point2f origen;
    origen.x = 0;
    origen.y = 0;
    data.clear();
    for(unsigned int i = 0; i < 4; i++)
    {
        data.push_back(std::make_pair(distanciaPuntos(corners[i], origen), corners[i]));
    }
    std::sort(data.begin(), data.end(), [] (std::pair<float, cv::Point2f> &c1,std::pair<float, cv::Point2f> &c2) { return c1.first < c2.first; });
}

/**
 * Obtiene la posición de la marca imaginaria a partir de la cual el robot debe colocarse.
 * Si no hay dos ruedas devuelve falso. True en caso contrario
 */
bool SpecificWorker::posicionMarca(std::pair<float, cv::Point2f>& tag, std::vector<std::pair<float, cv::Point2f>>& data)
{    
    bool found = false;
    int p1, p2;
    if(data.size() >= 2) 
    {
        if(data[0].second.x <= data[1].second.x) 
        {
            p1 = 0; p2 = 1;  //std::cout << "X a la izquierda" << std::endl;
        }
        else 
        {
            p1 = 1; p2 = 0;  //std::cout << "X a la derecha" << std::endl;
        }
        found = true;
    }
    /*if (data.size() > 2) //hay más de dos ruedas (dos o más rectas)
    {
        double angulo_recta1 = angulo(data[0].second, data[1].second);
        double angulo_recta3a = angulo(data[0].second, data[2].second);
        if  (((angulo_recta1 > angulo_recta3a) && ((180 - angulo_recta1) >= angulo_recta3a)) || 
            ((angulo_recta3a > angulo_recta1) && ((180 - angulo_recta3a) < angulo_recta1)))
        {
            p2 = 2;
        }
        found = true;
    }*/
    if(found)
    {
        tag.second.x = (data[p1].second.x+data[p2].second.x)/2;
        tag.second.y = (data[p1].second.y+data[p2].second.y)/2;
        tag.first = angulo(data[p1].second, data[p2].second);
    }
    
  
    
    
    return found;
}


std::pair<float, cv::Point2f> SpecificWorker::posicionMarca(cv::Point2f& point1, cv::Point2f& point2)
{
    
    cv::Point2f p_res;
    float angle_res;
    
   // std::cout << "x: " << point1.x << " " << point2.x << " y: " << point1.y << " " << point2.y <<  " angulo:  "  << angulo(point1, point2) << " " << angulo(point2, point1) << endl;
    
    if(point1.x <= point2.x) 
    {
        p_res.x = (point1.x+point2.x)/2;
        p_res.y = (point1.y+point2.y)/2;
        angle_res = angulo(point1, point2);
    }
    else 
    {
        p_res.x = (point2.x+point1.x)/2;
        p_res.y = (point2.y+point1.y)/2;
        angle_res = angulo(point2, point1);
    }
    
    return std::make_pair(angle_res, p_res);
}



/*
 * Calcula el ángulo de la recta, dados dos puntos.
 */
float SpecificWorker::angulo(cv::Point2f& p1, cv::Point2f& p2)
{
    float radians = -atan2f(p1.y - p2.y, p1.x - p2.x); //eje "y" hacia abajo dextrógiro
    return (radians * 180)/3.14159265;
}


float SpecificWorker::anguloRadianes(cv::Point2f& p1, cv::Point2f& p2)
{
    return atan2f(p1.y - p2.y, p1.x - p2.x);
}


/**
 * Calcula las rectas entre las ruedas para formar un rectángulo
 * */

RoboCompTagBasedLocalization::TrolleyRectangle SpecificWorker::estimarRectangulo(std::vector<std::pair<float, cv::Point2f>> & info)
{
    std::cout << std::endl;
    RoboCompTagBasedLocalization::TrolleyRectangle lines_v;
    double alpha1, length1;
    double alpha2, length2;
    double alpha3a, length3a;
    double alpha3b, length3b;
    double alpha4a, length4a;
    double alpha4b, length4b;
    
    if(info.size() == 4) //hay cuatro ruedas. En otros casos mirar cuáles ha visto.
    {

        for(unsigned int i = 0; i < info.size(); ++i)
        {
           // std::cout << "Rueda " << i << " (" << info[i].second.x << ", " << info[i].second.y << "), distancia: " << info[i].first << std::endl;
        }

        length1 = distanciaPuntos(info[0].second, info[1].second);  
        length2 = distanciaPuntos(info[2].second, info[3].second);
        length3a = distanciaPuntos(info[0].second, info[2].second);
        length3b = distanciaPuntos(info[1].second, info[2].second);
        length4a = distanciaPuntos(info[0].second, info[3].second);
        length4b = distanciaPuntos(info[1].second, info[3].second);
        
        if(info[0].second.x <= info[1].second.x)
            alpha1 = angulo(info[0].second, info[1].second);
        else
            alpha1 = angulo(info[1].second, info[0].second);
        
        if(info[2].second.x <= info[3].second.x)
            alpha2 = angulo(info[2].second, info[3].second);
        else
            alpha2 = angulo(info[3].second, info[2].second);
        
        if(info[0].second.x <= info[2].second.x)
            alpha3a = angulo(info[0].second, info[2].second);
        else
            alpha3a = angulo(info[2].second, info[0].second);
        
        if(info[1].second.x <= info[2].second.x)
            alpha3b = angulo(info[1].second, info[2].second);
        else
            alpha3b = angulo(info[2].second, info[1].second);
        
        if(info[0].second.x <= info[3].second.x)
            alpha4a = angulo(info[0].second, info[3].second);
        else
            alpha4a = angulo(info[3].second, info[0].second);
        
        if(info[1].second.x <= info[3].second.x)
            alpha4b = angulo(info[1].second, info[3].second);
        else
            alpha4b = angulo(info[3].second, info[1].second);
        
       // std::cout << "Ángulo de la recta 1: " << alpha1  << ", longitud: " << length1 << std::endl;
       // std::cout << "Ángulo de la recta 2: " << alpha2   << ", longitud: " << length2 << std::endl;
        
        lines_v.push_back({"line1", length1, alpha1});
        lines_v.push_back({"line2", length2, alpha2});
        
        
        if(distanciaPuntos(info[0].second, info[2].second) < distanciaPuntos(info[1].second, info[2].second))
        {
         //   std::cout << "Ángulo de la recta 3a: " << alpha3a  << ", longitud: " << length3a << std::endl;
            lines_v.push_back({"line3a", length3a, alpha3a});
        }
        else
        {
           // std::cout << "Ángulo de la recta 3b: " << alpha3b  << ", longitud: " << length3b << std::endl;
            lines_v.push_back({"line3b", length3b, alpha3b});
        }
        
        if(distanciaPuntos(info[0].second, info[3].second) < distanciaPuntos(info[1].second, info[3].second))
        {
            //std::cout << "Ángulo de la recta 4a: " << alpha4a << ", longitud: " << length4a << std::endl;            
            lines_v.push_back({"line4a", length4a, alpha4a});
        }
        else
        {
            //std::cout << "Ángulo de la recta 4b: " << alpha4b  << ", longitud: " << length4b << std::endl;
            lines_v.push_back({"line4b", length4b, alpha4b});
        }
        
    }
    else if(info.size()==3) //ahora mismo me fijo en las dos más cercanas
    {
        alpha1 = angulo(info[0].second, info[1].second);
        
        
        if(info[0].second.x <= info[1].second.x)
            alpha1 = angulo(info[0].second, info[1].second);
        else
            alpha1 = angulo(info[1].second, info[0].second);
        
        if(info[0].second.x <= info[2].second.x)
            alpha3a = angulo(info[0].second, info[2].second);
        else
            alpha3a = angulo(info[2].second, info[0].second);
        
         if(info[1].second.x <= info[2].second.x)
            alpha3b = angulo(info[1].second, info[2].second);
        else
            alpha3b = angulo(info[2].second, info[1].second);
        
        length1 = distanciaPuntos(info[0].second, info[1].second);  
        length3a = distanciaPuntos(info[0].second, info[2].second);
        length3b = distanciaPuntos(info[1].second, info[2].second);
    
        //std::cout << "Ángulo de la recta 1: " << alpha1 << ", longitud: " << length1 << std::endl;
        lines_v.push_back({"line1", length1, alpha1});
        
        if(distanciaPuntos(info[0].second, info[2].second) < distanciaPuntos(info[1].second, info[2].second))
        {
          //  std::cout << "Ángulo de la recta 3a: " << alpha3a  << ", longitud: " << length3a << std::endl;
            lines_v.push_back({"line3a", length3a, alpha3a});
        }
        else
        {
            //std::cout << "Ángulo de la recta 3b: " << alpha3b  << ", longitud: " << length3b << std::endl;
            lines_v.push_back({"line3b", length3b, alpha3b});
        }
    }
    else if(info.size() >= 1)
    {
         if(info[0].second.x <= info[1].second.x)
            alpha1 = angulo(info[0].second, info[1].second);
        else
            alpha1 = angulo(info[1].second, info[0].second);
        
        length1 = distanciaPuntos(info[0].second, info[1].second);
        //std::cout << "Ángulo de la recta 1: " << alpha1 << ", longitud: " << length1 << std::endl;
        lines_v.push_back({"line1", length1, alpha1});
    }
    
    return lines_v;
}

/**
 * Calcula la distancia entre dos puntos
 */
float SpecificWorker::distanciaPuntos(cv::Point2f& p1, cv::Point2f& p2)
{
     return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

/*
 * Dadas los pixels de cada rueda, se calcula el ángulo y la coordenada desde la cámara.
 */
std::vector<std::pair<float, cv::Point2f>> 
SpecificWorker::obtenerCoordenadasYDistancia(rs2::video_frame& color_frame, 
                                             rs2::depth_frame& depth_frame, 
                                             const std::vector<std::pair<int,cv::Point2f>>& objects)
{
    std::vector<std::pair<float, cv::Point2f>> result;
    
    auto depth_profile = depth_frame.get_profile().as<rs2::video_stream_profile>();
    auto color_profile = color_frame.get_profile().as<rs2::video_stream_profile>();

    auto depth_intrin = depth_profile.get_intrinsics();
    auto color_intrin = color_profile.get_intrinsics();
    auto depth2color_extrin = depth_profile.get_extrinsics_to(color_profile);
    auto color2depth_extrin = color_profile.get_extrinsics_to(depth_profile);
  
    auto sensor = selection.get_device().first<rs2::depth_sensor>();
    auto scale = sensor.get_depth_scale();

    cv::Point2f center(width/2, height/2);

    int i = 0;
    for(auto& object : objects) 
    {
        if(object.first)
        {    
           // std::cout << std::endl << i++ << " -> rueda detectada: " << std::endl;
        
            float rgb_src_pixel[2] = { object.second.x,object.second.y }; // The RGB coordinate for the center of the marble
            float dpt_tgt_pixel[2] = { 0 }; // The depth pixel that has the best match for that RGB coordinate

            // Search along a projected beam from 0.1m to 10 meter. This can be optimized to the concrete scenario, e.g. if you know that the data is bounded within [min..max] range
            rs2_project_color_pixel_to_depth_pixel(dpt_tgt_pixel, reinterpret_cast<const uint16_t*>(depth_frame.get_data()), scale, 0.1f, 2,
                &depth_intrin, &color_intrin,
                &color2depth_extrin, &depth2color_extrin, rgb_src_pixel);

            // Verify that the depth correspondence is valid, i.e within the frame boundaries
            if ((dpt_tgt_pixel[0] <= depth_frame.get_width()) && (dpt_tgt_pixel[1] <= depth_frame.get_height()))
            {
                auto distance_ = depth_frame.get_distance(dpt_tgt_pixel[0], dpt_tgt_pixel[1]);
                // Get the depth value for the pixel found
                //cout << "Object class: " << object.first << ". The distance to the object is: " << distance_ << endl;
                //std::cout << object.second.x << ", " << object.second.y << std::endl;
            
                int x_dis = 0;
                
                if(object.second.x < center.x)
                {
                    x_dis = center.x - object.second.x;
                }
                else
                {
                    x_dis = object.second.x - center.x;
                }
                
                if(x_dis)
                {
                    float alpha = (x_dis*(fov_h/2))/(width/2); //grados
                    double alpha_r = (alpha*3.14159265)/180; //radianes
                    float x = sin(alpha_r)*distance_;
                    float y = cos(alpha_r)*distance_;
                    if(object.second.x < center.x)
                    {
                        x = -sin(alpha_r)*distance_;
                        alpha = alpha + 90; //para visualizarlo mejor
                    }
                    else
                        alpha = 90 - alpha; //para visualizarlo mejor
                    
                    //std::cout << "Distancia: " <<  distance_ << ". Ángulo: " << alpha << std::endl;
                    //std::cout << "Coordenadas mapa: (" << x  << ", " << y << ")" << std::endl;
                    
                    result.push_back(std::make_pair(distance_, cv::Point2f(x,y)));
                }
            }
        }
    }
    std::sort(result.begin(), result.end(), [] (std::pair<float, cv::Point2f> &c1,std::pair<float, cv::Point2f> &c2) { return c1.first < c2.first; });
    return result;
}

void SpecificWorker::sendTagToAgent(double x, double z, double angle)
{
    static unsigned int cont = 0;
    if (!(cont++%2)) 
    {
        try {
            tagbasedlocalization_proxy->newTagBasedPose(x, z, angle);
        }catch(...){ 
            //std::cout << "Proxy error: ObjectDetectionAgent not available" << std::endl; 
        }
    }
}

/**
 * Not used
 */

vector<std::pair<float, cv::Point2f>> SpecificWorker::posicionMarcaDesdeLateral( std::vector<std::pair<float, cv::Point2f>>& data)
{
    vector<std::pair<float, cv::Point2f>> coords_res;
    
    std::pair<float, cv::Point2f> coord1, coord2;
    
    
    if(data.size() == 4) //hay cuatro ruedas. En otros casos mirar cuáles ha visto.
    {
        //calculo el punto intermedio entre los dos
        double move_in_z = distanciaPuntos(data[0].second, data[1].second)/2;  
        
        if(distanciaPuntos(data[0].second, data[2].second) < distanciaPuntos(data[1].second, data[2].second))
        {
            coord1 = posicionMarca(data[0].second, data[2].second);
        }
        else
        {
            coord1 = posicionMarca(data[1].second, data[2].second);
        }
        
        if(distanciaPuntos(data[0].second, data[3].second) < distanciaPuntos(data[1].second, data[3].second))
        {
            coord2 = posicionMarca(data[0].second, data[3].second);
        }
        else
        {
            coord2 = posicionMarca(data[1].second, data[3].second);
            
        }
        
        if(coord1.second.x < coord2.second.x)
        {
            coord1.second.x += move_in_z;
            coord2.second.x -= move_in_z;
            coords_res.push_back(coord1);
            coords_res.push_back(coord2);
        }
        else
        {
            coord1.second.x -= move_in_z;
            coord2.second.x += move_in_z;
            coords_res.push_back(coord2);
            coords_res.push_back(coord1);
        }
    }
    else if(data.size()>2)
    {
        double move_in_z = distanciaPuntos(data[0].second, data[1].second)/2;  
        if(distanciaPuntos(data[0].second, data[2].second) < distanciaPuntos(data[1].second, data[2].second))
        {
            coord1 = posicionMarca(data[0].second, data[2].second);
            if(data[0].second.x < data[2].second.x)
            {
                coord1.second.x += move_in_z;
            }
            else
            {
                coord1.second.x -= move_in_z;
            }
        }
        else
        {
            coord1 = posicionMarca(data[1].second, data[2].second);
            if(data[1].second.x < data[2].second.x)
            {
                 coord1.second.x += move_in_z;
            }
            else
            {
                 coord1.second.x -= move_in_z;
            }
        }
        coords_res.push_back(coord1);
    }
    
    return coords_res;
}

/**
 * Deprecated
 */

float SpecificWorker::pendienteRecta(cv::Point2f& p1, cv::Point2f& p2)
{
    return (p1.x - p2.x) / (p1.y - p2.y);
}


/**
 * DEPRECATED
 * */

void SpecificWorker::alphaFromCamera(const std::vector<std::pair<int,cv::Point2f>>& objects)
{
 
    std::cout << "Solución 1:" << std::endl;
    
    cv::Point2f center(width/2, height/2);
    for(auto &e :objects)
    {
        std::cout << e.second.x << ", " << e.second.y << std::endl;
        
        int x_dis = 0;
        if(e.second.x < center.x)
        {
            x_dis = center.x - e.second.x;
        }
        else
        {
            x_dis = e.second.x - center.x;
        }
        
        if(x_dis)
        {
            int alpha = (x_dis*(fov_h/2))/(width/2);
            std::cout << "El ángulo es: " << alpha << std::endl;
        }
    }
}


/**
 * DEPRECATED
 */

void SpecificWorker::printObjectsDistance(rs2::video_frame& color_frame, rs2::depth_frame& depth_frame, const std::vector<std::pair<int, cv::Point2f>>& objects)
{
    auto depth_profile = depth_frame.get_profile().as<rs2::video_stream_profile>();
    auto color_profile = color_frame.get_profile().as<rs2::video_stream_profile>();

    auto depth_intrin = depth_profile.get_intrinsics();
    auto color_intrin = color_profile.get_intrinsics();
    auto depth2color_extrin = depth_profile.get_extrinsics_to(color_profile);
    auto color2depth_extrin = color_profile.get_extrinsics_to(depth_profile);
  
    auto sensor = selection.get_device().first<rs2::depth_sensor>();
    auto scale = sensor.get_depth_scale();
    
    for(unsigned int i = 0; i < objects.size(); i++) 
    {
        
        float rgb_src_pixel[2] = { objects[i].second.x,objects[i].second.y }; // The RGB coordinate for the center of the marble
        float dpt_tgt_pixel[2] = { 0 }; // The depth pixel that has the best match for that RGB coordinate

        // Search along a projected beam from 0.1m to 10 meter. This can be optimized to the concrete scenario, e.g. if you know that the data is bounded within [min..max] range
        rs2_project_color_pixel_to_depth_pixel(dpt_tgt_pixel, reinterpret_cast<const uint16_t*>(depth_frame.get_data()), scale, 0.1f, 2,
            &depth_intrin, &color_intrin,
            &color2depth_extrin, &depth2color_extrin, rgb_src_pixel);

        // Verify that the depth correspondence is valid, i.e within the frame boundaries
        if ((dpt_tgt_pixel[0] <= depth_frame.get_width()) && (dpt_tgt_pixel[1] <= depth_frame.get_height()))
        {
            auto distance = depth_frame.get_distance(dpt_tgt_pixel[0], dpt_tgt_pixel[1]);
            // Get the depth value for the pixel found
            cout << "Object class: " << objects[i].first << ". The distance to the object is: " << distance << endl;
        }
    }
}


int SpecificWorker::test()
{

    cv::Mat drawing(500, 1000, CV_8UC3, Scalar(10, 100, 150));
    
     
    std::vector<cv::Point2f> points;
    
    points.push_back(cv::Point2f(100,100));
    points.push_back(cv::Point2f(90,200));
    points.push_back(cv::Point2f(200,130));
    points.push_back(cv::Point2f(200,240));
  
    cv::line(drawing, points[0], points[1], cv::Scalar(255,255,0));
    cv::line(drawing, points[0], points[2], cv::Scalar(255,255,0));
    cv::line(drawing, points[2], points[3], cv::Scalar(255,255,0));
    cv::line(drawing, points[1], points[3], cv::Scalar(255,255,0));

    cv::imshow("drawing", drawing);
    cv::waitKey(0);
    
    // compute the rotated bounding rect of the points
    cv::RotatedRect boundingBox = cv::minAreaRect(points);
    // one thing to remark: this will compute the OUTER boundary box, so maybe you have to erode/dilate if you want something between the ragged lines
    
    // draw the rotated rect
    cv::Point2f corners[4];
    boundingBox.points(corners);
    cv::line(drawing, corners[0], corners[1], cv::Scalar(100,255,100));
    cv::line(drawing, corners[1], corners[2], cv::Scalar(255,255,200));
    cv::line(drawing, corners[2], corners[3], cv::Scalar(255,255,225));
    cv::line(drawing, corners[3], corners[0], cv::Scalar(255,255,255));

    // display
   // cv::imshow("input", input);
    cv::imshow("drawing", drawing);
    cv::waitKey(0);
    cv::imwrite("rotatedRect.png",drawing);

    return 0;

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
}
