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
    try {
        rs2::pipeline pipe;
        rs2::pipeline_profile pprofile = pipe.start();
#ifdef PERSON_DETECTION
        openvinom.loadAndCreateInfer();
#endif
        while (true) { 
            rs2::frameset data = pipe.wait_for_frames();    // Wait for next set of frames from the camera
            rs2::video_frame vf = data.get_color_frame();
            
            Mat frame(Size(vf.get_width(),vf.get_height()),CV_8UC3, (void*)vf.get_data(), Mat::AUTO_STEP);

            AprilTagPose robotPose;
            int frame_number = data.get_frame_number();
            if( !(frame_number % 5) ) {
                robotPose = apriltagm.detectTag(data.get_color_frame(),frame, pprofile); //Looking for the tags
                if(robotPose.isValid()) {
                    try {
                        aprilbasedlocalization_proxy->newAprilBasedPose(robotPose.getX(), robotPose.getZ(), robotPose.getAngle());
                    }catch(...) {
                        std::cout << "Proxy error: ObjectDetectionAgent not available" << std::endl;
                    }
                    std::cout << "Tag detectado" << std::endl;
                }
            }
            
#ifdef PERSON_DETECTION
            static int prev_nperson = -1;
            int nperson = openvinom.detectPerson(frame);
            QMutexLocker locker(mutex);
            if(prev_nperson != nperson) {
                try {
                    persontoagent_proxy->peopleDetected(nperson);
                }
                catch(...) {
                    std::cout << "Proxy error: PersonAgent not available" << std::endl;
                }                
                std::cout << "Personas detectadas: " << nperson << std::endl;
                prev_nperson = nperson;
            }
#endif

        }        
    }
    catch (const std::exception& error) {
        std::cerr << "[ ERROR ] " << error.what() << std::endl;
    }
    catch (...) {
        std::cerr << "[ ERROR ] Unknown/internal exception happened." << std::endl;
    }

    slog::info << "Execution successful" << slog::endl;
}



bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
}
