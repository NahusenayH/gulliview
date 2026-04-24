/*********************************************************************
* This file is distributed as part of the C++ port of the APRIL tags
* library. The code is licensed under GPLv2.
*
* Original author: Edwin Olson <ebolson@umich.edu>
* C++ port and modifications: Matt Zucker <mzucker1@swarthmore.edu>
* ----------------------- Modified ---------------------------------e
* Code modified for project in Vision Based Localization for
* Autonomous Vehicles at Chalmers University, Goteborg, Sweden
* Modification Authors:
* Copyright (c) 2013-2014 Andrew Soderberg-Rivkin <sandrew@student.chalmers.se>
* Copyright (c) 2013-2014 Sanjana Hangal <sanjana@student.chalmers.se>
* Copyright (c) 2014 Thomas Petig <petig@chalmers.se>
* Copyright (c) 2025 Emil Nylander <emilnyla@chalmers.se>
* Copyright (c) 2025 Elias Svensson <eliasve@chalmers.se>
********************************************************************/

#ifndef _FASTTHREAD_H_
#define _FASTTHREAD_H_

#include <eigen3/Eigen/Dense>

#include "AccelerationTracker.hpp"
#include "AngleTracker.hpp"
#include "CalibrateCameras.hpp"
#include "Declarations.hpp"
#include "FastSearch.hpp"
#include "GlobalCoordination.hpp"
#include "GUI.hpp"
#include "InitCameras.hpp"
#include "LogTime.hpp"
#include "NiceThread.hpp"
#include "Undistortion.hpp"

float get_uncertainty();

int fast_consume_frame(int camera_id, 
                        int thread_id, 
                        boost::interprocess::named_semaphore& sem, 
                        boost::interprocess::named_semaphore& sem_1, 
                        char* shared_memory, 
                        SharedData *ptr, 
                        cv::Mat frame, 
                        cv::Mat gray, 
                        int CAM_NAME, 
                        cv::Mat map1, 
                        cv::Mat map2, 
                        GulliViewOptions opts, 
                        std::string win, 
                        DebugLogger& fast_thread_logger);

#endif