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

#ifndef _NICETHREAD_H_
#define _NICETHREAD_H_

#include <thread>
#include <queue>

#include <boost/interprocess/sync/named_semaphore.hpp>

#include "apriltag/apriltag_pose.h"

#include "../AprilTypes.h"
#include "../TagFamily.h"

#include "DebugLogger.hpp"
#include "GeneralSearchFunctions.hpp"
#include "GlobalCoordination.hpp"
#include "TransformFrame.hpp"
#include "Undistortion.hpp"

zarray* exhaustive_search(image_u8_t& im, 
                          apriltag_detector_t* detector);

int nice_consume_frame(int camera_id, 
                       boost::interprocess::named_semaphore& sem_1, 
                       boost::interprocess::named_semaphore& sem, 
                       char* shared_memory, 
                       SharedData *ptr, 
                       cv::Mat frame, 
                       cv::Mat gray, 
                       int CAM_NAME, 
                       cv::Mat map1, 
                       cv::Mat map2, 
                       GulliViewOptions opts, 
                       std::string win, 
                       DebugLogger& nice_thread_logger);

#endif