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

#ifndef _PRODUCERTHREAD_H_
#define _PRODUCERTHREAD_H_

#include <fstream>
#include <optional>
#include <thread>

#include "boost/date_time/posix_time/posix_time.hpp"

#include "Declarations.hpp"
#include "LogTime.hpp"
#include "InitCameras.hpp"

// void produce_frame(int camera_id, cv::VideoCapture *cap);
//edited to handle empty frames
void produce_frame(int camera_id, cv::VideoCapture *cap, int frame_width, int frame_height);

#endif