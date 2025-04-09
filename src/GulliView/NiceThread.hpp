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
#include "TransformFrame.hpp"

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