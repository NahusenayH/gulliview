#ifndef _FASTTHREAD_H_
#define _FASTTHREAD_H_

#include <eigen3/Eigen/Dense>

#include "AccelerationTracker.hpp"
#include "AngleTracker.hpp"
#include "CalibrateCameras.hpp"
#include "Declarations.hpp"
#include "FastSearch.hpp"
#include "GUI.hpp"
#include "InitCameras.hpp"
#include "NiceThread.hpp"

float get_uncertainty();

int fast_consume_frame(int camera_id, 
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