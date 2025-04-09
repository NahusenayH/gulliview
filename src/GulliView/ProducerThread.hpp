#ifndef _PRODUCERTHREAD_H_
#define _PRODUCERTHREAD_H_

#include <fstream>
#include <optional>
#include <thread>

#include "boost/date_time/posix_time/posix_time.hpp"

#include "Declarations.hpp"

void produce_frame(int camera_id, cv::VideoCapture *cap);

#endif