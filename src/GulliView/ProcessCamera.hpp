#ifndef _PROCESSCAMERA_H_
#define _PROCESSCAMERA_H_

#include <sys/mman.h>

#include "FastThread.hpp"
#include "ProducerThread.hpp"

int process_camera(int camera_id, GulliViewOptions opts);

#endif