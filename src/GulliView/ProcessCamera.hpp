#ifndef _PROCESSCAMERA_H_
#define _PROCESSCAMERA_H_

#include "../AprilTypes.h"
#include "../TagFamily.h"


#include "apriltag/apriltag_pose.h" // added 2025;
#include "apriltag/common/image_u8.h" // added 2025;
#include <unordered_map> // added 2025;
#include "opencv2/core/cvstd.hpp"
#include "pthread.h"
#include <optional>

#include <fstream> // added 2025;
#include <eigen3/Eigen/Dense> // added 2025;
#include <eigen3/Eigen/Geometry> // added 2025;

#include <ctime>
#include <iostream>
#include <cstdio>
#include <getopt.h>
#include <cstring>
#include <cstdlib>
#include <string>
#include <csignal>
#include <cmath>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/asio.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

#include <thread>     // added 2024
#include <queue> // added 2024
#include <vector> // added 2024
#include <tuple> // added 2024
#include <regex> // added 2024
#include <fcntl.h> // added 2024
#include <sys/stat.h> // added 2024
#include <sys/mman.h> // added 2024
#include <sstream> // added 2024
#include <boost/interprocess/sync/named_semaphore.hpp> // added 2024
#include <boost/interprocess/file_mapping.hpp> // added 2024
#include <boost/interprocess/mapped_region.hpp> // added 2024

#include <sstream>
#include <thread>

#include "../CameraUtil.h"

#include "apriltag/apriltag.h"

#include "DebugLogger.hpp"
#include "GeneralSearchFunctions.hpp"
#include "Declarations.hpp"
#include "TransformFrame.hpp"

#include "FastThread.hpp"
#include "ProducerThread.hpp"

int process_camera(int camera_id, GulliViewOptions opts);

#endif