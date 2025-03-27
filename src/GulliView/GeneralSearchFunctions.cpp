
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

#include "Declarations.hpp" // for Tag and Message structs

#include "GeneralSearchFunctions.hpp"


void update_tag(const cv::Point2f* detection,
    const cv::Point2f* cornerDetections,
    const boost::posix_time::ptime latest_frame,
    Tag* tag, std::ofstream& file_output) {
    if (tag_exists(tag->x, tag->y)) {
        tag->velocity = calc_velocity(tag->x, tag->y, 
                        detection->x, detection->y,
                        tag->latest_detection, latest_frame);
    tag->valid_velocity = true;

#if PRINT_DEBUG_MSG
file_output << "tag->x: " << tag->x << " tag->y: " << tag->y << " detection->x: " << detection->x << " detection->y: " << detection->y << std::endl;
#endif
    }
    tag->x = detection->x;
    tag->y = detection->y;
    tag->latest_detection = latest_frame;
    tag->is_detected = true;
    float x0 = (cornerDetections + 1)->x;
    float y0 = (cornerDetections + 1)->y;
    float x1 = cornerDetections->x;
    float y1 = cornerDetections->y;
    tag->theta = atan2(y1 - y0, x1 - x0);
}


float calc_velocity(const int old_x, const int old_y, 
    const int new_x, const int new_y,
    const boost::posix_time::ptime old_frame,
    const boost::posix_time::ptime new_frame) {
float dy = new_y - old_y;
float dx = new_x - old_x;
float diag = sqrt(powf(dx, 2) + powf(dy, 2));
auto elapsed = new_frame - old_frame;
auto dt = (elapsed).total_microseconds();
return diag / (dt / 1e6f);
}


// modifeid 2024, "detectoinTime_ms" added
void add_detection_to_msg(const int id, uint64_t detectionTime_ms, const float room_x, const float room_y, 
    const float theta, const size_t index, 
    const int CAM_NAME, Message& buf) {
int32_t x_coord = (int32_t) (room_x * 1000.0);
int32_t y_coord = (int32_t) (room_y * 1000.0);
union {
float        f;
unsigned int i;
} angle;
union {
float        f;
unsigned int i;
} speed_f;
float speed = 0.25f; // I HAVE SET THIS TO AN ARBITRARY VALUE SINCE REMOVING THE "safeSpeed" FUNCTION. 
                    // IT IS SENT TO SOCKET BUT NOT USED LATER.    // Convert theta to big endian angle
angle.f = theta;
angle.i = htobe32(angle.i);
// Convert speed to big endian speed
speed_f.f = speed;
speed_f.i = htobe32(speed_f.i);
buf.detections[index] = {      
htobe32(id),  /* id */
htobe64(detectionTime_ms),   /* added 2024*/
htobe32(x_coord), /* x */
htobe32(y_coord), /* y */
angle.f,          /* angle theta */
speed_f.f,        /* speed */
htobe32(CAM_NAME) /* camera_id */
};

#if PRINT_DEBUG_MSG
// cout << "[*] Camera: " << CAM_NAME << " Tag: " << id 
//     << " X: " << x_coord << " Y: " << y_coord << " Theta: " 
//     << theta << " Speed: " << speed << " Time: " 
//     << detectionTime_ms << endl;
#endif
}

void reset_tag(const int image_width, const int image_height, Tag *tag) {
    tag->x = 0;
    tag->y = 0;
    tag->valid_velocity = false;
    tag->velocity = 0;
    tag->area.x_start = 0;
    tag->area.y_start = 0;
    tag->area.x_end = image_width;
    tag->area.y_end = image_height;
    tag->area.x_length = image_width;
    tag->area.y_length = image_height;
}
