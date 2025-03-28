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
********************************************************************/

#include "../AprilTypes.h"
#include "../TagFamily.h"
#include "../Detections.h"


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
// #include <boost/array.hpp>
#include <boost/asio.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
// #include <boost/chrono/chrono.hpp>
// #include <boost/chrono/chrono_io.hpp>
// #include <boost/chrono/process_cpu_clocks.hpp>
// #include <boost/chrono/ceil.hpp>
// #include <boost/chrono/floor.hpp>
// #include <boost/chrono/round.hpp>
// #include <chrono>
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



// HERE WE INCLUDE THE DIFFERENT PARTS THAT WERE ONCE ONE FILE
#include "AccelerationTracker.hpp"
#include "AngleTracker.hpp"
#include "CalibrateCameras.hpp"
#include "DebugLogger.hpp"
#include "Declarations.hpp"
#include "FastSearch.hpp"
#include "FastSearchFunctions.hpp"
#include "FastThread.hpp"
#include "GeneralSearchFunctions.hpp"
#include "GUI.hpp"
#include "InitCameras.hpp"
#include "NiceThread.hpp"
#include "ProducerThread.hpp"
#include "TransformFrame.hpp"

using namespace std;
using boost::asio::ip::udp;
using boost::posix_time::ptime;
using boost::posix_time::time_duration;

// DEFINES GLOBAL VARIABLES
int nines = 0;
IntPoint mainEntry = {785, 7550};
IntPoint rampEntry = {2090, 7145};
IntPoint mainBot;
IntPoint rampBot;
uint32_t entryRadius = 200;
EntryDetection rampDetection;
// Limits of the lab
uint32_t xMinLimit = 300;
uint32_t xMaxLimit = 4600;
uint32_t yMinLimit = 500;
uint32_t yMaxLimit = 9000;
DetectionData search_buffer[4][BUFFER_SIZE];
CyclicBuffer buffer_01, buffer_10, buffer_12, buffer_21, buffer_23, buffer_32;
std::vector<std::atomic<unsigned int>> producer_counter(4);
std::vector<std::atomic<unsigned int>> search_producer_counter(4);
std::vector<std::atomic<unsigned int>> search_consumer_counter(4);
std::vector<std::atomic<unsigned int>> fast_consumer_counter(4);
std::vector<std::atomic<unsigned int>> nice_consumer_counter(4);
cv::Mat buffer[4][BUFFER_SIZE];
std::atomic<uint32_t> shared_frame_count(0);
sig_atomic_t sig_stop = 0;

void signal_handler(int signum) {
    // Unlink the shared memory
    if (shm_unlink("/my_shared_memory") == -1) {
        std::cerr << "shm_unlink failed" << std::endl;
    }

    // Remove the semaphore
    boost::interprocess::named_semaphore::remove("/my_semaphore");
    sig_stop = 1;
    exit(signum);
}

void multi_signal_handler(int signum) {
    // Unlink shared memory and remove semaphore based on device_num

    // If device_num is 5 (multi-threaded, 4 cameras), unlink multiple shared memories and remove semaphores

    for (int i = 0; i < 4; ++i) {
        std::string shm_name = "/shared_memory" + std::to_string(i+1);
        std::string sem_name = "/my_semaphore" + std::to_string(i+1);

        if (shm_unlink(shm_name.c_str()) == -1) {
            std::cerr << "shm_unlink failed for " << shm_name << std::endl;
        }

        boost::interprocess::named_semaphore::remove(sem_name.c_str());
    }


    sig_stop = 1;  // Stop flag
    exit(signum);
}

// Returns true if the tag is within the limits of the lab
bool isWithinLimits(IntPoint* point) {
    if (point->x > xMinLimit && point->x < xMaxLimit && point->y > yMinLimit
            && point->y < yMaxLimit) {
        return true;
    } else {
        return false;
    }
}

// Maximum allowed speed
float speedLimit = 0.3;

// Clamp the speed to the maximum allowed speed
float safeSpeed(float speed) {
    if (speed > speedLimit) {
        std::cout << "[!] Safety warning: attempt to set speed to " << speed << ", clamped at " << speedLimit << std::endl;
        return speedLimit;
    } else {
        return speed;
    }
}

// Return true if detection is within the entry
bool isWithinEntry(IntPoint* point, IntPoint* entry) {
    int xDiff = point->x - entry->x;
    int yDiff = point->y - entry->y;
    uint32_t distanceSq = xDiff * xDiff + yDiff * yDiff;
    return distanceSq < entryRadius * entryRadius;
}

void print_usage(const char *tool_name, FILE *output = stderr) {

    //TagDetectorParams p;
    GulliViewOptions o;

    fprintf(output, "\
Usage: %s [OPTIONS]\n\
GulliView Program used for tag detection on Autonomous Vehicles. Options:\n\
-h              Show this help message.\n\
-f FAMILY       Look for the given tag family (default \"%s\")\n\
-d DEVICE       Set camera device number (default %d)\n\
-z SIZE         Set the tag size in meters (default %f)\n\
-W WIDTH        Set the camera image width in pixels\n\
-H HEIGHT       Set the camera image height in pixels\n\
-v VELOCITY     Set the max velocity in m/s of the Wifibot (default %g)\n\
-A ACCELERATION Set the max acceleration in m/s² of the Wifibot (default %g)\n\
-c Certainty    Set the certainty in terms of nines from 1 to 6 (1: 0.9, 2: 0.99)\n\
-M              Toggle display mirroring\n\
-n              No gui\n\n\
-V              Server IP-address\n\
-B              Enable broadcast (use when broadcast IP is given for -V flag)\n\
-N              Server Port number (Default: 2121)\n",
            tool_name,
            /* Options removed that are not needed */
            /* Can be added later for further functionality */
            //p.sigma,
            //p.segSigma,
            //p.thetaThresh,
            //p.magThresh,
            //p.adaptiveThresholdValue,
            //p.adaptiveThresholdRadius,
            DEFAULT_TAG_FAMILY,
            //o.error_fraction,
            o.device_num,
            //o.focal_length,
            o.tag_size,
            o.velocity_max,
            o.acceleration_max);


fprintf(output, "Known tag families:");

    for (std::string& t : TagFamily::getFamilyNames()) {
        fprintf(output, " %s", t.c_str());
    }
    fprintf(output, "\n");
    /* Old Options removed can be re-added if they are needed. Default values set for now:
    * -D              Use decimation for segmentation stage.\n\
    * -S SIGMA        Set the original image sigma value (default %.2f).\n\
    * -s SEGSIGMA     Set the segmentation sigma value (default %.2f).\n\
    * -a THETATHRESH  Set the theta threshold for clustering (default %.1f).\n\
    * -m MAGTHRESH    Set the magnitude threshold for clustering (default %.1f).\n\
    * -V VALUE        Set adaptive threshold value for new quad algo (default %f).\n\
    * -N RADIUS       Set adaptive threshold radius for new quad algo (default %d).\n\
    * -b              Refine bad quads using template tracker.\n\
    * -r              Refine all quads using template tracker.\n\
    * -n              Use the new quad detection algorithm.\n\
    * -e FRACTION     Set error detection fraction (default %f)\n\
    * -F FLENGTH      Set the camera's focal length in pixels (default %f)\n\
    */
}

GulliViewOptions parse_options(int argc, char **argv) {
    GulliViewOptions opts;
    const char *options_str = "hDS:s:a:m:V:BN:brnf:e:d:F:z:W:H:MA:v:c: T:";
    int c;
    while ((c = getopt(argc, argv, options_str)) != -1) {
        switch (c) {
            // Reminder: add new options to 'options_str' above and print_usage()!
            case 'h':
                print_usage(argv[0], stdout);
                exit(0);
                break;
                //case 'D': opts.params.segDecimate = true; break;
                //case 'S': opts.params.sigma = atof(optarg); break;
                //case 's': opts.params.segSigma = atof(optarg); break;
                //case 'a': opts.params.thetaThresh = atof(optarg); break;
                //case 'm': opts.params.magThresh = atof(optarg); break;
                //case 'V': opts.params.adaptiveThresholdValue = atof(optarg); break;
                //case 'N': opts.params.adaptiveThresholdRadius = atoi(optarg); break;
                //case 'b': opts.params.refineBad = true; break;
                //case 'r': opts.params.refineQuads = true; break;
                //case 'n': opts.params.newQuadAlgorithm = true; break;
            case 'f':
                opts.family_str = optarg;
                break;
                //case 'e': opts.error_fraction = atof(optarg); break;
            case 'd':
                opts.device_num = atoi(optarg);
                break;
                //case 'F': opts.focal_length = atof(optarg); break;
            case 'z':
                opts.tag_size = atof(optarg);
                break;
            case 'W':
                opts.frame_width = atoi(optarg);
                break;
            case 'H':
                opts.frame_height = atoi(optarg);
                break;
            case 'A':
                opts.acceleration_max = atof(optarg);
                break;
            case 'v':
                opts.velocity_max = atof(optarg);
                break;
            case 'c':
                opts.certainty = atoi(optarg);
                break;
            case 'M':
                opts.mirror_display = !opts.mirror_display;
                break;
            case 'n':
                opts.no_gui = 1;
                break;
                // *ADDED: Flags for providing IP address and port number to server
            case 'V' :
                opts.ip = optarg;
                break;
            case 'B' :
                opts.broadcast = !opts.broadcast;
                break;
            case 'N' :
                opts.shared_semaphore = optarg; //modified 2024
                break;
            //added 2024
            case 'T' :
                opts.shared_memory = optarg; // added 2024
                break;
            default:
                fprintf(stderr, "\n");
                print_usage(argv[0], stderr);
                exit(1);
        }
    }
    // opts.params.adaptiveThresholdRadius += (opts.params.adaptiveThresholdRadius + 1) % 2;
    return opts;
}

// Working on it
// apriltag_detector_t* set_up_apriltag_detector(string& tag_family) {
//     // tag family for calibration
//     TagFamily family(tag_family);

//     // set up april tag detector
//     apriltag_detector_t* detector = apriltag_detector_create();
//     apriltag_detector_add_family(detector, family.at_family);

//     detector->quad_decimate = 1.0f;
//     detector->quad_sigma = 0.6f; // low-pass blur, negative values sharpen
//     detector->refine_edges = 1;  // align edges of rags

//     return detector;
// }

// Get the current timestamp and output the values ​​of search_producer_counter and search_consumer_counter
void print_counters(int camera_id, std::ofstream& file_output) {
    // Get the current timestamp
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    //Format timestamp
    std::tm now_tm = *std::localtime(&now_time_t);
    file_output << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S") << '.' << std::setfill('0') << std::setw(3) << now_ms.count() << " ";

    // Output the values ​​of search_producer_counter and search_consumer_counter
    file_output << "search_producer_counter[" << camera_id << "]: " << search_producer_counter[camera_id] << ", ";
    file_output << "search_consumer_counter[" << camera_id << "]: " << search_consumer_counter[camera_id] << std::endl;
}

// Map to store camera ID to Camera Name mapping
std::unordered_map<int, int> cameraMap = {
    {0, 3},
    {1, 2},
    {2, 1},
    {3, 0}
};


// Camera processing thread
int process_camera(int camera_id, GulliViewOptions opts) {

    DebugLogger fast_thread_logger(16000); // 16-bit × 16000 = 32KB
    DebugLogger nice_thread_logger(16000);


    // Initialize each to 0, added 2025
    for (int i = 0; i < 4; ++i) {
        producer_counter[i] = 0;
        search_producer_counter[i] = 0;
        search_consumer_counter[i] = 0;
        fast_consumer_counter[i] = 0;
        nice_consumer_counter[i] = 0;
    }

#if PRODUCE_FRAME_MODE == 2

    LockFreeBuffer* lock_buffer = new LockFreeBuffer();  // Initializing pointers
    LockFreeSearchBuffer* lock_search_buffer = new LockFreeSearchBuffer();

#endif

    // Initialize video capture for this camera
    cv::VideoCapture video_capture;
    cv::Mat frame, gray;

#if LIVE_FEED
    // Live feed
    init_video_capture(camera_id, opts.frame_width, opts.frame_height, video_capture, frame);
#else
    // Saved video
    init_video_open(camera_id, opts.frame_width, opts.frame_height, video_capture, frame); // added 2025
#endif

    cout << "enter resolution" << endl;

    std::cout << "Set camera to resolution: "
            << video_capture.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
            << video_capture.get(cv::CAP_PROP_FRAME_HEIGHT) << "\n";

    // Aron: purpose of these three?
    at::Point SOURCE_POINTS_PTS[4];
    at::Point DESTINATION_POINTS_PTS[4];
    int CAM_NAME;

    cv::Mat map1, map2;

    automated_calibration(opts.frame_width,
                        opts.frame_height,
                        DESTINATION_POINTS_PTS,
                        SOURCE_POINTS_PTS,
                        &CAM_NAME, 
                        video_capture,
                        map1,
                        map2,
                        camera_id); // Aron: add tag height?

    std::cout << "Camera Name " << CAM_NAME << ": camera id " << camera_id << endl; 


    at::Mat pts = getPerspectiveTransform(SOURCE_POINTS_PTS, DESTINATION_POINTS_PTS);
    
    std::string win = "Camera " + std::to_string(CAM_NAME);
    if (!opts.no_gui) {
        cv::namedWindow(win, cv::WINDOW_AUTOSIZE);
    }



#if USE_MEMORY_SHARING

    // shared memory-file between membership_service and GulliView start 
    boost::interprocess::named_semaphore sem(boost::interprocess::open_only, "/my_semaphore");

    int fd =shm_open("/my_shared_memory", O_RDWR, 0666);


    if (fd == -1) {
        std::cerr << "shm_open failed" << std::endl;
        return 1;
    }
    struct stat sb;
    if (fstat(fd, &sb) == -1) {buffer
        std::cerr << "faied to get size" << std::endl;
        return 1;
    }

    char* shared_memory = static_cast<char*>(mmap(NULL, sb.st_size, PROT_READ, MAP_SHARED, fd, 0));
    if (shared_memory == MAP_FAILED) {
        std::cerr << "mmap failed" << std::endl;
        return 1;
    }
    // shared memory-file end


#else

    static boost::interprocess::named_semaphore sem(
        boost::interprocess::open_or_create, "dummy1", 1
    );
    // static boost::interprocess::named_semaphore sem_1(
    //     boost::interprocess::open_or_create, "dummy2", 1
    // );

    char* shared_memory = nullptr;
    // SharedData* ptr = nullptr;

    //********************************************** Shared memory for sending msg to Sender (added 2024) **********************************************
    
    boost::interprocess::named_semaphore sem_1(boost::interprocess::open_or_create, opts.shared_semaphore.c_str(), 1);
    
    //std::cout << "semaphore in Gulliview  " << sem_1 << std::endl;
    std::cout<<opts.shared_semaphore.c_str()<<std::endl;
    const char *memName = opts.shared_memory.c_str();
    std::cout<<opts.shared_memory.c_str()<<std::endl;
    const size_t SIZE = sizeof(SharedData);
    int shm_fd = shm_open(memName, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        std::cerr << "shm_open failed" << std::endl;
        return 1;
    }

    if(ftruncate(shm_fd, SIZE) != 0) {
        std::cerr << "size set fail" << std::endl;
        return 1;
    }

    SharedData *ptr = (SharedData *)mmap(0, SIZE, PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED) {
        std::cerr << "mmap failed" << std::endl;
        return 1;
    }

#endif


    //********************************************** Shared memory for sending msg to Sender (added 2024) **********************************************



    std::thread nice_consumer(
        nice_consume_frame, 
        CAM_NAME, 
        std::ref(sem), 
        std::ref(sem_1), 
        shared_memory, // Passing raw pointers is fine
        ptr, // Assuming ptr is already a pointer
        frame,
        gray,
        camera_id, 
        map1, 
        map2, 

        // tags,
        opts, // Pass by reference if it's non-copyable or costly to copy
        win, // `std::string` should be wrapped in std::ref if mutable
        std::ref(nice_thread_logger)
    );

    std::thread fast_consumer(
        fast_consume_frame, 
        CAM_NAME, 
        std::ref(sem), 
        std::ref(sem_1), 
        shared_memory, // Passing raw pointers is fine
        ptr, // Assuming ptr is already a pointer
        frame,
        gray,
        camera_id, 
        map1, 
        map2, 

        // tags,
        opts, // Pass by reference if it's non-copyable or costly to copy
        win,
        std::ref(fast_thread_logger)
    );

    std::thread producer(produce_frame, camera_id, &video_capture);


    nice_consumer.join();
    fast_consumer.join();
    producer.join();
}

// Add general settings to log
void general_log(){
    #if ENABLE_LOGS
    std::ostringstream filename;
    filename << "output/general.log";
    std::ofstream file_output(filename.str(), std::ios::out);

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    file_output << "TIME: " << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << endl;
    file_output << "VERSION: " << TIME_PERIOD << "." << VERSION << endl; 
    file_output << "COMMENT: " << COMMENT << endl;

    file_output << "PRINT_DEBUG_MSG: " << PRINT_DEBUG_MSG << endl;
    file_output << "FAST_SEARCH_ACC_TEST: " << FAST_SEARCH_ACC_TEST << endl;
    file_output << "TIME_PROFILING: " << TIME_PROFILING << endl;

    file_output << "USE_MEMORY_SHARING: " << USE_MEMORY_SHARING << endl;
    file_output << "USE_EWMA: " << USE_EWMA << endl;
    file_output << "BINDING_CPU_CORES: " << BINDING_CPU_CORES << endl;

    file_output << "PRODUCE_FRAME_MODE: " << PRODUCE_FRAME_MODE << endl;
    file_output << "DEFAULT_TAG_FAMILY: " << DEFAULT_TAG_FAMILY << endl;
    file_output << "DEFAULT_IP: " << DEFAULT_IP << endl;
    file_output << "DEFAULT_PORT: " << DEFAULT_PORT << endl;

    file_output << "MAX_TAG_ID: " << MAX_TAG_ID << endl;
    file_output << "FORCE_GLOBAL_SEARCH_LOOP_NUM: " << FORCE_GLOBAL_SEARCH_LOOP_NUM << endl;

    file_output << "ROOM_WIDTH_METER: " << ROOM_WIDTH_METER << endl;

    file_output << "DEFAULT_VELOCITY_MAX: " << DEFAULT_VELOCITY_MAX << endl;
    file_output << "DEFAULT_ACCELERATION_MAX: " << DEFAULT_ACCELERATION_MAX << endl;
    file_output << "DEFAULT_LIMIT_MAX: " << DEFAULT_LIMIT_MAX << endl;

    file_output << "FPS: " << FPS << endl;
    file_output << "BUFFER_SIZE: " << BUFFER_SIZE << endl;
    file_output << "GLOBAL_SEARCH_MIN: " << GLOBAL_SEARCH_MIN << endl;

    file_output << "ENABLE_LOGS: " << ENABLE_LOGS << endl;
    file_output << "LIVE_FEED: " << LIVE_FEED << endl;
    
    #if !LIVE_FEED
    file_output << "RECORDING_FOLDER: " << RECORDING_FOLDER << endl;
    #endif

    #endif
}

// Main function
int main(int argc, char **argv) {

    // Parsing command line arguments
    GulliViewOptions opts = parse_options(argc, argv);

    // Output general settings to log
    general_log();

    // Doing graceful shutdown, prevents Linux USB system from crashing

    if (opts.device_num == 4)
        signal(SIGINT, signal_handler);
    else signal(SIGINT, signal_handler);
    
    // Single-threaded if device_num is 0 to 3
    if (opts.device_num >= 0 && opts.device_num <= 3) {
            GulliViewOptions camera_opts = opts;

        process_camera(opts.device_num, camera_opts);  // Calling individual camera handler functions directly
    }
    // If device_num is 5, start multithreading
    else if (opts.device_num == 4) {
        std::vector<std::thread> threads;

        GulliViewOptions camera_opts[4];

        // Start four threads for each of the four cameras
        for (int i = 0; i < 4; ++i) {
            camera_opts[i] = opts;

            camera_opts[i].shared_semaphore = "my_semaphore" + std::to_string(i+1);  // Create different semaphores for each camera
            camera_opts[i].shared_memory = "shared_memory" + std::to_string(i+1);   // Create different shared memory for each camera

            threads.push_back(std::thread(process_camera, i, std::cref(camera_opts[i])));
        }

        // Wait for all threads to complete
        for (auto& t : threads) {
            t.join();
        }
    }
    else {
        std::cerr << "Unsupported device_num: " << opts.device_num << std::endl;
        return 1; // Deal with unexpected situations
    }


    return 0;
}