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

#include <filesystem>

#include "ProcessCamera.hpp"

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
FrameData buffer[4][BUFFER_SIZE];
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
#include "LogTime.hpp"
// Add general settings to log
void general_log(){

    // Check if the output directory exists, if not create it
    if (!std::filesystem::exists("output")) {
        std::filesystem::create_directory("output");
    }

    // Clear the output directory
    for (const auto& entry : std::filesystem::directory_iterator("output")) {
        std::filesystem::remove(entry.path());
    }

    std::ostringstream filename;
    filename << "output/general.log";
    std::ofstream file_output(filename.str(), std::ios::out);

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    file_output << "TIME: " << std::put_time(&tm, "%Y-%m-%d %H:%M:%S")              << std::endl;
    file_output << "VERSION: "                      << TIME_PERIOD<<"."<<VERSION    << std::endl; 
    file_output << "COMMENT: "                      << COMMENT                      << std::endl;

    file_output << "LIVE_FEED: "                    << LIVE_FEED                    << std::endl;
    
    #if !LIVE_FEED
    file_output << "RECORDING_FOLDER: "             << RECORDING_FOLDER             << std::endl;
    #endif

    int lenght = 1000000;
    std::ostringstream filename_test;
    filename_test << "output/timer_test_exclude.log";
    std::ofstream file_output_test(filename_test.str(), std::ios::out);

    LogTime timer_print;
    for (int i = 0; i<lenght;i++){
        LogTime test_timer;
        test_timer.stop_us("testing testing", file_output_test);
    }
    file_output << lenght << " timer print penalty: " << timer_print.stop_ms() << " ms" << std::endl;
    file_output << "1 timer print penalty: " << timer_print.stop_ns()/lenght << " ns" << std::endl;

    LogTime timer_start;
    for (int i = 0; i<lenght;i++){
        LogTime test_timer;
    }
    file_output << lenght << " timer start penalty: " << timer_start.stop_ms() << " ms" << std::endl;
    file_output << "1 timer start penalty: " << timer_start.stop_ns()/lenght << " ns" << std::endl;

    LogTime timer_ns;
    for (int i = 0; i<lenght;i++){
        LogTime test_timer;
        test_timer.stop_ns();
    }
    file_output << lenght << " timer total ns penalty: " << timer_ns.stop_ms() << " ms" << std::endl;
    file_output << "1 timer total ns penalty: " << timer_ns.stop_ns()/lenght << " ns" << std::endl;

    LogTime timer_ms;
    for (int i = 0; i<lenght;i++){
        LogTime test_timer;
        test_timer.stop_ms();
    }
    file_output << lenght << " timer total ms penalty: " << timer_ms.stop_ms() << " ms" << std::endl;
    file_output << "1 timer total ms penalty: " << timer_ms.stop_ns()/lenght << " ns" << std::endl;
    
    file_output.close();

}

// Main function
int main(int argc, char **argv) {
    std::cout << "sizeof FrameData: " << sizeof(FrameData) << " bytes" << std::endl;
    std::cout << "Total frame buffer: " << (sizeof(FrameData) * 4 * BUFFER_SIZE) / (1024*1024) << " MB" << std::endl;
    std::cout << "sizeof DetectionData: " << sizeof(DetectionData) << " bytes" << std::endl;
    std::cout << "Total search buffer: " << (sizeof(DetectionData) * 4 * BUFFER_SIZE) / (1024*1024) << " MB" << std::endl;
    
    // std::cout << "Sleeping 10 seconds - check RSS now..." << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(10));
    // Parsing command line arguments
    GulliViewOptions opts = parse_options(argc, argv);

#if ENABLE_ANY_LOGS
    // Output general settings to log
    general_log();
#endif

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
        for (int i = 0; i < opts.device_num; ++i) {
            camera_opts[i] = opts;

            camera_opts[i].shared_semaphore = "my_semaphore" + std::to_string(i+1);  // Create different semaphores for each camera
            camera_opts[i].shared_memory = "shared_memory" + std::to_string(i+1);   // Create different shared memory for each camera

            threads.push_back(std::thread(process_camera, i, std::cref(camera_opts[i])));
        }

        // Wait for all threads to complete
        for (auto& t : threads) {
            t.join();
        }
        std::cout << "All threads joined" << std::endl;
    }
    else {
        std::cerr << "Unsupported device_num: " << opts.device_num << std::endl;
        return 1; // Deal with unexpected situations
    }

    std::cout << "Exiting" << std::endl;

    return 0;
}