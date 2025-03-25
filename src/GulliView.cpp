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

#include "AprilTypes.h"
#include "TagFamily.h"
#include "Detections.h"

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

#include "CameraUtil.h"


#include "apriltag/apriltag.h"

#define PRINT_DEBUG_MSG         true
#define FAST_SEARCH_ACC_TEST    false
#define TIME_PROFILING          false

#define USE_MEMORY_SHARING      false // added 2025
#define USE_EWMA                true // added 2025
#define BINDING_CPU_CORES       true // added 2025

#define PRODUCE_FRAME_MODE      1 // added 2025

#define DEFAULT_TAG_FAMILY "tag36h11" // tag36h11
#define DEFAULT_IP "127.0.0.1"
#define DEFAULT_PORT "2121"

#define MAX_TAG_ID                      10
#define FORCE_GLOBAL_SEARCH_LOOP_NUM    10

#define ROOM_WIDTH_METER  5.035f

#define DEFAULT_VELOCITY_MAX 0.3
#define DEFAULT_ACCELERATION_MAX 8
#define DEFAULT_LIMIT_MAX 17000

#define FPS 60
#define BUFFER_SIZE 128
#define PARALLELL_FRAME_COUNT 2
#define GLOBAL_SEARCH_MIN 16

// ### ADDED MARS 2025
#define TIME_PERIOD "VT25"
// Version string, adds to time period ex VT25.2
#define VERSION "3"
// change this text to denote version, this is saved by log script to catagorize
#define COMMENT "Moved if statements"

#define ENABLE_LOGS true
#define LIVE_FEED false
#define RECORDING_FOLDER "recordings0.5"

using namespace std;
using boost::asio::ip::udp;
using boost::posix_time::ptime;
using boost::posix_time::time_duration;

string CALIBRATION_TAG_FAMILY = "tag25h9";


sig_atomic_t sig_stop = 0;

int nines = 0;

// added 2025
// Global shared frame counter (thread-safe)
std::atomic<uint32_t> shared_frame_count(0);

// boost::interprocess::named_semaphore sem(boost::interprocess::open_only, "/my_semaphore");

// int fd = shm_open("/my_shared_memory", O_RDWR, 0666);


/*if (fd == -1) {
    std::cerr << "shm_open failed" << std::endl;
    return 1;
}*/

// struct stat sb;
/*if (fstat(fd, &sb) == -1) {
    std::cerr << "faied to get size" << std::endl;
    return 1;
}*/

// char* shared_memory = static_cast<char*>(mmap(NULL, sb.st_size, PROT_READ, MAP_SHARED, fd, 0));
/*if (shared_memory == MAP_FAILED) {
    std::cerr << "mmap failed" << std::endl;
    return 1;
}*/

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


typedef struct __attribute__ ((packed)) DetectionMessage {
    uint32_t id;
    uint64_t time_msec;    //added 2024
    uint32_t x;
    uint32_t y;
    float theta;
    float speed;
    uint32_t camera_id;
} DetectionMessage;

typedef struct __attribute__ ((packed)) DetectionArea {
    int32_t x_start;
    int32_t y_start;
    int32_t x_end;
    int32_t y_end;
    int32_t x_length;
    int32_t y_length;
} DetectionArea;

typedef struct __attribute__ ((packed)) Message {
    uint32_t type;
    uint32_t subtype;
    uint32_t seq;
    uint64_t time_msec;
    uint64_t avg_time_gap;   //modified 2024, previously UNUSED
    uint32_t cam_id;    // previously UNUSED2
    uint32_t length;
    DetectionMessage detections[11];
} Message;

struct SharedData {
    int flag;
    Message msg;
};

typedef struct GulliViewOptions {
    GulliViewOptions() :
            family_str(DEFAULT_TAG_FAMILY),
            error_fraction(1),
            device_num(0),
            focal_length(500),
            tag_size(0.1905),
            frame_width(1920),
            frame_height(1080),
            acceleration_max(DEFAULT_ACCELERATION_MAX),
            velocity_max(DEFAULT_VELOCITY_MAX),
            /* Changed to False so that text comes out correctly. */
            /* Issues with detection when set to False */
            mirror_display(false), //Change to true?? merge
            no_gui(false), //merge
            // *ADDED: Default value for IP address and port number to server
            ip(DEFAULT_IP),
            broadcast(false),
            port(DEFAULT_PORT) ,
            shared_memory(), // added 2024
            shared_semaphore()
            {
    }

    std::string family_str;
    double error_fraction;
    int device_num;
    double focal_length;
    double tag_size;
    int frame_width;
    int frame_height;
    float acceleration_max;
    float velocity_max;
    int certainty;
    bool mirror_display;
    bool no_gui;
    // *ADDED: Variables for storing IP address and port number to server
    std::string ip;
    bool broadcast;
    std::string port;
    std::string shared_memory; // added 2024
    std::string shared_semaphore; // added 2024
} GulliViewOptions;

typedef struct IntPoint {
    uint32_t x;
    uint32_t y;
} IntPoint;

IntPoint mainEntry = {785, 7550};
IntPoint rampEntry = {2090, 7145};
IntPoint mainBot;
IntPoint rampBot;
uint32_t entryRadius = 200;

//std::shared_ptr<EntryDetection> mainDetection = std::make_shared<EntryDetection>();

EntryDetection rampDetection;

// Limits of the lab
uint32_t xMinLimit = 300;
uint32_t xMaxLimit = 4600;
uint32_t yMinLimit = 500;
uint32_t yMaxLimit = 9000;

typedef struct Tag {
    int32_t x = 0;
    int32_t y = 0;
    bool is_detected = 0;
    float velocity = 0;
    bool valid_velocity = false;
    float theta;
    ptime latest_detection;
    DetectionArea area;
} Tag;


// added 2025
struct DetectionData {
    // timestamp
    std::chrono::system_clock::time_point timestamp;

    // Structures representing coordinates
    struct CameraCoordinates {
        float x, y, theta;  // camera coordinate (x, y, theta)
    };

    struct SpaceCoordinates {
        double x, y, z;  // space coordinate (x, y, z)
    };

    // single tag data
    struct TagData {
        bool found;  // find or not
        // DetectionArea area;
        std::optional<CameraCoordinates> camera_coords;  // Camera coordinates, valid when found
        std::optional<SpaceCoordinates> space_coords;  // Spatial coordinates, valid when found

        TagData() : found(false) {}  // Default constructor, initialised to not found
    };

    // 10 tag data
    std::array<TagData, 10> tags;

    Tag tag_data[MAX_TAG_ID];

    // Optional packed Message buffer
    std::optional<Message> buf;

    // Default constructor to initialize timestamp
    DetectionData() : timestamp(std::chrono::system_clock::now()) {}

    // Method to copy an existing Message into DetectionData
    void storeMessage(const Message& message) {
        buf.emplace();  // Create space for a Message in the optional
        std::memcpy(&(*buf), &message, sizeof(Message)); // Copy the content
    }

    // Method to clear the stored Message
    void clearMessage() {
        buf.reset();
    }

};


cv::Mat buffer[4][BUFFER_SIZE]; // modified 2025

DetectionData search_buffer[4][BUFFER_SIZE]; // added 2025

// using for storing data produced by producer
struct BufferData {
    cv::Mat frame;         // original frame
    cv::Mat gray;          // gray frame

};


// Overlapping data structure
struct OverlapTagInfo {
    int tag_id;
    float a_max;
    float alpha;
    ptime timestamp;         // Timestamp indicating the time of inspection at the time of production
};

// cyclic buffer
class CyclicBuffer {
public:
    OverlapTagInfo buffer[BUFFER_SIZE];
    std::atomic<unsigned int> producer_counter{0};
    std::atomic<unsigned int> consumer_counter{0};

    // Producer：write data
    void produce(const OverlapTagInfo& data) {
        unsigned int next = (producer_counter + 1) % BUFFER_SIZE;
        while (next == consumer_counter.load(std::memory_order_acquire)) {
            std::this_thread::yield(); // Buffer full, wait
        }
        buffer[producer_counter] = data;
        producer_counter.store(next, std::memory_order_release);
    }

    // Consumer：read data
    bool consume(OverlapTagInfo& data) {
        if (consumer_counter.load(std::memory_order_acquire) == producer_counter.load(std::memory_order_relaxed)) {
            return false; // Buffer is empty 
        }
        data = buffer[consumer_counter];
        consumer_counter.store((consumer_counter + 1) % BUFFER_SIZE, std::memory_order_release);
        return true;
    }
};

// Global buffer definition
CyclicBuffer buffer_01, buffer_10, buffer_12, buffer_21, buffer_23, buffer_32;

struct OverlapRange {
    int min_y;
    int max_y;
};

// Table of overlapping ranges
const OverlapRange overlap_ranges[4][2] = {
    // Overlap range for camera 0
    {{0, 500}, {0, 0}}, // buffer_01
    // Camera 1's overlap range
    {{1660, 2160}, {0, 580}}, // buffer_10, buffer_12
    // Camera 2's overlap area
    {{1580, 2160}, {0, 460}}, // buffer_21, buffer_23
    // Camera 3's overlap area
    {{1700, 2160}, {0, 0}} // buffer_32
};


// added 2025

class DebugLogger {
private:
    struct LogEntry {
        uint8_t reason; // reason enumeration (1 byte)
        uint32_t value; // value (4 bytes)
        uint8_t counter; // current count (max 128, 1 byte)
        uint32_t timestamp; // timestamp (4 bytes)
    };

    LogEntry* log_buffer; // fixed size memory array
    size_t buffer_capacity; // array capacity (16000 entries = 32 KB)
    size_t write_index; // current write index
    bool is_buffer_full; // Flag if the oldest data has been overwritten.

public:
    enum Reason : uint8_t {
        TRANSFORM_TIME = 0,
        CONSUMER_TIME,
        INITIALIZE_TIME,
        PART_SEARCH_TIME,
        PROCESS_TIME,
        GLOBAL_SEARCH_TIME,
        LOOP_TIME
    };

    DebugLogger(size_t max_entries = 16000) {
        buffer_capacity = max_entries;
        log_buffer = new LogEntry[buffer_capacity];
        write_index = 0;
        is_buffer_full = false;
    }

    ~DebugLogger() {
        delete[] log_buffer;
    }

    void log_operation(Reason reason, uint32_t value, uint8_t counter) {
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);

        // fill in the log entry
        log_buffer[write_index] = {reason, value, counter, static_cast<uint32_t>(now_time_t)};

        // update the write index, use it as circularly
        write_index = (write_index + 1) % buffer_capacity;
        if (write_index == 0) {
            is_buffer_full = true;
        }
    }

    void write_to_file_if_needed(uint32_t loop_time, const std::string& thread_name, const std::string& file_name) {
        if (loop_time > 100000) {
            std::ofstream file(file_name, std::ios::app);
            if (file.is_open()) {
                file << "Thread: " << thread_name << ", Loop time exceeded: " << std::fixed << std::setprecision(2)<< loop_time / 1000.0 << "ms\n";
                // size_t start_index = is_buffer_full ? write_index : 0;
                size_t end_index = is_buffer_full ? buffer_capacity : write_index;

                size_t start_index = thread_name == "fast-producer"? end_index - 5 : end_index - 5;

                // size_t latest_index = is_buffer_full ? buffer_capacity - 1 : write_index - 1;
                // const auto& entry = log_buffer[latest_index];

                for (size_t i = start_index; i < end_index; ++i) {
                    const auto& entry = log_buffer[i];
                    file << "Reason: " << reason_to_string(static_cast<Reason>(entry.reason))
                         << ", Value: " << std::fixed << std::setprecision(2) << static_cast<int>(entry.value) / 1000.0
                         << ", Counter: " << static_cast<int>(entry.counter)
                         << ", Timestamp: " << entry.timestamp << "\n";
                }
                file << "----------------------------------------\n";
                file.close();
            }
        }
    }

private:
    std::string reason_to_string(Reason reason) {
        switch (reason) {
            case TRANSFORM_TIME: return "transform time";
            case CONSUMER_TIME: return "consumer time";
            case INITIALIZE_TIME: return "initialize time";
            case PART_SEARCH_TIME: return "part search time";
            case PROCESS_TIME: return "process time";
            case GLOBAL_SEARCH_TIME: return "global search time";
            case LOOP_TIME: return "loop time";
            default: return "unknown";
        }
    }
};


#include <iomanip>  // For std::fixed and std::setprecision
class AccelerationTracker {
private:
    struct DataPoint {
        float value; // The value of the data
        std::chrono::steady_clock::time_point timestamp; // The time the data was added
    };

    std::deque<DataPoint> last_10_numbers; // Stores the last 10 valid numbers with timestamps
    float average; // Running average of valid numbers

public:
    // Constructor initializes average to 0
    AccelerationTracker() : average(0.0) {}

    // Method to add a number to the tracker (only if the number is greater than the current average)
    void add_number(float number) {
        // Use absolute value for the number
        float abs_number = std::abs(number);

        // Compare the absolute value of the new number with the absolute value of the average
        if (abs_number > std::abs(average)) {
            // Add the new data point with the current time
            last_10_numbers.push_back({abs_number, std::chrono::steady_clock::now()});

            // Remove elements older than 5 seconds
            remove_old_elements();

            // If the deque exceeds 10 elements, remove the oldest
            if (last_10_numbers.size() > 10) {
                last_10_numbers.pop_front();
            }

            // Update the average using the new formula
            update_average(abs_number);
        }
    }

    // Method to get the current average of the numbers in the deque
    float get_average() {
        return average;
    }

    // Method to get the maximum value among the last 10 numbers
    float get_max_value(int epsilon) {
        // Find the maximum value in the deque
        float max_in_deque = 0;
        for (const auto& data : last_10_numbers) {
            max_in_deque = std::max(max_in_deque, data.value);
        }

        // Calculate the adjustment
        float adjustment = float(epsilon) / 100 * (max_in_deque + 1);

        return max_in_deque + adjustment;
    }

private:
    // Method to remove elements older than 5 seconds
    void remove_old_elements() {
        auto now = std::chrono::steady_clock::now();
        while (!last_10_numbers.empty() &&
               std::chrono::duration_cast<std::chrono::seconds>(now - last_10_numbers.front().timestamp).count() > 5) {
            last_10_numbers.pop_front();
        }
    }

    // Method to update the average
    void update_average(float number) {
        float simple_average = (std::abs(average) + std::abs(number)) / 2.0;
        average = simple_average;
    }
};

class AngleTracker {
private:
    struct DataPoint {
        float value; // The value of the data
        std::chrono::steady_clock::time_point timestamp; // The time the data was added
    };

    std::deque<DataPoint> last_10_numbers; // Stores the last 10 valid numbers with timestamps
    float average; // Running average of valid numbers

public:
    // Constructor initializes average to 0
    AngleTracker() : average(0.0) {}

    // Method to add a number to the tracker (only if the number is greater than the current average)
    void add_number(float number) {
        // Use absolute value for the number
        float abs_number = std::abs(number);

        // Compare the absolute value of the new number with the absolute value of the average
        if (abs_number > std::abs(average)) {
            // Add the new data point with the current time
            last_10_numbers.push_back({abs_number, std::chrono::steady_clock::now()});

            // Remove elements older than 5 seconds
            remove_old_elements();

            // If the deque exceeds 10 elements, remove the oldest
            if (last_10_numbers.size() > 10) {
                last_10_numbers.pop_front();
            }

            // Update the average using the new formula
            update_average(abs_number);
        }
    }

    // Method to get the current average of the numbers in the deque
    float get_average() {
        return average;
    }

    // Method to get the maximum value among the last 10 numbers
    float get_max_value(int epsilon) {
        // Find the maximum value in the deque
        float max_in_deque = 0;
        for (const auto& data : last_10_numbers) {
            max_in_deque = std::max(max_in_deque, data.value);
        }

        // Calculate the adjustment
        float adjustment = float(epsilon) / 100 * (max_in_deque + 1);

        return max_in_deque + adjustment;
    }

private:
    // Method to remove elements older than 5 seconds
    void remove_old_elements() {
        auto now = std::chrono::steady_clock::now();
        while (!last_10_numbers.empty() &&
               std::chrono::duration_cast<std::chrono::seconds>(now - last_10_numbers.front().timestamp).count() > 5) {
            last_10_numbers.pop_front();
        }
    }

    // Method to update the average
    void update_average(float number) {
        float simple_average = (std::abs(average) + std::abs(number)) / 2.0;
        average = simple_average;
    }
};



// std::atomic<unsigned int> producer_counter(0);
// std::atomic<unsigned int> consumer_counter(0);

// modified 2025

std::vector<std::atomic<unsigned int>> producer_counter(4);
std::vector<std::atomic<unsigned int>> search_producer_counter(4);
std::vector<std::atomic<unsigned int>> search_consumer_counter(4);
std::vector<std::atomic<unsigned int>> fast_consumer_counter(4);
std::vector<std::atomic<unsigned int>> nice_consumer_counter(4);


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


bool tag_exists(const int x_center, const int y_center) {
    return 0 < x_center && 0 < y_center;
}


//sets the true point of the tags for autocalibration (coordinates in meters)
//TODO Maybe change to coordinates in centimeters
 void setDestinationPoints(const int cam_name, at::Point* destination) {

    // _____________________________________________________________________________________________________
    // TODO: UPDATE DESTINATION POINTS TO THE POSITION OF THE ROBOT IF ITS TAG IS IN THE CALIBRATIONTAGS POSITION
    //this is due to the tags being on the floor while the tags on the robots are a bit off the floor therefore the true
    //position of the tag is not the same as the position we want to give to a robot in these pixel coordinates.
    // _____________________________________________________________________________________________________
    switch(cam_name) {
    case 0:
        destination[0] = at::Point(0.35, 0.68);
        destination[1] = at::Point(4.71, 0.69);
        destination[2] = at::Point(0.35, 2.60);
        destination[3] = at::Point(4.60, 2.74);
        break;
    case 1:
        destination[0] = at::Point(0.35, 2.60);
        destination[1] = at::Point(4.60, 2.74);
        destination[2] = at::Point(0.25, 4.84);
        destination[3] = at::Point(4.55, 4.91);
    case 2:
        destination[0] = at::Point(0.25, 4.84);
        destination[1] = at::Point(4.55, 4.91);
        destination[2] = at::Point(0.23, 6.95);
        destination[3] = at::Point(4.51, 7.02);
    case 3:
        destination[0] = at::Point(0.23, 6.95);
        destination[1] = at::Point(4.51, 7.02);
        destination[2] = at::Point(0.17, 9.26);
        destination[3] = at::Point(4.61, 9.18);
    }
}

float get_uncertainty() {
    float t_min;
    float t_max;
    switch (nines) {
    case 1:
        t_min = 0.031;
        t_max = 0.094;
        break;
    case 2:
        t_min = 0.031;
        t_max = 0.097;
        break;
    case 3:
        t_min = 0.028;
        t_max = 0.100;
        break;
    case 4:
        t_min = 0.023;
        t_max = 0.100;
        break;
    case 5:
        t_min = 0.015; //0.021;
        t_max = 0.200; //0.875;
        break;
    case 6:
        t_min = 0.015; //0.0165;
        t_max = 0.200; //8.8285;
        break;
    default:
        t_min = 0;
        t_max = 0;
    }
    return t_max - t_min;
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

class Log_Time {
    public:
        // constructor autmatically starts clock
        Log_Time(const std::string& input_name, std::ofstream& input_file) 
                : name(input_name), file(input_file) {
            start_time = std::chrono::high_resolution_clock::now();
        }

        // Stop clock and print to log file only if logs are enabled
        void stop_ms(){
            end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            file << name << ": "  << std::fixed << std::setprecision(2) << duration << " ms" << std::endl;
        }
        void stop_us(){
            end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
            file << name << ": " << std::fixed << std::setprecision(2) << duration << " us" << std::endl;
        }
    private:
        std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
        std::chrono::time_point<std::chrono::high_resolution_clock> end_time;
        string name;
        std::ofstream& file;
};

std::ofstream log_file(const std::string& name){
    #if ENABLE_LOGS
    std::ostringstream filename;
    filename << "output/" << name << ".log";
    std::ofstream file_output(filename.str(), std::ios::out);
    return file_output;
    #endif
}

bool transform_frame(cv::Mat& frame,
			cv::Mat& gray,
			cv::Mat& map1,
			cv::Mat& map2,
            std::ofstream& file_output // added 2025
            ) {
    // TODO save timestamp (maybe return the timestamp instead of bool)
    // cv::Mat undistorted_frame;

    if (frame.empty()) {
        cout << "no frame to transform, exiting" << endl;
        return false;
    }
#if ENABLE_LOGS
    Log_Time remap_timer("Remap", file_output);
#endif
    // cv::remap(frame, frame, map1, map2, cv::INTER_LINEAR);
#if ENABLE_LOGS
    remap_timer.stop_us();
#endif

#if ENABLE_LOGS
    Log_Time color_timer("Transform color", file_output);
#endif
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
#if ENABLE_LOGS
    color_timer.stop_us();
#endif

    return !frame.empty();
}

bool transform_frame(cv::Mat& frame,
			cv::Mat& gray,
			cv::Mat& map1,
			cv::Mat& map2) {
    // TODO save timestamp (maybe return the timestamp instead of bool)
    // cv::remap(frame, frame, map1, map2, cv::INTER_LINEAR);


    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    return !frame.empty();
}

std::map<int, Eigen::Matrix4d> camera_to_world_matrices = {
    {0, (Eigen::Matrix4d() << 2270.416948, 0.0, 1997.865610, 0.0,
                              0.0, 2267.062650, 1060.169248, 0.0,
                              0.0, 0.0, 1.0, 0.0,
                              0.0, 0.0, 0.0, 1.0).finished()},
    {1, (Eigen::Matrix4d() << 2290.512606, 0.0, 1923.229236, 0.0,
                              0.0, 2283.316632, 1028.981542, 0.0,
                              0.0, 0.0, 1.0, 0.0,
                              0.0, 0.0, 0.0, 1.0).finished()},
    {2, (Eigen::Matrix4d() << 2292.991706, 0.0, 1907.952064, 0.0,
                              0.0, 2292.022166, 1143.737530, 0.0,
                              0.0, 0.0, 1.0, 0.0,
                              0.0, 0.0, 0.0, 1.0).finished()},
    {3, (Eigen::Matrix4d() << 2297.690708, 0.0, 1974.297454, 0.0,
                              0.0, 2273.936552, 1047.443252, 0.0,
                              0.0, 0.0, 1.0, 0.0,
                              0.0, 0.0, 0.0, 1.0).finished()},
};



// define global variables
std::map<int, cv::Mat> camera_matrices = {
    {0, (cv::Mat_<double>(3, 3) << 2270.416948, 0.0, 1997.865610,
                                   0.0, 2267.062650, 1060.169248,
                                   0.0, 0.0, 1.0)},
    {1, (cv::Mat_<double>(3, 3) << 2290.512606, 0.0, 1923.229236,
                                   0.0, 2283.316632, 1028.981542,
                                   0.0, 0.0, 1.0)},
    {2, (cv::Mat_<double>(3, 3) << 2292.991706, 0.0, 1907.952064,
                                   0.0, 2292.022166, 1143.737530,
                                   0.0, 0.0, 1.0)},
    {3, (cv::Mat_<double>(3, 3) << 2297.690708, 0.0, 1974.297454,
                                   0.0, 2273.936552, 1047.443252,
                                   0.0, 0.0, 1.0)},
};

std::map<int, cv::Mat> global_distortion_coefficients = {
    {0, (cv::Mat_<double>(1, 5) << -0.073165, 0.025731, 0.006159, 0.007375, 0.0)},
    {1, (cv::Mat_<double>(1, 5) << -0.079821, 0.027860, -0.002763, -0.00027, 0.0)},
    {2, (cv::Mat_<double>(1, 5) << -0.080634, 0.032041, 0.006835, 0.000915, 0.0)},
    {3, (cv::Mat_<double>(1, 5) << -0.095870, 0.038556, 0.007781, 0.001609, 0.0)},
};

// void init_undistortion_matrices(const cv::VideoCapture& video_capture,
//                                 const cv::Size frame_size,
// 				cv::Mat& map1,
// 				cv::Mat& map2) {
//     // create 3x3 matrix of doubles
//     cv::Mat k1 = (cv::Mat1d(3, 3) << (927.42805517 / 800) * video_capture.get(cv::CAP_PROP_FRAME_WIDTH), 0.0, (401.59811614 / 800) * video_capture.get(cv::CAP_PROP_FRAME_WIDTH), 0, 
//                                     (850.04900153 / 448) * video_capture.get(cv::CAP_PROP_FRAME_HEIGHT), (225.08468986 / 448) * video_capture.get(cv::CAP_PROP_FRAME_HEIGHT), 0, 0, 1);
//     // create 1x5 matrix of doubles
//     cv::Mat d1 = (cv::Mat1d(1, 5) << 0.24592604, -1.97913584, -0.01938124, 
//                                      0.00740747, 2.37610561);

//     cv::Mat opt1 = cv::getOptimalNewCameraMatrix(k1, d1, frame_size, 0);

//     cv::initUndistortRectifyMap(k1, d1, cv::Mat(), opt1, frame_size, CV_32FC1, map1, map2);
// }

// added 2025

void init_undistortion_matrices(const cv::VideoCapture& video_capture,
                                const cv::Size& frame_size,
                                cv::Mat& map1,
                                cv::Mat& map2,
                                int camera_id) {
    // Check that the camera number is valid
    if (camera_matrices.find(camera_id) == camera_matrices.end() ||
        global_distortion_coefficients.find(camera_id) == global_distortion_coefficients.end()) {
        throw std::invalid_argument("Invalid camera ID or parameters not found!");
    }

    // Get camera matrix and distortion factor
    const cv::Mat& camera_matrix = camera_matrices[camera_id];
    const cv::Mat& distortion_coefficients = global_distortion_coefficients[camera_id];

    // Computationally optimised new camera matrices
    cv::Mat optimal_camera_matrix = cv::getOptimalNewCameraMatrix(
        camera_matrix, distortion_coefficients, frame_size, 1, frame_size);

    // Initialise the mapping matrix
    cv::initUndistortRectifyMap(
        camera_matrix, distortion_coefficients, cv::Mat(),
        optimal_camera_matrix, frame_size, CV_32FC1, map1, map2);
}


void automated_calibration(const int32_t width,
                            const int32_t height,
                            at::Point* destination,
                            at::Point* source,
                            int32_t* camera,
                            cv::VideoCapture& video_capture,
		            cv::Mat &map1,
                            cv::Mat &map2, int camera_id) { // modified 2025, add camera_id
    // declare frame
    cv::Mat frame, gray;

    // get frame from video_capture
    video_capture >> frame;

    if (frame.empty()) {
        // no frame was found, ouputs error and exits
        std::cerr << "no frames\n";
        // exit(1);
    }

    // tag family for calibration
    TagFamily family(CALIBRATION_TAG_FAMILY);

    // set up april tag detector
    apriltag_detector_t* detector = apriltag_detector_create();
    apriltag_detector_add_family(detector, family.at_family);

    detector->quad_decimate = 1.0f;
    detector->quad_sigma = 0.6f; // low-pass blur, negative values sharpen
    detector->refine_edges = 1; // align edges of rags

    init_undistortion_matrices(video_capture, frame.size(), map1, map2, camera_id); // modified 2025

    video_capture.read(frame);

    transform_frame(frame, gray, map1, map2);

    image_u8_t im = {
        gray.cols,
        gray.rows,
        gray.cols,
        gray.data   
    };
    
    zarray_t* detections = apriltag_detector_detect(detector, &im);

    if (zarray_size(detections) != 4){
        apriltag_detections_destroy(detections);
        apriltag_detector_destroy(detector);

        // start new calibration if all calibration tags were not found
        automated_calibration(width,
                             height,
                             destination,
                             source,
                             camera,
                             video_capture,
                             map1,
                             map2,
                             camera_id);
        return;
    }

    int32_t tag_sum = 0;

    // get detections of the calibration tags
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* detection;
        zarray_get(detections, i, &detection);

        // sum id's to figure out which camera 
        tag_sum += detection->id;
    }

    // updating camera name to the real one
    int32_t cam_name = (tag_sum - 6) / 8;

    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* detection;
        zarray_get(detections, i, &detection);

        // set source points (pixel coordinates for the calibration tags)
        double x = detection->c[0];
        double y = detection->c[1];
        int32_t src_index = detection->id - (cam_name * 2);
        source[src_index] = at::Point(x, y);
    }

    apriltag_detections_destroy(detections);
    apriltag_detector_destroy(detector);

    setDestinationPoints(cam_name, destination);
    *camera = cam_name;

    cout << "Calibration done for camera " << cam_name << "\n";
}

void init_video_open(const int32_t device_number,
                        const int32_t frame_width,
                        const int32_t frame_height,
                        cv::VideoCapture& video_capture,
                        cv::Mat& frame){

    std::cout << "device number: " << device_number << std::endl;
    string folder = RECORDING_FOLDER;
    string video_path = "../src/" + folder + "/video" + to_string(device_number*2) + ".mp4";
    // Open video file，instead of cameras
    video_capture = cv::VideoCapture(video_path, cv::CAP_FFMPEG); // Decoding with FFmpeg

    if (!video_capture.isOpened()) {
        std::cerr << "Error: Unable to open video file: " << video_path << std::endl;
        return;
    }

    // Set the video resolution (may not be able to change as the file has a fixed resolution)
    if (frame_width && frame_height) {
        video_capture.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
        video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
    }

    // Read the first frame
    video_capture >> frame;

    if (frame.empty()) {
        std::cerr << "Error: Unable to read frames from video file: " << video_path << std::endl;
        return;
    }

    std::cout << "Video capture initialized successfully." << std::endl;
    std::cout << "Video resolution: " 
              << video_capture.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << video_capture.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    std::cout << "Frame rate: " << video_capture.get(cv::CAP_PROP_FPS) << " FPS" << std::endl;
}


void init_video_capture(const int32_t device_number,
    const int32_t frame_width,
    const int32_t frame_height,
    cv::VideoCapture& video_capture,
    cv::Mat& frame){
/* choose camera and buffer-size */
cout << "Camera " << device_number << " init" << endl;

video_capture = cv::VideoCapture(2*device_number, cv::CAP_V4L2);
video_capture.set(cv::CAP_PROP_BUFFERSIZE, 1);

/* set output codec and FPS */
int32_t codec = cv::VideoWriter::fourcc('M','J','P','G');
video_capture.set(cv::CAP_PROP_FOURCC, codec);
video_capture.set(cv::CAP_PROP_FPS, FPS);

/* set video height and width */
if (frame_width && frame_height) {
// Use uvcdynctrl to figure this out dynamically at some point?
video_capture.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
}

video_capture >> frame;

cout << "enter here" << endl;

std::cout << "Frame rate: " << video_capture.get(cv::CAP_PROP_FPS) << " FPS" << std::endl;

if (frame.empty()) {
cerr << "no frames from camera " << device_number << endl;
// exit(1);
}
}


void set_search_area(const int32_t im_width,
                     const int32_t im_height,
                     const int min_search_dim,
                     const float min_travel,
                     const float max_travel,
                     const float alpha,
                     Tag& tag) {
    DetectionArea& area = tag.area;
    vector<float> xs{
        0.0f,
        max_travel * cosf(tag.theta),
        max_travel * cosf(tag.theta + alpha),
        max_travel * cosf(tag.theta - alpha),
        min_travel * cosf(tag.theta),
        min_travel * cosf(tag.theta + alpha),
        min_travel * cosf(tag.theta - alpha)
    };
        
    vector<float> ys{
        0.0f,
        max_travel * sinf(tag.theta),
        max_travel * sinf(tag.theta + alpha),
        max_travel * sinf(tag.theta - alpha),
        min_travel * sinf(tag.theta),
        min_travel * sinf(tag.theta + alpha),
        min_travel * sinf(tag.theta - alpha)
    };
    
    vector<float>::iterator x_min = min_element(xs.begin(), xs.end());
    vector<float>::iterator x_max = max_element(xs.begin(), xs.end());
    vector<float>::iterator y_min = min_element(ys.begin(), ys.end());
    vector<float>::iterator y_max = max_element(ys.begin(), ys.end());
    area.x_start = max(*x_min + tag.x - min_search_dim, 0.0f);
    area.y_start = max(*y_min + tag.y - min_search_dim, 0.0f);
    area.x_end = min(*x_max + tag.x + min_search_dim, float(im_width));
    area.y_end = min(*y_max + tag.y + min_search_dim, float(im_height));

    area.x_length = area.x_end - area.x_start;
    area.y_length = area.y_end - area.y_start;
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

zarray* exhaustive_search(image_u8_t& im, apriltag_detector_t* detector) {
    //detect tags

    return apriltag_detector_detect(detector, &im);
}

image_u8_t* get_partial_image(const image_u8_t& im, const DetectionArea& area){
    image_u8_t* im_part = image_u8_create(area.x_length, area.y_length);
    int y_part = 0;
    for(int y = area.y_start; y < area.y_end; y++){
        uint8_t* dest = &im_part->buf[y_part * im_part->stride];
        uint8_t* src = &im.buf[y*im.stride + area.x_start];
        size_t size = area.x_length;
        memcpy(dest, src, size);
        y_part++;
    }
    return im_part;
}

void partial_search(const image_u8_t& im,
                    const DetectionArea& area,
                    zarray_t* detections,
                    apriltag_detector_t* detector) {
    image_u8_t* im_part = get_partial_image(im, area);
    //detect tags in part image
    zarray_t *detection = apriltag_detector_detect(detector, im_part);
    if(zarray_size(detection) != 0){
        apriltag_detection_t *temp;
        zarray_get(detection, 0, &temp);
        zarray_add(detections, &temp);
    }
}

void partial_search1(const image_u8_t& im,
                    const DetectionArea& area,
                    zarray_t* detections,
                    apriltag_detector_t* detector,
                    ptime total_start_time,
                    std::ofstream& file_output
                    ) {

    auto start = std::chrono::high_resolution_clock::now();

    image_u8_t* im_part = get_partial_image(im, area);

    auto end = std::chrono::high_resolution_clock::now();

#if PRINT_DEBUG_MSG
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    file_output << "Get partial image time: " << std::fixed << std::setprecision(2) << duration << " ms" << std::endl;
#endif

    ptime search_end = boost::posix_time::microsec_clock::universal_time();

    uint32_t total_time = (search_end - total_start_time).total_microseconds();    
    if (total_time > 17000)
        return;

    start = std::chrono::high_resolution_clock::now();

    //detect tags in part image
    zarray_t *detection = apriltag_detector_detect(detector, im_part);

    end = std::chrono::high_resolution_clock::now();

#if PRINT_DEBUG_MSG
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    file_output << "Apriltag detector detect time: " << std::fixed << std::setprecision(2) << duration << " ms" << std::endl;
#endif

    search_end = boost::posix_time::microsec_clock::universal_time();

    total_time = (search_end - total_start_time).total_microseconds();    
    if (total_time > 17000)
        return;

    start = std::chrono::high_resolution_clock::now();


    if(zarray_size(detection) != 0){
        apriltag_detection_t *temp;
        zarray_get(detection, 0, &temp);
        zarray_add(detections, &temp);
    }

    end = std::chrono::high_resolution_clock::now();

#if PRINT_DEBUG_MSG
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    file_output << "Zarray time: " << std::fixed << std::setprecision(2) << duration << " ms" << std::endl;
#endif

}

float calc_displacement(const float velocity, 
                        const float time_s,
                        const float acceleration) {
    return velocity * time_s + 0.5 * acceleration * pow(time_s, 2);
}

void get_min_max_travel(const Tag* tag,
                        const float time_s,
                        const float v_max,
                        const float a_max,
                        float& min_travel, 
                        float& max_travel){
    max_travel = v_max * time_s;
    min_travel = -max_travel;
    if (tag->valid_velocity) {
        float tmp_travel = calc_displacement(tag->velocity, time_s, a_max);
        max_travel = min(tmp_travel, max_travel);
        tmp_travel = calc_displacement(tag->velocity, time_s, -a_max);
        min_travel = max(tmp_travel, min_travel);
    }
}

void fast_search(const image_u8_t& im,
                 const ptime latest_frame,
                 const float v_max,
                 const float a_max,
                 const float alpha,
                 const int min_search_dim,
                 const int CAM_NAME,
                 const float time_uncertainty,
                 apriltag_detector_t* detector,
                 zarray_t* detections,
                 Tag* tags_start,
                 bool& use_exhaustive_search,
                 std::ofstream& file_output
                 ) {

    std::mutex detections_mutex; // Protect shared resource
    std::mutex log_mutex;        // Protect log messages

    ptime total_start_time = boost::posix_time::microsec_clock::universal_time();

    auto thread_task = [&](Tag* current_tag) {
        if (!tag_exists(current_tag->x, current_tag->y)) {
            return;
        }

        ptime search_start = boost::posix_time::microsec_clock::universal_time();

        auto elapsed = latest_frame - current_tag->latest_detection;
        float us = elapsed.total_microseconds();
        float time_s = us / 1e6 + time_uncertainty;

        float max_travel, min_travel;
        get_min_max_travel(current_tag, time_s, v_max, a_max, min_travel, max_travel);

        set_search_area(im.width, im.height, min_search_dim, min_travel, max_travel, alpha, *current_tag);

        {
            std::lock_guard<std::mutex> lock(log_mutex);
#if PRINT_DEBUG_MSG
            std::cout << "CAM#" << CAM_NAME << " using PART SEARCH "
                      << current_tag->area.x_length << "x" << current_tag->area.y_length
                      << "\n";
#endif
        }

        {
            std::lock_guard<std::mutex> lock(detections_mutex);
            partial_search(im, current_tag->area, detections, detector);
        }

        if (total_time > DEFAULT_LIMIT_MAX) {

#if PRINT_DEBUG_MSG
            std::cout << "CAM#" << CAM_NAME
                      << " exceeded " << DEFAULT_LIMIT_MAX / 1000 << "ms search time. Switching to exhaustive search.\n";
            use_exhaustive_search = true;
#endif

            return; // Terminate fast search early
        }

    };

    // Process tags with threads
    const size_t num_threads = std::min<size_t>(static_cast<size_t>(MAX_TAG_ID), std::thread::hardware_concurrency());
    std::vector<std::thread> threads;
    for (Tag* tag = tags_start; tag < tags_start + MAX_TAG_ID; tag++) {
        threads.emplace_back(thread_task, tag);
    }

    // Join threads
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}


void fast_search1(const image_u8_t& im,
                 const ptime latest_frame,
                 const float v_max,
                 const float a_max,
                 const float alpha,
                 const int min_search_dim,
                 const int CAM_NAME,
                 const float time_uncertainty,
                 apriltag_detector_t* detector,
                 zarray_t* detections,
                 Tag* tags_start) {

    int total_tag = 0;

    for (Tag* tag = tags_start; tag < tags_start + MAX_TAG_ID; tag++) {
        if (!tag_exists(tag->x, tag->y)){
            // tag was not detected during the previous exhaustive search
            continue;
        }

        total_tag ++;

        ptime search_start = boost::posix_time::microsec_clock::universal_time();


        auto elapsed = latest_frame - tag->latest_detection;
        float us = elapsed.total_microseconds();
        float time_s = us / 1e6 + time_uncertainty;  // time elapsed in seconds
        //Create part image to detect for each tag
        // image_u8_t* im_part = CreatePartImage(Areas[i], im);
        float max_travel;
        float min_travel;
        get_min_max_travel(tag, time_s, v_max, a_max, min_travel, max_travel);
        set_search_area(im.width, im.height, min_search_dim, 
                        min_travel, max_travel, alpha, *tag);
#if PRINT_DEBUG_MSG

        // modified 2025
        // cout <<"CAM#"<<CAM_NAME<<": " << "Using PART SEARCH " 
        //     << tag->area.x_length << "x" << tag->area.y_length 
        //     << " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
        cout <<"CAM#"<<CAM_NAME<<" " << "using PART SEARCH " 
            << tag->area.x_length << "x" << tag->area.y_length 
            << " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
#endif
        partial_search(im, tag->area, detections, detector);


    ptime search_end = boost::posix_time::microsec_clock::universal_time();
    uint32_t search_time = (search_end - search_start).total_milliseconds();

    // modified 2025
    double st = boost::posix_time::milliseconds(search_time).total_microseconds();

    int index = static_cast<int>(tag - tags_start);
            
    std::cout <<"CAM#"<<CAM_NAME<<": " <<"tag#"<< index <<": "<< "FAST SEARCH: search_time: " << st << " microseconds\n";

    }

    std::cout << "total tag: " << total_tag << std::endl;

}

void fast_search2(const image_u8_t& im,
                  const ptime latest_frame,
                  const float v_max,
                  const float a_max,
                  const float alpha,
                  const int min_search_dim,
                  const int CAM_NAME,
                  const float time_uncertainty,
                  apriltag_detector_t* detector,
                  zarray_t* detections,
                  Tag* tags_start,
                  bool& use_exhaustive_search,
                  std::ofstream& file_output) {

    int total_tag = 0;
    ptime total_start_time = boost::posix_time::microsec_clock::universal_time();

    for (Tag* tag = tags_start; tag < tags_start + MAX_TAG_ID; tag++) {
        if (!tag_exists(tag->x, tag->y)) {
            // Tag was not detected during the previous exhaustive search
            continue;
        }

        total_tag++;

        // ptime search_start = boost::posix_time::microsec_clock::universal_time();

        auto elapsed = latest_frame - tag->latest_detection;
        float us = elapsed.total_microseconds();
        float time_s = us / 1e6 + time_uncertainty;  // Time elapsed in seconds

        // file_output << "time s: " << time_s << endl;

        auto initial_start = std::chrono::high_resolution_clock::now();

        float max_travel;
        float min_travel;
        get_min_max_travel(tag, time_s, v_max, a_max, min_travel, max_travel);
        set_search_area(im.width, im.height, min_search_dim, 
                        min_travel, max_travel, alpha, *tag);

#if PRINT_DEBUG_MSG
        // file_output <<"CAM#"<<CAM_NAME<<": " << "Using PART SEARCH " 
        //     << tag->area.x_length << "x" << tag->area.y_length 
        //     << " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
        file_output <<"CAM#"<<CAM_NAME << " using PART SEARCH " 
            << tag->area.x_length << "x" << tag->area.y_length 
            << " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
#endif

        auto initial_end = std::chrono::high_resolution_clock::now();
        auto initial_duration = std::chrono::duration_cast<std::chrono::microseconds>(initial_end - initial_start).count() / 1000.0;

        // file_output << "Execution time for the producer: " << std::fixed << std::setprecision(2) << initial_duration << " ms" << std::endl;

        auto partial_start = std::chrono::high_resolution_clock::now();

        float scaling_f = 0.125; // Scales GUI to fit monitor, higher res needs smaller factor. Use values of 0.5^k as fit 

        // Calculate the size of the search area
        DetectionArea* area = &tag->area;
        double search_area_width = scaling_f * (area->x_end - area->x_start);
        double search_area_height = scaling_f * (area->y_end - area->y_start);
        double search_area_size = search_area_width * search_area_height;

        // file_output << "Search Area Size: " << search_area_size << endl;

        partial_search1(im, tag->area, detections, detector, total_start_time, file_output);

        auto partial_end = std::chrono::high_resolution_clock::now();
        auto partial_duration = std::chrono::duration_cast<std::chrono::microseconds>(partial_end - partial_start).count() / 1000.0;

        ptime search_end = boost::posix_time::microsec_clock::universal_time();
        // uint32_t search_time = (search_end - search_start).total_microseconds();

        int index = static_cast<int>(tag - tags_start);

#if PRINT_DEBUG_MSG
        // file_output <<"CAM#"<<CAM_NAME<<": " <<"tag#"<< index <<": "<< "FAST SEARCH: search_time: " << std::fixed << std::setprecision(2) << partial_duration << " milliseconds\n";
        file_output << "CAM#" << CAM_NAME << " tag#"<< index <<
            " FAST SEARCH time: " << std::fixed << std::setprecision(2) << partial_duration << " ms\n";
#endif
        uint32_t total_time = (search_end - total_start_time).total_microseconds();
        // cout << "fast_search2 time: " << total_time << endl;

        if (total_time > DEFAULT_LIMIT_MAX) {

#if PRINT_DEBUG_MSG
            file_output << "CAM#" << CAM_NAME << " tag#"<< index <<
                " exceeded " << DEFAULT_LIMIT_MAX / 1000 << "ms search time. Switching to exhaustive search.\n";
#endif

            use_exhaustive_search = true;
            break; // Terminate fast search early
        }
    }
}



float calc_velocity(const int old_x, const int old_y, 
                    const int new_x, const int new_y,
                    const ptime old_frame,
                    const ptime new_frame) {
    float dy = new_y - old_y;
    float dx = new_x - old_x;
    float diag = sqrt(powf(dx, 2) + powf(dy, 2));
    auto elapsed = new_frame - old_frame;
    auto dt = (elapsed).total_microseconds();
    return diag / (dt / 1e6f);
}


/**
 * Updates GUI to display search areas.
*/
void update_gui(const zarray_t* detections, Tag* tags_start, cv::Mat& frame, float avg_hz, std::ofstream& file_output) {

    float scaling_f = 0.125; // Scales GUI to fit monitor, higher res needs smaller factor. Use values of 0.5^k as fit 

    std:: ostringstream ss;
    ss << avg_hz;

    cv::resize(frame, frame, cv::Size(), scaling_f, scaling_f, cv::INTER_LINEAR);

    putText(frame, ss.str(),
        cv::Point(30, 30),
        cv::FONT_HERSHEY_PLAIN,
        2, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

    if (zarray_size(detections) == 0) {
        string idToText = "---Nothing Detected---";
        putText(frame, idToText,
                cv::Point(30, 30),
                cv::FONT_HERSHEY_PLAIN,
                1.5, cv::Scalar(180, 250, 0), 1, cv::LINE_AA);

        return;
    }
    // Get time of frame/detection----------------
    //show = family.superimposeDetections(frame, detections); //-- Used to actually
    //superimpose tag image in video
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *dd;
        zarray_get(detections, i, &dd);
        Tag* tag = tags_start + dd->id;
        DetectionArea* area = &tag->area;
        // Draw green square around the search area
        cv::rectangle(frame,
                      cv::Point(scaling_f*area->x_start, scaling_f*area->y_start),
                      cv::Point(scaling_f*area->x_end, scaling_f*area->y_end),
                      cv::Scalar(0, 255, 0));
        // Print out Tag ID in center of Tag
        putText(frame, std::to_string(dd->id), cv::Point(scaling_f*tag->x, scaling_f*tag->y),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 250, 0), 2,
                cv::LINE_AA);

    // Calculate the size of the AprilTag in pixels
        double apriltag_size = 0.0;
        for (int j = 0; j < 4; j++) {
            int next = (j + 1) % 4;
            double dx = dd->p[next][0] - dd->p[j][0];
            double dy = dd->p[next][1] - dd->p[j][1];
            apriltag_size += sqrt(dx * dx + dy * dy); // Sum edge lengths
        }
        apriltag_size /= 4.0; // Average size of edges

        // Calculate the size of the search area
        double search_area_width = scaling_f * (area->x_end - area->x_start);
        double search_area_height = scaling_f * (area->y_end - area->y_start);
        double search_area_size = search_area_width * search_area_height;

        // Calculate the ratio of search area size to AprilTag size
        double ratio = search_area_size / apriltag_size;

        // Print out the ratio and related information

#if PRINT_DEBUG_MSG
        file_output << "Tag ID: " << dd->id
                << ", AprilTag Size: " << apriltag_size
                << ", Search Area Size: " << search_area_size
                << ", Ratio: " << ratio << std::endl;
#endif

    }
}

void update_exhaustive_gui(DetectionData detection_data, Tag* tags_start, cv::Mat& frame, float avg_hz) {

    float scaling_f = 0.125; // Scales GUI to fit monitor, higher res needs smaller factor. Use values of 0.5^k as fit 

    std:: ostringstream ss;
    ss << avg_hz;

    cv::resize(frame, frame, cv::Size(), scaling_f, scaling_f, cv::INTER_LINEAR);

    putText(frame, ss.str(),
        cv::Point(30, 30),
        cv::FONT_HERSHEY_PLAIN,
        2, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

    // Get time of frame/detection----------------
    //show = family.superimposeDetections(frame, detections); //-- Used to actually
    //superimpose tag image in video

    bool found = false;

    for (int i = 0; i < 10; i++) 
    if (detection_data.tags[i].found)
    {
        found = true;

        Tag* tag = tags_start + i;
        DetectionArea* area = &detection_data.tag_data[i].area;
        // Draw green square around the search area
        cv::rectangle(frame,
                      cv::Point(scaling_f*area->x_start, scaling_f*area->y_start),
                      cv::Point(scaling_f*area->x_end, scaling_f*area->y_end),
                      cv::Scalar(0, 255, 0));
        // Print out Tag ID in center of Tag
        putText(frame, std::to_string(i), cv::Point(scaling_f*detection_data.tags[i].camera_coords->x, scaling_f*detection_data.tags[i].camera_coords->y),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 250, 0), 2,
                cv::LINE_AA);
    }

    if (!found) {
                string idToText = "---Nothing Detected---";
        putText(frame, idToText,
                cv::Point(30, 30),
                cv::FONT_HERSHEY_PLAIN,
                1.5, cv::Scalar(180, 250, 0), 1, cv::LINE_AA);

        return;
    }

}


void update_tag(const cv::Point2f* detection,
                const cv::Point2f* cornerDetections,
                const ptime latest_frame,
                Tag* tag, std::ofstream& file_output) {
    if (tag_exists(tag->x, tag->y)) {
        tag->velocity = calc_velocity(tag->x, tag->y, 
                                      detection->x, detection->y,
                                      tag->latest_detection, latest_frame);
        tag->valid_velocity = true;

#if PRINT_DEBUG_MSG
        file_output << "tag->x: " << tag->x << " tag->y: " << tag->y << " detection->x: " << detection->x << " detection->y: " << detection->y << endl;
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

void produce_frame(int camera_id, cv::VideoCapture *cap) {

#if BINDING_CPU_CORES
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);

    // bind the thread to the corresponding core
    CPU_SET(camera_id + 8, &cpuset);

    // set the CPU affinity of the thread
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
        std::cerr << "Error: Unable to set CPU affinity for thread " << camera_id << std::endl;
        return;
    }
#endif

    struct sched_param param;
    int policy;

    // Set the priority to the highest (the maximum priority of SCHED_RR)
    param.sched_priority = sched_get_priority_max(SCHED_RR);

    // Set the scheduling policy to SCHED_RR
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &param) != 0) {
        std::cerr << "Error: Unable to set thread scheduling parameters" << std::endl;
        perror("Error details");
        return;
    }

    // Get the scheduling policy and priority of the current thread (confirm whether the setting is successful)
    if (pthread_getschedparam(pthread_self(), &policy, &param) != 0) {
        std::cerr << "Error: Unable to get thread scheduling parameters" << std::endl;
        return;
    }

    std::ostringstream filename;
    filename << "output-producer/camera_" << camera_id << "_output-producer.log";
    std::ofstream file_output(filename.str(), std::ios::out);

    while (true)
    {

        // Get the scheduling policy and priority of the current thread
        if (pthread_getschedparam(pthread_self(), &policy, &param) != 0) {
            std::cerr << "Error: Unable to get thread scheduling parameters" << std::endl;
            break;
        }

#if BINDING_CPU_CORES
        // Get the CPU affinity of the current thread
        if (pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
            std::cerr << "Error: Unable to get CPU affinity for thread" << std::endl;
            break;
        }

        // Find the core where the current thread is running
        for (int i = 0; i < CPU_SETSIZE; ++i) {
            if (CPU_ISSET(i, &cpuset)) {
                file_output << "Core number: " << i << std::endl;
                break;
            }
        }
#endif

        // Start
        auto start = std::chrono::high_resolution_clock::now();

        unsigned int next = (producer_counter[camera_id].load() + 1) % BUFFER_SIZE;

        while(next == fast_consumer_counter[camera_id].load() && next == nice_consumer_counter[camera_id].load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        *cap >> buffer[camera_id][next];
        producer_counter[camera_id] = next;

        // End measurement
        auto end = std::chrono::high_resolution_clock::now();

        // Calculation time (in microseconds)
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;

#if PRINT_DEBUG_MSG
        // printing time
        file_output << "Execution time for the producer: " << std::fixed << std::setprecision(2) << duration << " ms" << std::endl;
#endif
        // added 2025
        // Increment the shared frame counter
        // shared_frame_count++;
    }

    file_output.close();

}


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

int nice_consume_frame(int camera_id, boost::interprocess::named_semaphore& sem, boost::interprocess::named_semaphore& sem_1, char* shared_memory, SharedData *ptr, cv::Mat frame, cv::Mat gray, int CAM_NAME, cv::Mat map1, cv::Mat map2, GulliViewOptions opts, std::string win, DebugLogger& nice_thread_logger) {

    std::ostringstream filename;
    filename << "output-nice/camera_" << camera_id << "_output-nice.log";
    std::ofstream file_output(filename.str(), std::ios::out);

    // pthread_t current_thread = pthread_self();

    // struct sched_param sched_param;
    // sched_param.sched_priority = 1; // Set priority to a nice value (higher = higher priority)

#if BINDING_CPU_CORES
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);

    // Bind the thread to the corresponding core
    CPU_SET(camera_id, &cpuset);

    // Set the CPU affinity of the thread
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
        std::cerr << "Error: Unable to set CPU affinity for thread " << camera_id << std::endl;
        return -1;
    }
#endif

    struct sched_param sched_param;
    int policy;

    // Get the scheduling policy and priority of the current thread
    if (pthread_getschedparam(pthread_self(), &policy, &sched_param) != 0) {
        std::cerr << "Error: Unable to get thread scheduling parameters" << std::endl;
        return -1;
    }

    // std::cout << "Before setting, Thread priority: " << sched_param.sched_priority << std::endl;

    // Get the lowest priority of the SCHED_RR policy
    int min_priority = sched_get_priority_min(SCHED_RR);
    sched_param.sched_priority = min_priority;  // Set to lowest priority

    // Set the scheduling policy to SCHED_RR
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &sched_param) != 0) {
        std::cerr << "Error: Unable to set thread scheduling parameters" << std::endl;
        perror("Error details");
        return -1;
    }

    // Get the scheduling policy and priority of the current thread (confirm whether the setting is successful)
    if (pthread_getschedparam(pthread_self(), &policy, &sched_param) != 0) {
        std::cerr << "Error: Unable to get thread scheduling parameters" << std::endl;
        return -1;
    }

    // file_output << "After setting, Thread priority: " << sched_param.sched_priority << std::endl;

    auto start = std::chrono::system_clock::now().time_since_epoch();
    auto start_ms = std::chrono::duration_cast<std::chrono::milliseconds>(start).count();
    std::queue<Message> messageQueue; // added 2024


    Tag tags[MAX_TAG_ID];

    TagFamily family(opts.family_str);
    apriltag_detector_t* detector = apriltag_detector_create();
    apriltag_detector_add_family(detector, family.at_family);
    // apriltag_family_t *tf = tagStandard41h12_create(); // added 2025

    // apriltag_detector_add_family(detector, tf);

    detector->nthreads = 16;
    detector->quad_decimate = 1.0f; // 
    detector->quad_sigma = 0.6f; // Low-pass blur, negative values sharpen
    detector->refine_edges = 1; 

    // detector->quad_decimate = 2.0f; // Downsampling to increase speed
    // detector->quad_sigma = 0.0f; // Turn off blurring to reduce calculations
    // detector->refine_edges = 0; // Turn off edge refinement

    uint32_t seq = 0;
    int global_search_counter = 0;
    int loop_count = 0;
    int hz_counter = 0;
    int tot_hz = 0;
    int avg_hz = 0;
    int sum_hz = 0;

    while (true) {

        // Get the scheduling policy and priority of the current thread
        if (pthread_getschedparam(pthread_self(), &policy, &sched_param) != 0) {
            std::cerr << "Error: Unable to get thread scheduling parameters" << std::endl;
            break;
        }



#if BINDING_CPU_CORES
        // Get the CPU affinity of the current thread
        if (pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
            std::cerr << "Error: Unable to get CPU affinity for thread" << std::endl;
            break;
        }

        // Find the core on which the current thread is running
        for (int i = 0; i < CPU_SETSIZE; ++i) {
            if (CPU_ISSET(i, &cpuset)) {
                file_output << "Core number: " << i << std::endl;
                break;
            }
        }
#endif

        // Start measurement
        auto loop_start = std::chrono::high_resolution_clock::now();


        float avg_time_gap = -1;
        auto while_start = std::chrono::system_clock::now().time_since_epoch();
        auto while_start_ms = std::chrono::duration_cast<std::chrono::milliseconds>(while_start).count();




        auto detectionTime = std::chrono::system_clock::now().time_since_epoch();
        uint64_t detectionTime_ms = std::chrono::duration_cast<std::chrono::milliseconds>(detectionTime).count();

        ptime import_start = boost::posix_time::microsec_clock::universal_time();

#if PRODUCE_FRAME_MODE == 1 || PRODUCE_FRAME_MODE == 3

        // Start measurement
        auto consumer_start = std::chrono::high_resolution_clock::now();

        while(nice_consumer_counter[camera_id].load() == producer_counter[camera_id].load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        nice_consumer_counter[camera_id]=producer_counter[camera_id].load();
        frame = buffer[camera_id][nice_consumer_counter[camera_id].load()];

        // End measurement
        auto consumer_end = std::chrono::high_resolution_clock::now();

        // Calculation time (in microseconds)
        auto consumer_duration = std::chrono::duration_cast<std::chrono::microseconds>(consumer_end - consumer_start).count();

#if PRINT_DEBUG_MSG
        // printing time
        file_output << "Execution time for the consumer: " << std::fixed << std::setprecision(2) << consumer_duration / 1000.0 << " ms" << std::endl;
#endif

        nice_thread_logger.log_operation(DebugLogger::CONSUMER_TIME, consumer_duration, nice_consumer_counter[camera_id].load());


        ptime transform_start = boost::posix_time::microsec_clock::universal_time();
        bool frame_captured = transform_frame(frame, gray, map1, map2, file_output);

        if (!frame_captured) {
            cout << "no frame captured (nice), exiting. Camera " << camera_id << endl;
            return -1;
        }

        image_u8_t im = {
                gray.cols,
                gray.rows,
                gray.cols,
                gray.data
        };

        // image_u8_t im = {
        //     .width = frame.cols,
        //     .height = frame.rows,
        //     .stride = frame.cols,
        //     .buf = frame.data
        // };

        ptime transform_end = boost::posix_time::microsec_clock::universal_time();
        uint32_t transform_time = (transform_end - transform_start).total_milliseconds();
        //	std::cout << transform_time << std::endl;

#elif PRODUCE_FRAME_MODE == 2

        ptime latest_frame = boost::posix_time::microsec_clock::universal_time();

        BufferData data = lock_buffer->consume_nice(); // Read from buffer
        frame = data.frame;
        // gray = data.gray;

        bool frame_captured = transform_frame1(frame, gray, map1, map2, file_output);

        if (!frame_captured) {
            cout << "no frame captured, exiting" << endl;
            return -1;
        }


        image_u8_t im = {
                gray.cols,
                gray.rows,
                gray.cols,
                gray.data
        };

#else
    #error "Unsupported mode"
#endif

        nice_thread_logger.log_operation(DebugLogger::TRANSFORM_TIME, transform_time, nice_consumer_counter[camera_id].load());

        // ptime search_start = boost::posix_time::microsec_clock::universal_time();

        // zarray_t *detections = apriltag_detector_detect(detector, &im);
        zarray_t *detections = zarray_create(sizeof(apriltag_detection_t*)); //2023: from FastSearch-code

        // add 2025

        // Start time measurement
        // ptime search_start1 = boost::posix_time::microsec_clock::universal_time();

        // Initialize frame count
        uint32_t frame_count = 0;

        // Before search
        uint32_t frames_before_search = shared_frame_count.load();

        //Use exhaustive search

        loop_count = 1;
        global_search_counter = 0;
        //Clear previous coordinates of all tags.
        for (Tag* tag = tags; tag < tags + MAX_TAG_ID; tag++) {
            reset_tag(im.width, im.height, tag);
        }

        auto search_start = std::chrono::high_resolution_clock::now();

        ptime latest_frame = boost::posix_time::microsec_clock::universal_time();

        uint32_t import_time = (latest_frame - import_start).total_milliseconds();
            
        detections = exhaustive_search(im, detector);

        auto search_end = std::chrono::high_resolution_clock::now();   // End measurement
        double search_time = std::chrono::duration_cast<std::chrono::microseconds>(search_end - search_start).count();

        DetectionData detection_data;


#if PRINT_DEBUG_MSG

        // modified 2025
        // double st = boost::posix_time::milliseconds(search_time).total_microseconds();
            
        // file_output <<"CAM#"<<CAM_NAME<<": "<< "GLOBAL SEARCH: search_time: " << std::fixed << std::setprecision(2) << search_time / 1000.0 << " ms\n";
        file_output <<"CAM#"<<CAM_NAME<< " GLOBAL SEARCH time: " << std::fixed << std::setprecision(2) << search_time / 1000.0 << " ms\n";

#endif

        nice_thread_logger.log_operation(DebugLogger::GLOBAL_SEARCH_TIME, search_time, nice_consumer_counter[camera_id].load());


        if (zarray_size(detections) != 0) {
            // Get time of frame/detection----------------

            size_t index = 0;
            Message buf {
                htobe32(1) /* type */ ,
                htobe32(2) /* subtype */,
                htobe32(seq) /* seq */,
                // added 2024
                htobe64(detectionTime_ms), /* detection timestamp */
                htobe64(avg_time_gap) /* avg time gap */
                // htobe64(msecs) /* time_msec */   // commented out 2024
            };
            // Camera coordinates for tag center.
            vector<at::Point> camera_detections(zarray_size(detections)); 
            // Camera coordinates for tag corners.
            vector<at::Point> camera_corner_detections(2*zarray_size(detections)); 

            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *dd;
                zarray_get(detections, i, &dd);
                Tag* tag = tags + dd->id;
                camera_detections[i] = at::Point(dd->c[0], dd->c[1]);
                camera_corner_detections[2*i] = at::Point(dd->p[0][0], dd->p[0][1]);
                camera_corner_detections[2*i+1] = at::Point(dd->p[3][0], dd->p[3][1]);
                // Aron: Adjust for part image coordinates?
                camera_detections[i].x += tag->area.x_start;
                camera_detections[i].y += tag->area.y_start;
// #if PRINT_DEBUG_MSG           
//                     std::cout <<"CAM#"<<CAM_NAME<<": " 
//                               << "Found when using GLOBAL_IMAGE. X: " 
//                               << camera_detections[i].x << " Y:" 
//                               << camera_detections[i].y << endl;

// #endif         
            }
            // Room coordinates for tag center.
            vector<at::Point> room_detections(zarray_size(detections)); 
            // Room coordinates for tag corner.
            vector<at::Point> room_corner_detections(2*zarray_size(detections)); 

            
            static ptime epoch(boost::gregorian::date(1970,1,1));
            uint64_t msecs = (import_start - epoch).total_milliseconds();

            buf.cam_id = htobe32(CAM_NAME);
            int n_detections = zarray_size(detections);
            std::vector<cv::Point2f> points;

            for (int i = 0; i < n_detections; i++) {
                apriltag_detection_t *dd;
                zarray_get(detections, i, &dd);
                Tag* tag = tags + dd->id;

                cv::Point2f* cornerDetection = 2*i + room_corner_detections.data();
                cv::Point2f* detection = i + camera_detections.data();
                update_tag(detection, cornerDetection, latest_frame, tag, file_output);
                detection = i + room_detections.data();
                add_detection_to_msg(dd->id, detectionTime_ms, tag->x, tag->y, 
                                     tag->theta, i, CAM_NAME, buf);   // added 2024, "detectionTime_ms" added


                detection_data.tags[dd->id].found = true;
                // detection_data.tags[dd->id].area = tag->area;
                detection_data.tags[dd->id].camera_coords = DetectionData::CameraCoordinates{tag->x, tag->y, tag->theta};

             // First create an apriltag_detection_info_t struct using your known parameters.
                apriltag_detection_info_t info;
                info.det = dd;
                info.tagsize = opts.tag_size;

#if PRINT_DEBUG_MSG
                file_output << "Tag size of opts: " << opts.tag_size << endl;
#endif

                // Define the parameter table corresponding to the device
                struct DeviceInfo {
                    double fx_2k, fx_4k;
                    double fy_2k, fy_4k;
                    double cx_2k, cx_4k;
                    double cy_2k, cy_4k;
                };

                static const DeviceInfo device_params[] = {
                    {1135.208474, 2270.416948, 1133.531325, 2267.062650, 998.932805, 1997.865610, 530.084624, 1060.169248}, // Device 1
                    {1146.495853, 2290.512606, 1146.011083, 2283.316632, 953.976032, 1923.229236, 571.868765, 1028.981542}, // Device 2
                    {1145.256303, 2292.991706, 1141.658316, 2292.022166, 961.614618, 1907.952064, 514.490771, 1143.737530}, // Device 3
                    {1148.845354, 2297.690708, 1136.968276, 2273.936552, 987.148727, 1974.297454, 523.721356, 1047.443252}  // Device 4
                };

                if (opts.device_num >= 1 && opts.device_num <= 4) {
                    const DeviceInfo& params = device_params[opts.device_num - 1];

                    if (opts.frame_width == 3840 && opts.frame_height == 2160) { // 4K
                        info.fx = params.fx_4k;
                        info.fy = params.fy_4k;
                        info.cx = params.cx_4k;
                        info.cy = params.cy_4k;
                    } else if (opts.frame_width == 1920 && opts.frame_height == 1080) { // 2K
                        info.fx = params.fx_2k;
                        info.fy = params.fy_2k;
                        info.cx = params.cx_2k;
                        info.cy = params.cy_2k;
                    } else {
                        // Default value or error handling
                        throw std::invalid_argument("Unsupported resolution");
                    }
                } else {
                    throw std::invalid_argument("Unsupported device number");
                }



                // Then call estimate_tag_pose.
                apriltag_pose_t pose;
                double err = estimate_tag_pose(&info, &pose);
                // Do something with pose.
                
                // Now, pose.t should contain the translation vector (x, y, z)
                // if (pose.t) {
                //     // Assuming pose.t is a pointer to a matd_t structure representing a 3x1 translation vector
                //     double x = pose.t->data[0];  // The x-coordinate
                //     double y = pose.t->data[1];  // The y-coordinate
                //     double z = pose.t->data[2];  // The z-coordinate
                    
                //     // Print the coordinates

                //     detection_data.tags[dd->id].space_coords = DetectionData::SpaceCoordinates{x, y, z};


                // }


            }

            buf.length = htobe32(n_detections);

            detection_data.storeMessage(buf);

        }


        auto copy_start = std::chrono::high_resolution_clock::now();

        std::copy(std::begin(tags), std::end(tags), detection_data.tag_data);


        // End measurement
        auto copy_end = std::chrono::high_resolution_clock::now();

        // Calculation time (in microseconds)
        auto copy_duration = std::chrono::duration_cast<std::chrono::microseconds>(copy_end - copy_start).count();

        // std::cout << "Execution time for the copy: " << copy_duration << " microseconds" << std::endl;

        auto start = std::chrono::high_resolution_clock::now();

        unsigned int search_next = (search_producer_counter[camera_id].load() + 1) % BUFFER_SIZE;


        // print_counters(camera_id, file_output);

        while(search_next == search_consumer_counter[camera_id].load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        search_buffer[camera_id][search_next] = detection_data;
        search_producer_counter[camera_id] = search_next;




        // End measurement
        auto end = std::chrono::high_resolution_clock::now();

        // Calculation time (in microseconds)
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

#if PRINT_DEBUG_MSG
        // printing time
        file_output << "Execution time for the producer: " << std::fixed << std::setprecision(2) << duration / 1000.0 << " ms" << std::endl;
#endif

        nice_thread_logger.log_operation(DebugLogger::PROCESS_TIME, duration, nice_consumer_counter[camera_id].load());

        //If there are tags missing, use exhaustive search next time

        if (sig_stop) {
            break;
        }
        apriltag_detections_destroy(detections);

        loop_count++;

            // End measurement
    auto loop_end = std::chrono::high_resolution_clock::now();

    // Calculation time (in microseconds)
    auto loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start).count();

#if PRINT_DEBUG_MSG
    // printing time
    file_output << "Execution time for one loop: " << std::fixed << std::setprecision(2) << loop_duration / 1000.0 << " ms" << std::endl;
#endif

    nice_thread_logger.log_operation(DebugLogger::LOOP_TIME, loop_duration, nice_consumer_counter[camera_id].load());

    nice_thread_logger.write_to_file_if_needed(loop_duration, "nice_thread", filename.str());


    }

    // tagStandard41h12_destroy(tf);

    apriltag_detector_destroy(detector);

    file_output.close();

    return 0;
}

int fast_consume_frame(int camera_id, boost::interprocess::named_semaphore& sem, boost::interprocess::named_semaphore& sem_1, char* shared_memory, SharedData *ptr, cv::Mat frame, cv::Mat gray, int CAM_NAME, cv::Mat map1, cv::Mat map2, GulliViewOptions opts, std::string win, DebugLogger& fast_thread_logger) {

#if BINDING_CPU_CORES
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);

    // bind the thread to the corresponding core
    CPU_SET(camera_id + 4, &cpuset);

    // set the CPU affinity of the thread
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
        std::cerr << "Error: Unable to set CPU affinity for thread " << camera_id << std::endl;
        return -1;
    }
#endif

    struct sched_param param;
    int policy;

    // set the priority to the highest (the maximum priority of SCHED_RR)
    param.sched_priority = sched_get_priority_max(SCHED_RR);

    // set the scheduling policy to SCHED_RR
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &param) != 0) {
        std::cerr << "Error: Unable to set thread scheduling parameters" << std::endl;
        perror("Error details");
        return -1;
    }

    // get the scheduling policy and priority of the current thread
    if (pthread_getschedparam(pthread_self(), &policy, &param) != 0) {
        std::cerr << "Error: Unable to get thread scheduling parameters" << std::endl;
        return -1;
    }

    std::ostringstream filename;
    filename << "output/camera_" << camera_id << "_output-fast.log";
    std::ofstream file_output(filename.str(), std::ios::out);

    // file_output << "CAM_NAME " << CAM_NAME << ": camera id: " << camera_id << endl;
    // #if BINDING_CPU_CORES
    //     file_output << "USING BINDING CPU CORES" << endl;
    // #endif

    auto start = std::chrono::system_clock::now().time_since_epoch();
    auto start_ms = std::chrono::duration_cast<std::chrono::milliseconds>(start).count();
    std::queue<Message> messageQueue; // added 2024

    Tag tags[MAX_TAG_ID];

    Tag previous_tags[MAX_TAG_ID];

    TagFamily family(opts.family_str);
    apriltag_detector_t* detector = apriltag_detector_create();

    // apriltag_family_t *tf = tagStandard41h12_create(); // added 2025

    apriltag_detector_add_family(detector, family.at_family);

    // apriltag_detector_add_family(detector, tf);

    detector->nthreads = 16;
    detector->quad_decimate = 1.0f; // 
    detector->quad_sigma = 0.6f; // Low-pass blur, negative values sharpen
    detector->refine_edges = 1; 

    // detector->quad_decimate = 2.0f; // Downsampling to increase speed
    // detector->quad_sigma = 0.0f; // Turn off blurring to reduce calculations
    // detector->refine_edges = 0; // Turn off edge refinement

    const float time_uncertainty = get_uncertainty();
    bool use_exhaustive_search = true;
    const float pixels_per_meter = opts.frame_width / ROOM_WIDTH_METER;
    const float tag_diag_m = 0.428f;
    float a_max = opts.acceleration_max * pixels_per_meter;
    float v_max = opts.velocity_max * pixels_per_meter;
    const int min_search_dim = ceil(0.5 * tag_diag_m * pixels_per_meter);   // min dimension find a tag at any angle. 92 is diagonal of 36h11 tag, converted to pixels.
                                     // halved to account for search area creation
    float alpha = M_PI / 12; // modified 2025


    // cout << a_max << " " << v_max << " " << min_search_dim << " " << alpha << endl;

    uint32_t seq = 0;
    int global_search_counter = 0;
    int loop_count = 0;
    int hz_counter = 0;
    int tot_hz = 0;
    int avg_hz = 0;
    int sum_hz = 0;

    AngleTracker angle_tracker;
    AccelerationTracker acceleration_tracker;

    // double max_duration = 0.0;
    // double avg_duration = 0.0;
    // double max_search = 0.0;
    // double avg_search = 0.0;

    // file_output << "Current angle: " << alpha << endl;
    // file_output << "Current speed: " << v_max << endl;

    // Mapping of camera and buffer
    CyclicBuffer* produce_buffers[2] = {nullptr, nullptr};
    CyclicBuffer* consume_buffers[2] = {nullptr, nullptr};

    if (camera_id == 0) {
        produce_buffers[0] = &buffer_01;
        consume_buffers[0] = &buffer_10;
    } else if (camera_id == 1) {
        produce_buffers[0] = &buffer_10;
        produce_buffers[1] = &buffer_12;
        consume_buffers[0] = &buffer_01;
        consume_buffers[1] = &buffer_21;
    } else if (camera_id == 2) {
        produce_buffers[0] = &buffer_21;
        produce_buffers[1] = &buffer_23;
        consume_buffers[0] = &buffer_12;
        consume_buffers[1] = &buffer_32;
    } else if (camera_id == 3) {
        produce_buffers[0] = &buffer_32;
        consume_buffers[0] = &buffer_23;
    }


    const int epsilon_max = 6;
    const int epsilon_min = 1;
    // int epsilon = 5;

    // for (int epsilon = epsilon_max; epsilon >= epsilon_min; epsilon--) {
    for (int trial = 1; trial <= 1; trial++) {

    int total_loop_count = 0;

    while (true) {


    // Message buf;

    // // Setting max values for buf fields
    // buf.type = htobe32(0xFFFFFFFF);               // Max value for 32-bit unsigned (0xFFFFFFFF)
    // buf.subtype = htobe32(0xFFFFFFFF);            // Max value for 32-bit unsigned (0xFFFFFFFF)
    // buf.seq = htobe32(0xFFFFFFFF);                // Max value for 32-bit unsigned (0xFFFFFFFF)
    // buf.time_msec = htobe64(0xFFFFFFFFFFFFFFFF);  // Max value for 64-bit unsigned (0xFFFFFFFFFFFFFFFF)
    // buf.avg_time_gap = htobe64(0xFFFFFFFFFFFFFFFF); // Max value for 64-bit unsigned (0xFFFFFFFFFFFFFFFF)
    // buf.cam_id = htobe32(camera_id);            // Max value for 32-bit unsigned (0xFFFFFFFF)
    // buf.length = htobe32(0xFFFFFFFF);            // Max value for 32-bit unsigned (0xFFFFFFFF)

    // // Initialize detections array with maximum values
    // for (int i = 0; i < 11; i++) {
    //     buf.detections[i]= {
    //         htobe32(i),                // Max 32-bit unsigned value
    //         htobe64(0xFFFFFFFFFFFFFFFF), // Max 64-bit unsigned value
    //         htobe32(0xFFFFFFFF),           // Max 32-bit unsigned value
    //         htobe32(0xFFFFFFFF),           // Max 32-bit unsigned value
    //         FLT_MAX,                          // Max float value (FLT_MAX)
    //         FLT_MAX,                          // Max float value (FLT_MAX)
    //         htobe32(camera_id)            // Max 32-bit unsigned value
    //     };
    // }

    // for(int i = 0; i<100; i++){
    //     if(sem_1.try_wait()){
    //         break;
    //     }
    //     usleep(1);
    // }
    // memcpy(&(ptr->msg), &buf, sizeof(buf));
    // ptr->flag = 1;
    // sem_1.post();

        // Get the scheduling policy and priority of the current thread
        if (pthread_getschedparam(pthread_self(), &policy, &param) != 0) {
            std::cerr << "Error: Unable to get thread scheduling parameters" << std::endl;
            break;
        }


#if BINDING_CPU_CORES
        // Get the CPU affinity of the current thread
        if (pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
            std::cerr << "Error: Unable to get CPU affinity for thread" << std::endl;
            break;
        }

        // Find the core where the current thread is running
        for (int i = 0; i < CPU_SETSIZE; ++i) {
            if (CPU_ISSET(i, &cpuset)) {
                file_output << "Core number: " << i << std::endl;
                break;
            }
        }
#endif

        total_loop_count++;

        // Start measurement
        auto loop_start = std::chrono::high_resolution_clock::now();


        float avg_time_gap = -1;
        auto while_start = std::chrono::system_clock::now().time_since_epoch();
        auto while_start_ms = std::chrono::duration_cast<std::chrono::milliseconds>(while_start).count();

#if USE_MEMORY_SHARING

        // added 2024
        //shared memory start
        
        sem.wait();

        std::stringstream ss(shared_memory);
        std::string detection_str = ss.str();
        sem.post();
            }

        std::regex tuple_regx("\\(([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*)\\)");

        std::sregex_iterator it(detection_str.begin(), detection_str.end(), tuple_regx);
        std::sregex_iterator end;
        auto current_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        try {
            float bot_num = 0;remap
            float tim_gap = 0;
            for (; it != end; ++it) {
                std::string first_element = (*it)[1].str();
                std::string second_element = (*it)[2].str();
                std::string third_element = (*it)[3].str();
                std::string fourth_element = (*it)[4].str();
                std::string fifth_element = (*it)[5].str();
                std::string sixth_element = (*it)[6].str();
                std::string seventh_element = (*it)[7].str();
                int id = std::stoi(first_element);
                long long time_msec = std::stoll(second_element);
                int x = std::stoi(third_element);
                int y = std::stoi(fourth_element);
                float theta = std::stof(fifth_element);
                float speed = std::stof(sixth_element);
                int camera_id = std::stoi(seventh_element);
                long diffrence = current_time - time_msec;
                if (id != -1) {
                    bot_num = bot_num + 1;
                    tim_gap = tim_gap + diffrence;
                    //std::cout << "recived id: " << id << ", recived_time: " << time_msec << ", recived_x: " << x << ", recived_y: " << y << ", recived_theta: " << theta << ", recived_speed: " << speed << ", recived_camera_id: " << camera_id << ", gap: " << diffrence << std::endl;
                }
            }
            if (bot_num != 0) {
                avg_time_gap = tim_gap / bot_num;
                //std::cout << "Number of bots detected: " << bot_num << std::endl;
                //std::cout << " time_gap: " << tim_gap << std::endl;
            } else {
                avg_time_gap = -1;
            }
            if (std::isnan(avg_time_gap)) {
                avg_time_gap = -1;
            }
            //std::cout << "AVG_Time_gap: " << avg_time_gap << std::endl;
            //std::cout << "Time_gap: " << tim_gap << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Exception caught: " << e.what() << '\n';
            avg_time_gap = -1;
        }
        //std::cout <<avg_time_gap << std::endl;

#endif

        auto detectionTime = std::chrono::system_clock::now().time_since_epoch();
        uint64_t detectionTime_ms = std::chrono::duration_cast<std::chrono::milliseconds>(detectionTime).count();

        ptime import_start = boost::posix_time::microsec_clock::universal_time();



        auto init_start = std::chrono::high_resolution_clock::now();

        // Start measurement
        auto consumer_start = std::chrono::high_resolution_clock::now();

        while(fast_consumer_counter[camera_id].load() == producer_counter[camera_id].load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        fast_consumer_counter[camera_id]=producer_counter[camera_id].load();
        frame = buffer[camera_id][fast_consumer_counter[camera_id].load()];

        // End measurement
        auto consumer_end = std::chrono::high_resolution_clock::now();

        // Calculation time (in microseconds)
        auto consumer_duration = std::chrono::duration_cast<std::chrono::microseconds>(consumer_end - consumer_start).count();

#if PRINT_DEBUG_MSG
        // printing time
        file_output << "Execution time for the consumer: " << std::fixed << std::setprecision(2) << consumer_duration / 1000.0 << " ms" << std::endl;
#endif

        fast_thread_logger.log_operation(DebugLogger::CONSUMER_TIME, consumer_duration, fast_consumer_counter[camera_id].load());

        // ptime transform_start = boost::posix_time::microsec_clock::universal_time();
        
        auto transform_start = std::chrono::high_resolution_clock::now();        

        bool frame_captured = transform_frame(frame, gray, map1, map2, file_output);

        if (!frame_captured) {
            cout << "no frame captured (fast), exiting. Camera " << camera_id << endl;
            // exit(1);
            return -1;
        }

        image_u8_t im = {
                gray.cols,
                gray.rows,
                gray.cols,
                gray.data
        };

        // image_u8_t im = {
        //     .width = frame.cols,
        //     .height = frame.rows,
        //     .stride = frame.cols,
        //     .buf = frame.data
        // };

        // ptime transform_end = boost::posix_time::microsec_clock::universal_time();
        // uint32_t transform_time = (transform_end - transform_start).total_milliseconds();
        
        auto transform_end = std::chrono::high_resolution_clock::now();

        // Calculation time (in microseconds)
        auto transform_time = std::chrono::duration_cast<std::chrono::microseconds>(transform_end - transform_start).count();        
        
        // file_output << "Transform time: " << std::fixed << std::setprecision(2) << transform_time / 1000.0 << " ms" << endl;

        fast_thread_logger.log_operation(DebugLogger::TRANSFORM_TIME, transform_time, fast_consumer_counter[camera_id].load());



        //	std::cout << transform_time << std::endl;
        // ptime search_start = boost::posix_time::microsec_clock::universal_time();

        // zarray_t *detections = apriltag_detector_detect(detector, &im);
        zarray_t *detections = zarray_create(sizeof(apriltag_detection_t*)); //2023: from FastSearch-code

            
        global_search_counter++;


        use_exhaustive_search = false;

        auto init_end = std::chrono::high_resolution_clock::now();

        // Calculation time (in microseconds)
        auto init_duration = std::chrono::duration_cast<std::chrono::microseconds>(init_end - init_start).count();

        // file_output << "Initialize time: " << std::fixed << std::setprecision(2) << init_duration / 1000.0 << " ms" << endl;

        fast_thread_logger.log_operation(DebugLogger::INITIALIZE_TIME, init_duration, fast_consumer_counter[camera_id].load());

        auto search_start = std::chrono::high_resolution_clock::now();

        // file_output << "Current angle: " << alpha << endl;
        // file_output << "Current acceleration: " << a_max << endl;

        std::copy(std::begin(tags), std::end(tags), previous_tags);

        ptime latest_frame = boost::posix_time::microsec_clock::universal_time();

        uint32_t import_time = (latest_frame - import_start).total_milliseconds();

        fast_search2(im, latest_frame, v_max, a_max, alpha,
                        min_search_dim, CAM_NAME, time_uncertainty, detector,
                        detections, tags, use_exhaustive_search, file_output);

        auto search_end = std::chrono::high_resolution_clock::now();   // End measurement
        double search_time = std::chrono::duration_cast<std::chrono::microseconds>(search_end - search_start).count();


#if PRINT_DEBUG_MSG

        // modified 2025
        // double st = boost::posix_time::milliseconds(search_time).total_microseconds();

        // file_output <<"CAM#"<<CAM_NAME<<": "<< "PART SEARCH: search_time: " << std::fixed << std::setprecision(2) << search_time / 1000.0 << " ms\n";
        file_output <<"CAM#"<<CAM_NAME<<" "<< "PART SEARCH time: " << std::fixed << std::setprecision(2) << search_time / 1000.0 << " ms\n";

#endif

        fast_thread_logger.log_operation(DebugLogger::PART_SEARCH_TIME, search_time, fast_consumer_counter[camera_id].load());

        // if (!have_exhaustive_searched && zarray_size(detections) != 0) {

        auto process_start = std::chrono::high_resolution_clock::now();

        if (!use_exhaustive_search && zarray_size(detections) != 0) {

            // Get time of frame/detection----------------

            size_t index = 0;
            Message buf {
                htobe32(1) /* type */ ,
                htobe32(2) /* subtype */,
                htobe32(seq) /* seq */,
                // added 2024
                htobe64(detectionTime_ms), /* detection timestamp */
                htobe64(avg_time_gap) /* avg time gap */
                // htobe64(msecs) /* time_msec */   // commented out 2024
            };
            // Camera coordinates for tag center.
            vector<at::Point> camera_detections(zarray_size(detections)); 
            // Camera coordinates for tag corners.
            vector<at::Point> camera_corner_detections(2*zarray_size(detections)); 

            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *dd;
                zarray_get(detections, i, &dd);
                Tag* tag = tags + dd->id;
                camera_detections[i] = at::Point(dd->c[0], dd->c[1]);
                camera_corner_detections[2*i] = at::Point(dd->p[0][0], dd->p[0][1]);
                camera_corner_detections[2*i+1] = at::Point(dd->p[3][0], dd->p[3][1]);
                // Aron: Adjust for part image coordinates?
                camera_detections[i].x += tag->area.x_start;
                camera_detections[i].y += tag->area.y_start;
#if PRINT_DEBUG_MSG           

                    // file_output <<"CAM#"<<CAM_NAME<<": " 
                    //           << "Found when using PART_IMAGE. X: " 
                    //           << camera_detections[i].x << " Y:" 
                    //           << camera_detections[i].y << endl;
                    file_output <<"CAM#"<<CAM_NAME<<" " 
                              << "Found when using PART_IMAGE. X = " 
                              << camera_detections[i].x << ", Y = " 
                              << camera_detections[i].y << endl;
#endif         
            }
            // Room coordinates for tag center.
            vector<at::Point> room_detections(zarray_size(detections)); 
            // Room coordinates for tag corner.
            vector<at::Point> room_corner_detections(2*zarray_size(detections)); 

            
            static ptime epoch(boost::gregorian::date(1970,1,1));
            uint64_t msecs = (import_start - epoch).total_milliseconds();
#if TIME_PROFILING

#else

#endif
            buf.cam_id = htobe32(CAM_NAME);
            int n_detections = zarray_size(detections);
            std::vector<cv::Point2f> points;

            float max_temp_alpha = 0.0f;
            float max_temp_a = 0.0f;

            for (int i = 0; i < n_detections; i++) {
                apriltag_detection_t *dd;
                zarray_get(detections, i, &dd);
                Tag* tag = tags + dd->id;

                Tag* previous_tag = previous_tags + dd->id;

                cv::Point2f* cornerDetection = 2*i + room_corner_detections.data();
                cv::Point2f* detection = i + camera_detections.data();
                update_tag(detection, cornerDetection, latest_frame, tag, file_output);
                detection = i + room_detections.data();
                add_detection_to_msg(dd->id, detectionTime_ms, tag->x, tag->y, 
                                     tag->theta, i, CAM_NAME, buf);   // added 2024, "detectionTime_ms" added
                
                max_temp_alpha = max(max_temp_alpha, std::abs(tag->theta));

                if (tag->valid_velocity && previous_tag->valid_velocity) {


                    auto elapsed = tag->latest_detection - previous_tag->latest_detection;
                    float us = elapsed.total_microseconds();
                    float time_s = us / 1e6 + time_uncertainty;

                    float temp_a = std::abs((tag->velocity - previous_tag->velocity) / time_s);
                    max_temp_a = max(max_temp_a, temp_a);

#if PRINT_DEBUG_MSG           

                    // file_output << "Previous velocity: " << previous_tag->velocity << " Current velocity: " << tag->velocity << " time s: " << time_s << " acceleration: " << temp_a << endl;
                    file_output << "Previous velocity = " << previous_tag->velocity << ", Current velocity = " << tag->velocity << ", time s = " << time_s << ", acceleration = " << temp_a << endl;

#endif

                }
                    // max_temp_v = max(max_temp_v, tag->velocity); 

             // First create an apriltag_detection_info_t struct using your known parameters.
                apriltag_detection_info_t info;
                info.det = dd;
                info.tagsize = opts.tag_size;


                // Define the parameter table corresponding to the device
                struct DeviceInfo {
                    double fx_2k, fx_4k;
                    double fy_2k, fy_4k;
                    double cx_2k, cx_4k;
                    double cy_2k, cy_4k;
                };

                static const DeviceInfo device_params[] = {
                    {1135.208474, 2270.416948, 1133.531325, 2267.062650, 998.932805, 1997.865610, 530.084624, 1060.169248}, // Device 1
                    {1146.495853, 2290.512606, 1146.011083, 2283.316632, 953.976032, 1923.229236, 571.868765, 1028.981542}, // Device 2
                    {1145.256303, 2292.991706, 1141.658316, 2292.022166, 961.614618, 1907.952064, 514.490771, 1143.737530}, // Device 3
                    {1148.845354, 2297.690708, 1136.968276, 2273.936552, 987.148727, 1974.297454, 523.721356, 1047.443252}  // Device 4
                };

                if (opts.device_num >= 1 && opts.device_num <= 4) {
                    const DeviceInfo& params = device_params[opts.device_num - 1];

                    if (opts.frame_width == 3840 && opts.frame_height == 2160) { // 4K
                        info.fx = params.fx_4k;
                        info.fy = params.fy_4k;
                        info.cx = params.cx_4k;
                        info.cy = params.cy_4k;
                    } else if (opts.frame_width == 1920 && opts.frame_height == 1080) { // 2K
                        info.fx = params.fx_2k;
                        info.fy = params.fy_2k;
                        info.cx = params.cx_2k;
                        info.cy = params.cy_2k;
                    } else {
                        // default value or error handling
                        throw std::invalid_argument("Unsupported resolution");
                    }
                } else {
                    throw std::invalid_argument("Unsupported device number");
                }



                // Then call estimate_tag_pose.
                apriltag_pose_t pose;
                double err = estimate_tag_pose(&info, &pose);
                // Do something with pose.

                // change to Eigen format
                Eigen::Matrix4d T_tag_to_camera = Eigen::Matrix4d::Identity();
                for (int r = 0; r < 3; r++) {
                    for (int c = 0; c < 3; c++) {
                        T_tag_to_camera(r, c) = MATD_EL(pose.R, r, c);
                    }
                    T_tag_to_camera(r, 3) = MATD_EL(pose.t, r, 0);
                }

                Eigen::Matrix4d camera_to_world = camera_to_world_matrices[camera_id];


                // Calculate the position of the label in the world coordinate system
                Eigen::Matrix4d T_tag_to_world = camera_to_world * T_tag_to_camera;

                // Extract the world coordinate position of the label
                Eigen::Vector3d position = T_tag_to_world.block<3, 1>(0, 3);

                // Extract the world coordinate direction of the label
                Eigen::Matrix3d rotation = T_tag_to_world.block<3, 3>(0, 0);
                Eigen::Quaterniond orientation(rotation);

                // print result
                // file_output << "=== Tag Pose in World Coordinate (Camera ID: " << camera_id << ") ===" << std::endl;
                // file_output << "Position:" << std::endl;
                // file_output << "  x: " << position.x() << std::endl;
                // file_output << "  y: " << position.y() << std::endl;
                // file_output << "  z: " << position.z() << std::endl;

                // file_output << "Orientation (quaternion):" << std::endl;
                // file_output << "  qw: " << orientation.w() << std::endl;
                // file_output << "  qx: " << orientation.x() << std::endl;
                // file_output << "  qy: " << orientation.y() << std::endl;
                // file_output << "  qz: " << orientation.z() << std::endl;

                // 清理资源
                matd_destroy(pose.R);
                matd_destroy(pose.t);
                
                // Now, pose.t should contain the translation vector (x, y, z)
                // if (pose.t) {
                //     // Assuming pose.t is a pointer to a matd_t structure representing a 3x1 translation vector
                //     double x = pose.t->data[0];  // The x-coordinate
                //     double y = pose.t->data[1];  // The y-coordinate
                //     double z = pose.t->data[2];  // The z-coordinate
                    
                //     // Print the coordinates
                //     file_output << "Tag position in 3D space (camera coordinates): " << "x: " << x << ", y: " << y << ", z: " << z << std::endl;
                // } else {
                //     std::cout << "Error: Translation vector (pose.t) is null." << std::endl;
                // }


                // Calculate the size of the AprilTag in pixels
                float scaling_f = 0.125;
                DetectionArea* area = &tag->area;
                double apriltag_size = 0.0;
                for (int j = 0; j < 4; j++) {
                    int next = (j + 1) % 4;
                    double dx = dd->p[next][0] - dd->p[j][0];
                    double dy = dd->p[next][1] - dd->p[j][1];
                    apriltag_size += sqrt(dx * dx + dy * dy); // Sum edge lengths
                }
                apriltag_size /= 4.0; // Average size of edges

                // Calculate the size of the search area
                double search_area_width = scaling_f * (area->x_end - area->x_start);
                double search_area_height = scaling_f * (area->y_end - area->y_start);
                double search_area_size = search_area_width * search_area_height;

                // Calculate the ratio of search area size to AprilTag size
                double ratio = search_area_size / apriltag_size;

#if PRINT_DEBUG_MSG           
                // Print out the ratio and related information
                // std::cout << "Tag ID: " << dd->id
                //         << ", AprilTag Size: " << apriltag_size
                //         << ", Search Area Size: " << search_area_size
                //         << ", Ratio: " << ratio << std::endl;
#endif


            }

            // added 2025

#if USE_EWMA
            alpha = angle_tracker.get_max_value(5);

            angle_tracker.add_number(max_temp_alpha);

            a_max = acceleration_tracker.get_max_value(5);

            acceleration_tracker.add_number(max_temp_a);
#endif



            for(int i = 0; i<100; i++){
                if(sem_1.try_wait()){
                    break;
                }
                usleep(1);
            }
            buf.length = htobe32(n_detections);
            memcpy(&(ptr->msg), &buf, sizeof(buf));
            ptr->flag = 1;
            sem_1.post();

#if PRINT_DEBUG_MSG           
            // std::cout << "GTR = " << avg_time_gap << " ms" << std::endl;
#endif



            // use_exhaustive_search = false;
        }
         // Producer: writes its own detection results to the neighbouring camera's buffer
        for (int i = 0; i < 2 && produce_buffers[i]; ++i) {
            for (int id = 0; id < MAX_TAG_ID; ++id) {
                if (tags[id].is_detected) {  // Tag is detected
                    int y = tags[id].y;

                    // file_output << "Tag#" << id << ": x=" << tags[id].x << ", y=" << tags[id].y << endl;

                    // Check if y is in the overlap area
                    if (y >= overlap_ranges[camera_id][i].min_y && y <= overlap_ranges[camera_id][i].max_y) {
                        OverlapTagInfo info{id, a_max, alpha, tags[id].latest_detection};

#if PRINT_DEBUG_MSG           
                        file_output << "Tag#" << id << " detected in the overlapping area. Time frame: " << tags[id].latest_detection << endl;
#endif
                        produce_buffers[i]->produce(info);
                    }
                }
            }
        }


        // Consumer：read data from adjacent camera's buffer
        for (int i = 0; i < 2 && consume_buffers[i]; ++i) {
            OverlapTagInfo incoming;
            while (consume_buffers[i]->consume(incoming)) {
                ptime current_frame = boost::posix_time::microsec_clock::universal_time();
                time_duration frame_duration = current_frame - incoming.timestamp; // 两个ptime相减
                long frame_duration_ms = frame_duration.total_milliseconds(); // 转换为毫秒

                // detect time difference and Tag status
                if (frame_duration_ms <= 5 && !tags[incoming.tag_id].is_detected) {
                    use_exhaustive_search = true;
                    a_max = incoming.a_max;
                    alpha = incoming.alpha;
#if PRINT_DEBUG_MSG           
                    file_output << "Missing detection of Tag#" << incoming.tag_id << " in the overlapping area. Frame duration time: " << frame_duration_ms << endl;
#endif
                    break;
                }
            }
        }

        //If there are tags missing, use exhaustive search next time
        for(Tag* tag = tags; tag < tags + MAX_TAG_ID; tag++){

            if (!tag->is_detected && tag_exists(tag->x, tag->y)) {
                use_exhaustive_search = true;
#if PRINT_DEBUG_MSG           
                file_output << "Tags are missing." << endl;
#endif
            }
            tag->is_detected = false; //clears variable for next search
        }


        auto process_end = std::chrono::high_resolution_clock::now();

        double process_duration = std::chrono::duration_cast<std::chrono::microseconds>(process_end - process_start).count();

#if PRINT_DEBUG_MSG           
        file_output << "Process time: " << std::fixed << std::setprecision(2) << process_duration / 1000.0 << " ms" << std::endl;
#endif

    fast_thread_logger.log_operation(DebugLogger::PROCESS_TIME, process_duration, fast_consumer_counter[camera_id].load());

        bool have_exhaustive_searched = false;

        DetectionData detection_data;

        if (use_exhaustive_search || global_search_counter == GLOBAL_SEARCH_MIN) {

            search_start = std::chrono::high_resolution_clock::now();


#if PRINT_DEBUG_MSG
            // file_output <<"CAM#"<<CAM_NAME<<": " << "Using GLOBAL SEARCH ##############################################\n";
            file_output <<"CAM#"<<CAM_NAME<<" " << "Using GLOBAL SEARCH ##############################################\n";
#endif

            loop_count = 1;
            global_search_counter = 0;
            // use_exhaustive_search = false;

            have_exhaustive_searched = true;

            std::copy(std::begin(tags), std::end(tags), previous_tags);

            //Clear previous coordinates of all tags.
            for (Tag* tag = tags; tag < tags + MAX_TAG_ID; tag++) {
                reset_tag(im.width, im.height, tag);
            }
            // detections = exhaustive_search(im, detector);

            // print_counters(camera_id, file_output);

            while(search_consumer_counter[camera_id].load() == search_producer_counter[camera_id].load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }

            // cout << "fast: " << nice_consumer_counter[camera_id].load() << " " << fast_consumer_counter[camera_id].load() << endl;

            search_consumer_counter[camera_id] = search_producer_counter[camera_id].load();
            detection_data = search_buffer[camera_id][search_consumer_counter[camera_id].load()];


            std::copy(std::begin(detection_data.tag_data), std::end(detection_data.tag_data), tags);

            search_end = std::chrono::high_resolution_clock::now();
            search_time = std::chrono::duration_cast<std::chrono::microseconds>(search_end - search_start).count();


#if PRINT_DEBUG_MSG

            // double st = boost::posix_time::milliseconds(search_time).total_microseconds();

            // file_output <<"CAM#"<<CAM_NAME<<": "<< "GLOBAL SEARCH: search_time: " << std::fixed << std::setprecision(2) << search_time / 1000.0 << " ms\n";
            file_output <<"CAM#"<<CAM_NAME<<" "<< "GLOBAL SEARCH time: " << std::fixed << std::setprecision(2) << search_time / 1000.0 << " ms\n";
#endif

            fast_thread_logger.log_operation(DebugLogger::GLOBAL_SEARCH_TIME, search_time, fast_consumer_counter[camera_id].load());


            float max_temp_alpha = 0.0f;
            float max_temp_a = 0.0f;

            bool no_detected = true;

            for (int i = 0; i < MAX_TAG_ID; i++) {
                if (detection_data.tags[i].found) {

                    no_detected = false;

                    max_temp_alpha = max(max_temp_alpha, std::abs(tags[i].theta));

                    if (tags[i].valid_velocity && previous_tags[i].valid_velocity) {

                        auto elapsed = tags[i].latest_detection - previous_tags[i].latest_detection;
                        float us = elapsed.total_microseconds();
                        float time_s = us / 1e6 + time_uncertainty;

                        float temp_a = std::abs((tags[i].velocity - previous_tags[i].velocity) / time_s);
                        max_temp_a = max(max_temp_a, temp_a);

#if PRINT_DEBUG_MSG
                        // file_output << "Previous velocity: " << previous_tags[i].velocity << " Current velocity: " << tags[i].velocity << " time s: " << time_s << " acceleration: " << temp_a << endl;
                        file_output << "Previous velocity = " << previous_tags[i].velocity << ", Current velocity = " << tags[i].velocity << ", time s = " << time_s << " acceleration: " << temp_a << endl;
#endif
                    }

#if PRINT_DEBUG_MSG
                    // file_output <<"CAM#"<<CAM_NAME<<": " 
                    //           << "Found when using GLOBAL_IMAGE. X: " 
                    //           << detection_data.tags[i].camera_coords->x << " Y:" 
                    //           << detection_data.tags[i].camera_coords->y << endl;
                    file_output <<"CAM#"<<CAM_NAME<<" " 
                              << "Found when using GLOBAL_IMAGE. X = " 
                              << detection_data.tags[i].camera_coords->x << " Y = " 
                              << detection_data.tags[i].camera_coords->y << endl;
#endif
                }
            }



#if USE_EWMA
    if (!no_detected) {
            alpha = angle_tracker.get_max_value(5);

            angle_tracker.add_number(max_temp_alpha);

            a_max = acceleration_tracker.get_max_value(5);

            acceleration_tracker.add_number(max_temp_a);
    }

        if (detection_data.buf) {    

            for(int i = 0; i<100; i++){
                if(sem_1.try_wait()){
                    break;
                }
                usleep(1);
            }
            memcpy(&(ptr->msg), &detection_data.buf, sizeof(detection_data.buf));
            ptr->flag = 1;
            sem_1.post();

#if PRINT_DEBUG_MSG
            // std::cout << "GTR = " << avg_time_gap << " ms" << std::endl;
#endif

            detection_data.clearMessage();

        }

#endif

            // printDetectionData(detection_data);
        }

        if (not opts.no_gui) {
            have_exhaustive_searched? update_exhaustive_gui(detection_data, tags, frame, avg_hz) : update_gui(detections, tags, frame, avg_hz, file_output);
			if (opts.mirror_display) {
				cv::flip(frame, frame, 1);
			}
	    
            cv::imshow(win, frame);
            cv::waitKey(1);
        }

        if (sig_stop) {
            break;
        }
        apriltag_detections_destroy(detections);

        loop_count++;

	ptime import_end = boost::posix_time::microsec_clock::universal_time();

	uint32_t tot_time = (import_end - import_start).total_milliseconds();
	
	sum_hz += tot_time;

        if (hz_counter == 2*FPS) {
	        avg_hz = 1 / (static_cast<float>(tot_hz)/(2*FPS*1000));

#if PRINT_DEBUG_MSG
            // file_output << "Hz: " << avg_hz << std::endl;
	        // file_output << "CAM " << CAM_NAME << ": " << avg_hz << "Hz"  << std::endl;
            file_output << "Frequency " << CAM_NAME << ": " << avg_hz << "Hz"  << std::endl;
#endif

            hz_counter = 0;
            sum_hz = 0;
	    }
        hz_counter++;

            // End measurement
    auto loop_end = std::chrono::high_resolution_clock::now();

    // Calculation time (in microseconds)
    auto loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start).count();

#if PRINT_DEBUG_MSG
    // printing time
    // std::cout << "Execution time for one loop: " << std::fixed << std::setprecision(2) << loop_duration / 1000.0 << " ms" << std::endl;
    // Log the performance
    // file_output << "Loop " << total_loop_count - 1 << ": epsilon=" << epsilon << ", Duration=" << loop_duration / 1000 << " ms\n";
    file_output << "Loop: count=" << total_loop_count - 1 << ", trial=" << trial << ", Duration=" << std::fixed << std::setprecision(2) << loop_duration << " us\n";

#endif

    fast_thread_logger.log_operation(DebugLogger::LOOP_TIME, loop_duration, fast_consumer_counter[camera_id].load());
    fast_thread_logger.write_to_file_if_needed(loop_duration, "fast-producer", filename.str());


    }

    }

    // tagStandard41h12_destroy(tf);

    apriltag_detector_destroy(detector);

    file_output.close();

    
    return 0;
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