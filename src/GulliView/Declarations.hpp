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

#ifndef _DECLARATIONS_H_
#define _DECLARATIONS_H_

// Used in other files but needed here as well
#include <optional>
#include <boost/asio.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../Detections.h"

#include "LogTime.hpp"

// This is for visualize_GulliView_logs
// Version string, adds to time period ex VT25.2
#define TIME_PERIOD "VT25"
#define VERSION "67"
// change this text to denote version, this is saved by log script to catagorize
#define COMMENT "Latency test with pixel remap"

#define ENABLE_FAST_LOGS        false                   // Enables fast thread log output files
#define ENABLE_NICE_LOGS        false                   // Enables nice thread log output files
#define ENABLE_PRODUCER_LOGS    false                   // Enables producer thread log output files
#define ENABLE_ANY_LOGS         ENABLE_FAST_LOGS || ENABLE_NICE_LOGS || ENABLE_PRODUCER_LOGS

#define LIVE_FEED               false                    // If the cameras live feed or recordings from RECORDING_FOLDER are used
#define LOOP_RECORDING          true
#define RECORDING_FOLDER        "recordings_1bot_0.7"   // Folder to get recordings from

#define GUI_SCALE               0.2                     // Scales GUI to fit monitor, higher res needs smaller factor. Use values of 0.5^k as fit
#define ELIAS_PRINT             false                   // Elias prints global coordination
#define RUN_ONLY_PRODUCER       false    // Only starts producer thread, used for debugging camera stability

// Old values
#define FAST_THREAD_NUM         0//4    // Start at here
#define FAST_THREAD_COUNT       4    // Count this many
#define NICE_THREAD_NUM         2//0    // Start at here
#define NICE_THREAD_COUNT       4    // Count this many
#define PRODUCER_THREAD_NUM     4//8    // Start at here
#define PRODUCER_THREAD_COUNT   4    // Count this many

// New values
// #define FAST_THREAD_NUM         8    // Start at here
// #define FAST_THREAD_COUNT       4    // Count this many
// #define NICE_THREAD_NUM         12    // Start at here
// #define NICE_THREAD_COUNT       4    // Count this many
// #define PRODUCER_THREAD_NUM     0    // Start at here
// #define PRODUCER_THREAD_COUNT   8    // Count this many


// Older defines
#define PRINT_DEBUG_MSG         false   // Should soon be replaced by ENABLE_LOGS
#define FAST_SEARCH_ACC_TEST    false
#define TIME_PROFILING          false

#define USE_MEMORY_SHARING      false
#define USE_EWMA                true
#define BINDING_CPU_CORES       true

#define PRODUCE_FRAME_MODE      1

#define DEFAULT_TAG_FAMILY      "tag36h11" // tag36h11
#define DEFAULT_IP              "127.0.0.1"
#define DEFAULT_PORT            "2121"

#define MAX_TAG_ID                      10

#define ROOM_WIDTH_METER  5.035f

#define DEFAULT_VELOCITY_MAX 0.3
#define DEFAULT_ACCELERATION_MAX 8
#define DEFAULT_LIMIT_MAX 17000

#define FPS 30
#define BUFFER_SIZE 2
#define PARALLELL_FRAME_COUNT 2
#define GLOBAL_SEARCH_MIN 16

const std::string CALIBRATION_TAG_FAMILY = "tag25h9";

extern sig_atomic_t sig_stop;

extern int nines;

// added 2025
// Global shared frame counter (thread-safe)
extern std::atomic<uint32_t> shared_frame_count;

struct FrameData {
    LogTime frametime;  // Use high_resolution_clock instead
    cv::Mat frame;
};

extern FrameData buffer[4][BUFFER_SIZE]; // modified 2025

typedef struct __attribute__ ((packed)) DetectionArea {
    int32_t x_start;
    int32_t y_start;
    int32_t x_end;
    int32_t y_end;
    int32_t x_length;
    int32_t y_length;
} DetectionArea;

typedef struct Tag {
    int32_t x = 0;
    int32_t y = 0;
    cv::Mat world_position = cv::Mat::zeros(1,3,CV_64F); // used for global coordination
    cv::Mat world_rotation = cv::Mat::zeros(3,3,CV_64F); // used for global coordination
    bool is_detected = 0;
    float velocity = 0;
    bool valid_velocity = false;
    float theta;
    boost::posix_time::ptime latest_detection;
    DetectionArea area;
} Tag;

typedef struct __attribute__ ((packed)) DetectionMessage {
    uint32_t id;
    uint64_t time_msec;    //added 2024
    uint32_t x;
    uint32_t y;
    uint32_t z;
    float theta;
    float speed;
    uint32_t camera_id;
} DetectionMessage;

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

struct SharedData {
    int flag;
    Message msg;
};

extern IntPoint mainEntry;
extern IntPoint rampEntry;
extern IntPoint mainBot;
extern IntPoint rampBot;
extern uint32_t entryRadius;

//std::shared_ptr<EntryDetection> mainDetection = std::make_shared<EntryDetection>();

extern EntryDetection rampDetection;

// Limits of the lab
extern uint32_t xMinLimit;
extern uint32_t xMaxLimit;
extern uint32_t yMinLimit;
extern uint32_t yMaxLimit;

extern DetectionData search_buffer[4][BUFFER_SIZE]; // added 2025

// using for storing data produced by producer
struct BufferData {
    cv::Mat frame;         // original frame
    cv::Mat gray;          // gray frame

};


// Overlapping data structure
struct OverlapTagInfo {
    int tag_id;
    Tag tag;
    float a_max;
    float alpha;
    boost::posix_time::ptime timestamp;         // Timestamp indicating the time of inspection at the time of production
};

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

// Table of overlapping ranges in 1080p
const OverlapRange overlap_ranges_1080p[4][2] = {
    // Overlap range for camera 0
    {{0, 250}, {0, 0}}, // buffer_01
    // Camera 1's overlap range
    {{830, 1080}, {0, 290}}, // buffer_10, buffer_12
    // Camera 2's overlap area
    {{790, 1080}, {0, 230}}, // buffer_21, buffer_23
    // Camera 3's overlap area
    {{850, 1080}, {0, 0}} // buffer_32
};

// std::atomic<unsigned int> producer_counter(0);
// std::atomic<unsigned int> consumer_counter(0);

// modified 2025

extern std::vector<std::atomic<unsigned int>> producer_counter;
extern std::vector<std::atomic<unsigned int>> search_producer_counter;
extern std::vector<std::atomic<unsigned int>> search_consumer_counter;
extern std::vector<std::atomic<unsigned int>> fast_consumer_counter;
extern std::vector<std::atomic<unsigned int>> nice_consumer_counter;

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
extern CyclicBuffer buffer_01, buffer_10, buffer_12, buffer_21, buffer_23, buffer_32;

#endif
