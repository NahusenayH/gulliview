#ifndef _DECLARATIONS_H_
#define _DECLARATIONS_H_

#include <boost/asio.hpp>   //Used in GUI.cpp and CalibrateCameras.cpp but needed here as well

// This is for visualize_GulliView_logs
// Version string, adds to time period ex VT25.2
#define TIME_PERIOD "VT25"
#define VERSION "18"
// change this text to denote version, this is saved by log script to catagorize
#define COMMENT "FastSearch file complete"

#define ENABLE_LOGS        true
#define LIVE_FEED          false
#define RECORDING_FOLDER   "recordings0.5"

// Older defines
#define PRINT_DEBUG_MSG         true
#define FAST_SEARCH_ACC_TEST    false
#define TIME_PROFILING          false

#define USE_MEMORY_SHARING      false // added 2025
#define USE_EWMA                true // added 2025
#define BINDING_CPU_CORES       true // added 2025

#define PRODUCE_FRAME_MODE      1 // added 2025

#define DEFAULT_TAG_FAMILY      "tag36h11" // tag36h11
#define DEFAULT_IP              "127.0.0.1"
#define DEFAULT_PORT            "2121"

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

const std::string CALIBRATION_TAG_FAMILY = "tag25h9";

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


#endif