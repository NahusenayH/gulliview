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

#include "InitCameras.hpp"

//{added to handle empty frames
#include <array>
#include <filesystem>
#include <optional>

namespace {

const std::array<std::string, 4> DEVICE_PATHS = {
    "/dev/v4l/by-path/pci-0000:00:0d.0-usb-0:1:1.0-video-index0",
    "/dev/v4l/by-path/pci-0000:00:0d.0-usb-0:3:1.0-video-index0",
    "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:1:1.0-video-index0",
    "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0"
};

std::optional<std::string> resolve_device_path(int device_number) {
    if (device_number < 0 ||
        device_number >= static_cast<int>(DEVICE_PATHS.size())) {
        return std::nullopt;
    }
    const auto& path = DEVICE_PATHS[device_number];
    if (std::filesystem::exists(path)) {
        return path;
    }
    return std::nullopt;
}

}  // namespace
//}

void init_video_open(const int32_t device_number,
                    const int32_t frame_width,
                    const int32_t frame_height,
                    cv::VideoCapture& video_capture,
                    cv::Mat& frame){

    std::cout << "device number: " << device_number << std::endl;
    std::string folder = RECORDING_FOLDER;
    std::string video_path = "../src/" + folder + "/video" + std::to_string(device_number*2) + ".mp4";
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


//fix from as_is to handle empty frames
bool open_camera_device(const int32_t device_number,
                        const int32_t frame_width,
                        const int32_t frame_height,
                        cv::VideoCapture& video_capture) {
    video_capture.release();

    bool opened = false;

    if (auto device_path = resolve_device_path(device_number)) {
        video_capture = cv::VideoCapture(*device_path, cv::CAP_V4L2);
        if (video_capture.isOpened()) {
            std::cout << "Camera " << device_number << " using device " << *device_path << std::endl;
            opened = true;
        } else {
            std::cerr << "Camera " << device_number << " failed to open " << *device_path
                      << ", falling back to index based lookup" << std::endl;
        }
    }

    if (!opened) {
        const int fallback_index = 2 * device_number;
        std::cout << "Camera " << device_number << " falling back to /dev/video" << fallback_index << std::endl;
        video_capture = cv::VideoCapture(fallback_index, cv::CAP_V4L2);
        opened = video_capture.isOpened();
    }

    if (!opened) {
        std::cerr << "Camera " << device_number << " failed to open capture device" << std::endl;
        return false;
    }

    video_capture.set(cv::CAP_PROP_AUTOFOCUS, 0);
    video_capture.set(cv::CAP_PROP_FOCUS, 100);
    video_capture.set(cv::CAP_PROP_BUFFERSIZE, 1);

    int32_t codec = cv::VideoWriter::fourcc('M','J','P','G');
    if (!video_capture.set(cv::CAP_PROP_FOURCC, codec)) {
        std::cerr << "Camera " << device_number << " failed to configure MJPG codec" << std::endl;
    } else {
        std::cout << "Camera " << device_number << " using codec MJPG" << std::endl;
    }

    video_capture.set(cv::CAP_PROP_FPS, FPS);

    if (frame_width && frame_height) {
        video_capture.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
        video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
    }

    return true;
}

// void init_video_capture(const int32_t device_number,
//                         const int32_t frame_width,
//                         const int32_t frame_height,
//                         cv::VideoCapture& video_capture,
//                         cv::Mat& frame){

//     /* choose camera and buffer-size */
//     std::cout << "Camera " << device_number << " init" << std::endl;

//     video_capture = cv::VideoCapture(2*device_number, cv::CAP_V4L2);
//     video_capture.set(cv::CAP_PROP_AUTOFOCUS, 0); // Turn off autofocus
//     video_capture.set(cv::CAP_PROP_FOCUS, 100); // Set focus to manual value
//     video_capture.set(cv::CAP_PROP_BUFFERSIZE, 1);

//     /* set output codec and FPS */
//     int32_t codec = cv::VideoWriter::fourcc('M','J','P','G');
//     video_capture.set(cv::CAP_PROP_FOURCC, codec);
//     video_capture.set(cv::CAP_PROP_FPS, FPS);

//     /* set video height and width */
//     if (frame_width && frame_height) {
//         // Use uvcdynctrl to figure this out dynamically at some point?
//         video_capture.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
//         video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
//     }

//     video_capture >> frame;

//     std::cout << "enter here" << std::endl;

//     std::cout << "Frame rate: " << video_capture.get(cv::CAP_PROP_FPS) << " FPS" << std::endl;

//     if (frame.empty()) {
//         std::cerr << "no frames from camera " << device_number << std::endl;
//         // exit(1);
//     }
// }

//edited to handle empty frames
void init_video_capture(const int32_t device_number,
                        const int32_t frame_width,
                        const int32_t frame_height,
                        cv::VideoCapture& video_capture,
                        cv::Mat& frame) {

    std::cout << "Camera " << device_number << " init" << std::endl;

    if (!open_camera_device(device_number, frame_width, frame_height, video_capture)) {
        return;
    }

    video_capture >> frame;

    std::cout << "enter here" << std::endl;
    std::cout << "Frame rate: " << video_capture.get(cv::CAP_PROP_FPS) << " FPS" << std::endl;

    if (frame.empty()) {
        std::cerr << "no frames from camera " << device_number << std::endl;
    }
}