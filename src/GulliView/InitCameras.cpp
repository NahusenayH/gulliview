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

void init_video_capture(const int32_t device_number,
                        const int32_t frame_width,
                        const int32_t frame_height,
                        cv::VideoCapture& video_capture,
                        cv::Mat& frame){

    /* choose camera and buffer-size */
    std::cout << "Camera " << device_number << " init" << std::endl;

    video_capture = cv::VideoCapture(2*device_number, cv::CAP_V4L2);
    video_capture.set(cv::CAP_PROP_AUTOFOCUS, 0); // Turn off autofocus
    video_capture.set(cv::CAP_PROP_FOCUS, 100); // Set focus to manual value
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

    std::cout << "enter here" << std::endl;

    std::cout << "Frame rate: " << video_capture.get(cv::CAP_PROP_FPS) << " FPS" << std::endl;

    if (frame.empty()) {
        std::cerr << "no frames from camera " << device_number << std::endl;
        // exit(1);
    }
}