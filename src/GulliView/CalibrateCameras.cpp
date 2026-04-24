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

#include "CalibrateCameras.hpp"

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
        break;
    case 2:
        destination[0] = at::Point(0.25, 4.84);
        destination[1] = at::Point(4.55, 4.91);
        destination[2] = at::Point(0.23, 6.95);
        destination[3] = at::Point(4.51, 7.02);
        break;
    case 3:
        destination[0] = at::Point(0.23, 6.95);
        destination[1] = at::Point(4.51, 7.02);
        destination[2] = at::Point(0.17, 9.26);
        destination[3] = at::Point(4.61, 9.18);
        break;
    }
}

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
    std::string calibration_tag_family_string = CALIBRATION_TAG_FAMILY;
    TagFamily family(calibration_tag_family_string);

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

    std::cout << "Calibration done for camera " << cam_name << "\n";
}