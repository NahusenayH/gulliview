#include "TransformFrame.hpp"

bool transform_frame(cv::Mat& frame,
    cv::Mat& gray,
    cv::Mat& map1,
    cv::Mat& map2,
    std::ofstream& file_output // added 2025
    ) {
    // TODO save timestamp (maybe return the timestamp instead of bool)
    // cv::Mat undistorted_frame;

    if (frame.empty()) {
        std::cout << "no frame to transform, exiting" << std::endl;
        return false;
    }
#if ENABLE_LOGS
    LogTime remap_timer("Remap", file_output);
#endif
    // cv::remap(frame, frame, map1, map2, cv::INTER_LINEAR);
#if ENABLE_LOGS
    remap_timer.stop_us();
#endif

#if ENABLE_LOGS
    LogTime color_timer("Transform color", file_output);
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