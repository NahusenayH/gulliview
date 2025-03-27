#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <cstring>
#include <string>


#include "TransformFrame.hpp"

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
        std::string name;
        std::ofstream& file;
};

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