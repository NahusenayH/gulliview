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

void set_search_area(const int32_t im_width,
                    const int32_t im_height,
                    const int min_search_dim,
                    const float min_travel,
                    const float max_travel,
                    const float alpha,
                    Tag& tag) {

    DetectionArea& area = tag.area;
    std::vector<float> xs{
        0.0f,
        max_travel * cosf(tag.theta),
        max_travel * cosf(tag.theta + alpha),
        max_travel * cosf(tag.theta - alpha),
        min_travel * cosf(tag.theta),
        min_travel * cosf(tag.theta + alpha),
        min_travel * cosf(tag.theta - alpha)
    };

    std::vector<float> ys{
        0.0f,
        max_travel * sinf(tag.theta),
        max_travel * sinf(tag.theta + alpha),
        max_travel * sinf(tag.theta - alpha),
        min_travel * sinf(tag.theta),
        min_travel * sinf(tag.theta + alpha),
        min_travel * sinf(tag.theta - alpha)
    };

    std::vector<float>::iterator x_min = std::min_element(xs.begin(), xs.end());
    std::vector<float>::iterator x_max = std::max_element(xs.begin(), xs.end());
    std::vector<float>::iterator y_min = std::min_element(ys.begin(), ys.end());
    std::vector<float>::iterator y_max = std::max_element(ys.begin(), ys.end());
    area.x_start = std::max(*x_min + tag.x - min_search_dim, 0.0f);
    area.y_start = std::max(*y_min + tag.y - min_search_dim, 0.0f);
    area.x_end = std::min(*x_max + tag.x + min_search_dim, float(im_width));
    area.y_end = std::min(*y_max + tag.y + min_search_dim, float(im_height));

    area.x_length = area.x_end - area.x_start;
    area.y_length = area.y_end - area.y_start;
}