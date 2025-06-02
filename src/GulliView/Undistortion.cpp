/*********************************************************************
* This is the main file of the implementation of the
* new Undistortion which undistorts certain points
********************************************************************/

#include "Undistortion.hpp"

// the K matrix for every camera 
std::map<int, cv::Mat> camera_K_matrices = { // parameters for 4k
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

// the distortions coeffs for every camera
std::map<int, cv::Mat> camera_global_distortion_coefficients = { // parameters for 4k
    {0, (cv::Mat_<double>(1, 5) << -0.073165, 0.025731, 0.006159, 0.007375, 0.0)},
    {1, (cv::Mat_<double>(1, 5) << -0.079821, 0.027860, -0.002763, -0.00027, 0.0)},
    {2, (cv::Mat_<double>(1, 5) << -0.080634, 0.032041, 0.006835, 0.000915, 0.0)},
    {3, (cv::Mat_<double>(1, 5) << -0.095870, 0.038556, 0.007781, 0.001609, 0.0)},
};

// Undistorts the given points and returns the undistorted points. 
// points (input): the points to undistort
// camera_id (input): id of the camera of the image
cv::Mat undistort_points(cv::Mat points, int camera_id) {
    cv::Mat undistorted_points;
    cv::Mat K = camera_K_matrices[camera_id];
    cv::Mat distortion_coeffs = camera_global_distortion_coefficients[camera_id];
    cv::undistortPoints(points, undistorted_points, K, distortion_coeffs, cv::Mat(), K);
    return undistorted_points;
}