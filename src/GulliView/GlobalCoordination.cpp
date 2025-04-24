/*********************************************************************
* This is the main file of the implementation of the 
* Global Coordination system 2025.
********************************************************************/

#include "GlobalCoordination.hpp"

cv::Mat matxvector(cv::Mat a, cv::Mat b) {
    cv::Mat result = (cv::Mat_<double>(1,3) << a.at<double>(0,0) * b.at<double>(0,0) + a.at<double>(0,1) * b.at<double>(0,1) + a.at<double>(0,2) * b.at<double>(0,2),
                                               a.at<double>(1,0) * b.at<double>(0,0) + a.at<double>(1,1) * b.at<double>(0,1) + a.at<double>(1,2) * b.at<double>(0,2),
                                               a.at<double>(2,0) * b.at<double>(0,0) + a.at<double>(2,1) * b.at<double>(0,1) + a.at<double>(2,2) * b.at<double>(0,2)
                      );
    return result;
}

cv::Mat addmatxvector(cv::Mat a, cv::Mat b) {
    cv::Mat result = (cv::Mat_<double>(1,3) << a.at<double>(0,0) + b.at<double>(0,0),
                                               a.at<double>(0,1) + b.at<double>(0,1),
                                               a.at<double>(0,2) + b.at<double>(0,2));
    return result;
}

cv::Mat average_mat(std::list<cv::Mat> mats) {
    double zero = 0.0;
    cv::Mat sum = cv::Mat(mats.front().rows, mats.front().cols, CV_64F, zero);//cv::Mat_<double>(mats.front().rows, mats.front().cols);
    int length = mats.size();
    //std::cout << "first step done! mats length: " << length  << std::endl;
    for (cv::Mat mat : mats) {
        //std::cout << "mat - rows: " << mat.rows << " cols: " << mat.cols << std::endl;
        for (int i = 0; i < mat.rows; i++) {
            for (int j = 0; j < mat.cols; j++) {
                //std::cout << "accessing mat at (" << i << "," << j << ")" << std::endl;
                sum.at<double>(i,j) += mat.at<double>(i,j);
            }
        }
    }
    // std::cout << "second step done" << std::endl;

    for (int i = 0; i < sum.rows; i++) {
        for (int j = 0; j < sum.cols; j++) {
            sum.at<double>(i,j) /= length;
        }
    }
    // std::cout << "third step done" << std::endl;

    return sum;
}