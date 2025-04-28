/*********************************************************************
* This is the main file of the implementation of the 
* Global Coordination system 2025.
********************************************************************/

#include "GlobalCoordination.hpp"

cv::Mat R_tag2global = (cv::Mat_<double>(3,3) << 1, 0, 0,   0, 1, 0,    0, 0, -1);

std::map<int, cv::Mat> global_tag_positions = {
    {0, (cv::Mat_<double>(1,3) << 0, 0, 0)},
    {1, (cv::Mat_<double>(1,3) << 4.25, 0, 0)},
    {2, (cv::Mat_<double>(1,3) << 0, 2, 0)},
    {3, (cv::Mat_<double>(1,3) << 4.25, 2, 0)},

    {4, (cv::Mat_<double>(1,3) << 0, 4, 0)},
    {5, (cv::Mat_<double>(1,3) << 4.28, 4, 0)},
    {6, (cv::Mat_<double>(1,3) << 0, 6, 0)},
    {7, (cv::Mat_<double>(1,3) << 4.28, 6, 0)},
    {8, (cv::Mat_<double>(1,3) << 0, 8, 0)},
    {9, (cv::Mat_<double>(1,3) << 4.28, 8, 0)},
};

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




void estimate_object_global_position(int camera_id, cv::Mat undistorted_points, cv::Mat* global_position, cv::Mat* global_rotation) {
    // Estimating Object Global Position
    // ###################################################################3
    cv::Mat K = camera_K_matrices[camera_id];
    cv::Mat distortion_coeffs = camera_global_distortion_coefficients[camera_id];
    double apriltag_size = 0.16;
    cv::Mat object_points = (cv::Mat_<double>(4,3) <<   -apriltag_size/2    , -apriltag_size/2 , 0,
                                                         apriltag_size/2    , -apriltag_size/2 , 0,
                                                         apriltag_size/2    ,  apriltag_size/2 , 0,
                                                        -apriltag_size/2    ,  apriltag_size/2 , 0);

    cv::Mat obj2cam_rvec, obj2cam_tvec;
    cv::solvePnP(object_points, undistorted_points, K, cv::Mat(), obj2cam_rvec, obj2cam_tvec);
    std::cout << "rvec " << obj2cam_rvec << " tvec " << obj2cam_tvec << std::endl;

    // MUST CALCULATE VALUES for obj2cam_tvec and obj2cam_rvec, these are gotten 

    cv::Mat R_obj2cam;
    cv::Mat T_obj2cam = obj2cam_tvec;
    cv::Rodrigues(obj2cam_rvec, R_obj2cam); // turns obj2cam_rvec into a 3x3 rotation matrix R_obj2cam
    
    std::list<cv::Mat> all_global_positions;
    std::list<cv::Mat> all_R_obj2global;

    std::map<int, cv::Mat> cam2tag_rvec_list; // GET the 4 from a list of all these based on the given camera number - since each camera has 4 tags, each with different rvec and tvec based on camera
    std::map<int, cv::Mat> cam2tag_tvec_list; // GET same as the one above

    for (const auto list : cam2tag_rvec_list) {
        //std::cout << "index of tag: " << list.first << std::endl;
        cv::Mat R_cam2tag = list.second;
        cv::Mat T_cam2tag = cam2tag_tvec_list.at(list.first);
        cv::Mat R_obj2tag = R_cam2tag * R_obj2cam;
        //std::cout << "R_cam2tag: " << R_cam2tag << " T_obj2cam: " << T_obj2cam << std::endl;
        cv::Mat T_obj2tag = addmatxvector(matxvector(R_cam2tag, T_obj2cam), T_cam2tag);   //R_cam2tag.dot(T_obj2cam) + T_cam2tag;
        //std::cout << "T_obj2tag: " << T_obj2tag << "R_tag2global: " << R_tag2global << std::endl;
        cv::Mat global_positions = addmatxvector(matxvector(R_tag2global, T_obj2tag), global_tag_positions[list.first]);//R_tag2global * T_obj2tag;// + global_tag_positions[list.first];
        //std::cout << "global_positions: " << global_positions << std::endl;
        //std::cout << "###################" << std::endl;
        //std::cout << "R_tag2global: " << R_tag2global << "R_obj2tag: " << R_obj2tag << std::endl;

        cv::Mat R_obj2global = R_tag2global * R_obj2tag;
        
        all_global_positions.push_back(global_positions);
        all_R_obj2global.push_back(R_obj2global);
        std::cout << "Object position from tag " << list.first << ": " << global_positions << std::endl;
        //std::cout << "R from tag " << list.first << ": " << R_obj2global << std::endl;
    }
    // get average of all_global_positions and all_R_obj2global
    cv::Mat average_global_positions = average_mat(all_global_positions);
    cv::Mat average_R_obj2global = average_mat(all_R_obj2global);
    
    std::cout << "Object position from all tags: " << average_global_positions << std::endl;
    std::cout << "R from all tags: " << average_R_obj2global << std::endl;

    // returns
    global_position = &average_global_positions;
    global_rotation = &average_R_obj2global;
}

