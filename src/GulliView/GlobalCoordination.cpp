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

/*********************************************************************
* This is the main file of the implementation of the 
* Global Coordination system 2025.
********************************************************************/

#include "GlobalCoordination.hpp"
#include "opencv2/calib3d.hpp"



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

cv::Mat R_tag2global = (cv::Mat_<double>(3,3) << 1, 0, 0,   0, -1, 0,    0, 0, 1);

std::map<int, cv::Mat> global_tag_positions = {
    {0, (cv::Mat_<double>(1,3) << 0,     0,     0)},
    {1, (cv::Mat_<double>(1,3) << 4.295, 0,     0)},
    {2, (cv::Mat_<double>(1,3) << 0,     2.002, 0)},
    {3, (cv::Mat_<double>(1,3) << 4.292, 1.995, 0)},

    {4, (cv::Mat_<double>(1,3) << 0,     4.003, 0)},
    {5, (cv::Mat_<double>(1,3) << 4.286, 3.995, 0)},
    {6, (cv::Mat_<double>(1,3) << 0,     5.992, 0)},
    {7, (cv::Mat_<double>(1,3) << 4.264, 5.997, 0)},
    {8, (cv::Mat_<double>(1,3) << 0,     7.989, 0)},
    {9, (cv::Mat_<double>(1,3) << 4.264, 7.964, 0)},
};

std::map<int, cv::Mat> camera_K_matrices_1080p = { // parameters for 2k
    {0, (cv::Mat_<double>(3, 3) << 1135.208474, 0.000000, 998.932805, 0.000000, 1133.531325, 530.084624, 0.000000, 0.000000, 1.000000)},
    {1, (cv::Mat_<double>(3, 3) << 1145.256303, 0.000000, 961.614618, 0.000000, 1141.658316, 514.490771, 0.000000, 0.000000, 1.000000)},
    {2, (cv::Mat_<double>(3, 3) << 1146.495853, 0.000000, 953.976032, 0.000000, 1146.011083, 571.868765, 0.000000, 0.000000, 1.000000)},
    {3, (cv::Mat_<double>(3, 3) << 1148.845354, 0.000000, 987.148727, 0.000000, 1136.968276, 523.721356, 0.000000, 0.000000, 1.000000)},
};


// ####### CAMERA 0
std::map<int, cv::Mat> camera0_rvec_list = {
    {6, (cv::Mat_<double>(3,3) << 0.9982754215668564, 0.0509068738342408, 0.02923478906952675,
     0.05327189195769921, -0.9947994945678074, -0.08681054737117029,
     0.02466349980765612, 0.08821822829812255, -0.9957957902969755)},
    {7, (cv::Mat_<double>(3,3) << 0.9988507344165356, 0.03396068337417401, 0.03382132966495603,
     0.03694864989539063, -0.9950694772203825, -0.09204092989134034,
     0.03052879995110165, 0.09318480288713325, -0.9951806795172582)},
    {8, (cv::Mat_<double>(3,3) << 0.9981049405950958, 0.05137281640929263, 0.0338727219726456,
     0.05430020754933951, -0.9942704191157407, -0.09207508420581591,
     0.02894848907555776, 0.09373989228491013, -0.9951757722003967)},
    {9, (cv::Mat_<double>(3,3) << 0.9979713808022055, 0.04451081458892242, 0.04551824342359094,
     0.04869704234579381, -0.9942446991185322, -0.09542576351002589,
     0.04100879377049116, 0.09744878480171006, -0.9943953002574777)}};

    std::map<int, cv::Mat> camera0_tvec_list = {
    {6, (cv::Mat_<double>(3,1) << 2.407983759300272,
     -0.7479610204652906,
     2.938676570170382)},
    {7, (cv::Mat_<double>(3,1) << -1.928470171069386,
     -0.6730976375220722,
     2.876701815717807)},
    {8, (cv::Mat_<double>(3,1) << 2.388503840602705,
     1.279111802006775,
     2.917276738435969)},
    {9, (cv::Mat_<double>(3,1) << -1.947289370048344,
     1.309780971833609,
     2.835603116704279)}};

// ####### CAMERA 1
std::map<int, cv::Mat> camera1_rvec_list = {
    {2, (cv::Mat_<double>(3,3) << 0.9987519365417029, 0.03259133926848195, -0.03784671529848555,
     0.03585864853820298, -0.995374210993264, 0.08913101264178994,
     -0.03476674530656482, -0.09037690354412453, -0.99530060219341)},
    {3, (cv::Mat_<double>(3,3) << 0.9983851456525554, 0.04485502716808124, -0.03485867866222981,
     0.0477709188147764, -0.9949840373320726, 0.08789029963517483,
     -0.03074150705346022, -0.08941360071107451, -0.9955200488950299)},
    {4, (cv::Mat_<double>(3,3) << 0.9991811667672679, 0.02444265702574283, -0.03224209197809979,
     0.02712398282323186, -0.9959661950135625, 0.08553144419458515,
     -0.03002141791134065, -0.08633594215463404, -0.9958136470037274)},
    {5, (cv::Mat_<double>(3,3) << 0.99805235833036, 0.0512993213255182, -0.03549464273307615,
     0.05479659352847051, -0.9928741784593035, 0.1058215435837096,
     -0.02981314087584642, -0.1075604266461629, -0.993751443395587)}};

    std::map<int, cv::Mat> camera1_tvec_list = {
    {2, (cv::Mat_<double>(3,1) << 2.388834761937858,
     -1.10037012660942,
     2.902427697608035)},
    {3, (cv::Mat_<double>(3,1) << -1.914143371952632,
     -1.07287455476528,
     2.909049699312688)},
    {4, (cv::Mat_<double>(3,1) << 2.394972875955088,
     0.9018697357130778,
     2.917067851246278)},
    {5, (cv::Mat_<double>(3,1) << -1.919060094608259,
     0.8739624658856628,
     2.917367423184139)}};

// ####### CAMERA 2
std::map<int, cv::Mat> camera2_rvec_list = {
    {4, (cv::Mat_<double>(3,3) << 0.999667990025002, 0.02087151785858865, 0.01510925086332736,
     0.02210987616678136, -0.9959578460442894, -0.08705815457904659,
     0.01323114111714999, 0.08736331406888243, -0.9960886447799914)},
    {5, (cv::Mat_<double>(3,3) << 0.999984081524295, 0.00197252304690791, 0.005286383531516525,
     0.00231858275089704, -0.9977985746460687, -0.06627690856022803,
     0.005144013223166782, 0.06628811045054009, -0.9977872646716117)},
    {6, (cv::Mat_<double>(3,3) << 0.9996072798748334, 0.0262306322611532, 0.009861032015821588,
     0.02679470232407414, -0.9977008004711873, -0.06225075635296775,
     0.008205482837783858, 0.06249053264561318, -0.9980118252711571)},
    {7, (cv::Mat_<double>(3,3) << 0.9999805640143973, 0.004337617803925409, -0.004478466839744363,
     0.003988194272794982, -0.9971541578772174, -0.0752839938806372,
     -0.004792275022373596, 0.07526466966621487, -0.9971520764658445)}};

    std::map<int, cv::Mat> camera2_tvec_list = {
    {4, (cv::Mat_<double>(3,1) << 2.328573090513532,
     -0.8878706328659157,
     2.980902932317242)},
    {5, (cv::Mat_<double>(3,1) << -1.981878075560448,
     -0.9083332359225564,
     2.913497498515037)},
    {6, (cv::Mat_<double>(3,1) << 2.309051397593271,
     1.055560399788938,
     2.92316377709065)},
    {7, (cv::Mat_<double>(3,1) << -1.919811662065577,
     1.129591757329826,
     2.894383620585697)}};

// ####### CAMERA 3
std::map<int, cv::Mat> camera3_rvec_list = {
    {0, (cv::Mat_<double>(3,3) << 0.9997433599978434, 0.022004496000714, 0.00538667763807304,
     0.02159922017480253, -0.9975647471927348, 0.06631778680064292,
     0.00683284919066404, -0.06618441896737996, -0.9977840121278192)},
    {1, (cv::Mat_<double>(3,3) << 0.999897877304451, -0.004542597158196036, 0.01354989937642784,
     -0.005402550268258533, -0.9979275907047251, 0.06411970181513219,
     0.01323054846376029, -0.06418635775085602, -0.9978502212586954)},
    {2, (cv::Mat_<double>(3,3) << 0.9997964989749848, 0.01891378795420988, 0.007016356788638836,
     0.01823058159377079, -0.9960251217401818, 0.08718717082925159,
     0.008637507285968484, -0.08704151588569342, -0.9961672389615137)},
    {3, (cv::Mat_<double>(3,3) << 0.9999292112480771, 0.006232385615752931, 0.0101355740998415,
     0.005291484270539345, -0.9958964481672521, 0.09034525290277524,
     0.01065704870081095, -0.09028522524415798, -0.9958589284710964)}};

    std::map<int, cv::Mat> camera3_tvec_list = {
    {0, (cv::Mat_<double>(3,1) << 2.383844395771065,
     -1.230386703845927,
     2.932055978972853)},
    {1, (cv::Mat_<double>(3,1) << -1.961402188527833,
     -1.188773814499419,
     2.89263715368315)},
    {2, (cv::Mat_<double>(3,1) << 2.342937310548115,
     0.7230954064621673,
     2.876931584588094)},
    {3, (cv::Mat_<double>(3,1) << -1.931606370225274,
     0.7279100696032805,
     2.844444528064798)}};

//ELIAS2025
void draw_tag_rotation(cv::Mat image, cv::Mat rvec, cv::Mat tvec, cv::Mat cameraMatrix, cv::Mat distCoeffs) {
    float axisLength = 0.16;

    // Origin and axes directions in tag coordinate space
    std::vector<cv::Point3f> axisPoints = {
        cv::Point3f(0, 0, 0),                  // origin
        cv::Point3f(axisLength, 0, 0),         // x-axis
        cv::Point3f(0, axisLength, 0),         // y-axis
        cv::Point3f(0, 0, -axisLength)         // z-axis (negative Z for OpenCV's convention)
    };
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
    cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 7); 
    cv::line(image, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 7);  
    cv::line(image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 7);  

    cv::putText(image, "X", imagePoints[1], cv::FONT_HERSHEY_DUPLEX, 4, cv::Scalar(0,0,255), 2);
    cv::putText(image, "Y", imagePoints[2], cv::FONT_HERSHEY_DUPLEX, 4, cv::Scalar(0,255,0), 2);
    cv::putText(image, "Z", imagePoints[3], cv::FONT_HERSHEY_DUPLEX, 4, cv::Scalar(255,0,0), 2);
}


cv::Mat estimate_object_global_position(int camera_id, cv::Mat undistorted_points, cv::Mat* global_position, cv::Mat* global_rotation, cv::Mat image) {
    // Estimating Object Global Position
    // ###################################################################3
    cv::Mat K = camera_K_matrices[camera_id];
    if (image.rows == 1080) {
        K = camera_K_matrices_1080p[camera_id]; // uses K matrix for 1080p
        // std::cout << "running global position on 1080p" << std::endl;
    }
    // else {
    //     std::cout << "running global position on 4k" << std::endl;
    // }
    cv::Mat distortion_coeffs = camera_global_distortion_coefficients[camera_id];
    double apriltag_size = 0.16;


    cv::Mat object_points = (cv::Mat_<double>(4,3) <<   -apriltag_size/2    ,  apriltag_size/2 , 0,
                                                         apriltag_size/2    ,  apriltag_size/2 , 0,
                                                         apriltag_size/2    , -apriltag_size/2 , 0,
                                                        -apriltag_size/2    , -apriltag_size/2 , 0);

    cv::Mat obj2cam_rvec = cv::Mat::zeros(1,3,CV_64F);
    cv::Mat obj2cam_tvec = cv::Mat::zeros(1,3,CV_64F);
    cv::solvePnP(object_points, undistorted_points, K, cv::Mat(), obj2cam_rvec, obj2cam_tvec);

    cv::Mat R_obj2cam;
    cv::Mat T_obj2cam = obj2cam_tvec;
    cv::Rodrigues(obj2cam_rvec, R_obj2cam); // turns obj2cam_rvec into a 3x3 rotation matrix R_obj2cam
    
    std::list<cv::Mat> all_global_positions;
    std::list<cv::Mat> all_R_obj2global;

    std::map<int, cv::Mat> cam2tag_rvec_list; // GET the 4 from a list of all these based on the given camera number - since each camera has 4 tags, each with different rvec and tvec based on camera
    std::map<int, cv::Mat> cam2tag_tvec_list; // GET same as the one above

    // CHANGE AND RECALIBRATE FOR THE RIGHT CAMERAS NOW THAT I KNOW THE CORRECT ORDER
    switch(camera_id) {
        case 0: cam2tag_rvec_list = camera0_rvec_list;
                cam2tag_tvec_list = camera0_tvec_list;
                break;
        case 2: cam2tag_rvec_list = camera1_rvec_list;
                cam2tag_tvec_list = camera1_tvec_list;
                break;
        case 1: cam2tag_rvec_list = camera2_rvec_list; 
                cam2tag_tvec_list = camera2_tvec_list;
                break;
        case 3: cam2tag_rvec_list = camera3_rvec_list;
                cam2tag_tvec_list = camera3_tvec_list;
                break;
    }


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
#if ELIAS_PRINT
        std::cout << "Object position from tag " << list.first << ": " << global_positions << std::endl;
#endif        
    // std::cout << "R from tag " << list.first << ": " << R_obj2global << std::endl;
    }
    // get average of all_global_positions and all_R_obj2global
    cv::Mat average_global_positions = average_mat(all_global_positions);
    cv::Mat average_R_obj2global = average_mat(all_R_obj2global);
    
    //std::cout << "Object position from all tags: " << average_global_positions << std::endl;
#if ELIAS_PRINT
    std::cout << "R from all tags: " << average_R_obj2global << std::endl;
#endif

    // returns
    global_position = &average_global_positions;
    global_rotation = &average_R_obj2global;

    draw_tag_rotation(image, obj2cam_rvec, obj2cam_tvec, K, distortion_coeffs);

    // results_global_position.insert({camera_id, average_global_positions});
    return average_global_positions;
}

cv::Mat global_to_pixel(int camera_id, cv::Mat global_position, cv::Mat image) {
    cv::Mat distorted_points;
    cv::Mat K = camera_K_matrices[camera_id];
    if (image.rows == 1080) {
        K = camera_K_matrices_1080p[camera_id]; 
    }
    cv::Mat distortion_coeffs = camera_global_distortion_coefficients[camera_id];
    
    cv::projectPoints(global_position, cv::Vec3d(0,0,0), cv::Vec3d(0,0,0), K, cv::Mat(), distorted_points);
    return distorted_points;
}