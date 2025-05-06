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

// ####### CAMERA 0
std::map<int, cv::Mat> camera0_rvec_list = {
    // ####### first tag of camera
    {6, (cv::Mat_<double>(3,3) << 0.9982754215668566, -0.05327189195770059, -0.02466349980765093,
        0.05090687383424264, 0.9947994945678074, -0.08821822829812229,
        0.02923478906952168, 0.08681054737117028, 0.995795790296975)},
    // ####### second tag of camera
    {7, (cv::Mat_<double>(3,3) << 0.9988507344159444, -0.03694864989416633, -0.03052879997194176,
        0.033960683371177, 0.9950694772208781, -0.09318480288293261,
        0.03382132968543919, 0.09204092988647315, 0.9951806795170123)},
    // ####### third tag of camera
    {8, (cv::Mat_<double>(3,3) << 0.9981049405846116, -0.05430020767303696, -0.02894848920503302,
        0.05137281650868881, 0.9942704190776626, -0.09373989263432007,
        0.03387272213084513, 0.09207508454405199, 0.995175772163718)},
    // ####### fourth tag of camera
    {9, (cv::Mat_<double>(3,3) << 0.9979713808022064, -0.04869704234579101, -0.04100879377048147,
        0.04451081458892053, 0.9942446991185322, -0.09744878480171061,
        0.04551824342358109, 0.095425763510027, 0.994395300257478)}};

std::map<int, cv::Mat> camera0_tvec_list = {
    {6, (cv::Mat_<double>(3,1) << 2.453996187821951,
        0.7622532692639873,
        -2.994829625651334)},
    {7, (cv::Mat_<double>(3,1) << -1.965319919610101,
        0.6859593758266564,
        -2.931670640159325)},
    {8, (cv::Mat_<double>(3,1) << 2.434144041067037,
        -1.30355343024622,
        -2.973020880188064)},
    {9, (cv::Mat_<double>(3,1) << -1.984498721068342,
        -1.334808633715786,
        -2.889786615749597)}
};

// ####### CAMERA 1
std::map<int, cv::Mat> camera1_rvec_list = {
    // ####### first tag of camera
    {4, (cv::Mat_<double>(3,3) << 0.9996679900250022, -0.02210987616678538, -0.01323114111713982,
        0.02087151785859337, 0.9959578460442883, -0.08736331406889489,
        0.01510925086331782, 0.08705815457905919, 0.9960886447799905)},
    // ####### second tag of camera
    {5, (cv::Mat_<double>(3,3) << 0.9999840815242946, -0.002318582750896286, -0.005144013223164418,
        0.001972523046907283, 0.9977985746460684, -0.06628811045054497,
        0.005286383531514127, 0.06627690856023291, 0.9977872646716114)},
    // ####### third tag of camera
    {6, (cv::Mat_<double>(3,3) << 0.9996072798748327, -0.02679470232406529, -0.008205482837873188,
        0.02623063226113956, 0.9977008004711923, -0.0624905326455382,
        0.009861032015908166, 0.06225075635289053, 0.9980118252711609)},
    // ####### fourth tag of camera
    {7, (cv::Mat_<double>(3,3) << 0.9999805640143538, -0.003988194267570316, 0.004792275035821712,
        0.004337617799780566, 0.9971541578763513, -0.0752646696779285,
        -0.004478466853496578, 0.0752839938923858, 0.9971520764648958)}};

std::map<int, cv::Mat> camera1_tvec_list = {
    {4, (cv::Mat_<double>(3,1) << 2.373068117720825,
        0.9048363137486537,
        -3.037862860960227)},
    {5, (cv::Mat_<double>(3,1) << -2.019748357259049,
        0.9256899219592776,
        -2.969169425238265)},
    {6, (cv::Mat_<double>(3,1) << 2.353173398821032,
        -1.075730343733752,
        -2.979020409774344)},
    {7, (cv::Mat_<double>(3,1) << -1.956495961298937,
        -1.151176313249528,
        -2.949690313990945)}
};

// ####### CAMERA 2
std::map<int, cv::Mat> camera2_rvec_list = {
    // ####### first tag of camera
    {2, (cv::Mat_<double>(3,3) << 0.9987519365425863, -0.03585864853225482, 0.03476674528732782,
        0.03259133926376021, 0.9953742109921994, 0.09037690355755193,
        -0.03784671527924444, -0.08913101265607151, 0.9953006021928626)},
    // ####### second tag of camera
    {3, (cv::Mat_<double>(3,3) << 0.998385145652555, -0.04777091881478688, 0.03074150705345484,
        0.04485502716809122, 0.9949840373320697, 0.08941360071110148,
        -0.0348586786622266, -0.08789029963520166, 0.9955200488950275)},
    // ####### third tag of camera
    {4, (cv::Mat_<double>(3,3) << 0.9991811667672719, -0.02712398282322215, 0.03002141791119776,
        0.02444265702574148, 0.9959661950135521, 0.08633594215475456,
        -0.03224209197795959, -0.0855314441947095, 0.9958136470037213)},
    // ####### fourth tag of camera
    {5, (cv::Mat_<double>(3,3) << 0.9980523583303599, -0.05479659352848509, 0.02981314087581119,
        0.05129932132553523, 0.9928741784592988, 0.1075604266461969,
        -0.03549464273304445, -0.1058215435837448, 0.9937514433955843)}};

std::map<int, cv::Mat> camera2_tvec_list = {
    {2, (cv::Mat_<double>(3,1) << 2.434481286035945,
        1.121396307451914,
        -2.957888099560511)},
    {3, (cv::Mat_<double>(3,1) << -1.950719359951715,
        1.093375342436046,
        -2.964636636242171)},
    {4, (cv::Mat_<double>(3,1) << 2.440736688871228,
        -0.9191029153760595,
        -2.972808001270797)},
    {5, (cv::Mat_<double>(3,1) << -1.955730032721906,
        -0.8906623856158302,
        -2.97311329751247)}
};

// ####### CAMERA 3
std::map<int, cv::Mat> camera3_rvec_list = {
    // ####### first tag of camera
    {0, (cv::Mat_<double>(3,3) << 0.999743359997857, -0.02159922017479574, -0.006832849188732689,
        0.02200449600058381, 0.9975647471926803, 0.06618441896824361,
        0.005386677636127802, -0.06631778680146382, 0.9977840121277751)},
    // ####### second tag of camera
    {1, (cv::Mat_<double>(3,3) << 0.9998978773044658, 0.005402550267898251, -0.01323054846279599,
        -0.004542597157886907, 0.9979275907046726, 0.06418635775169484,
        0.01354989937544624, -0.06411970181598034, 0.9978502212586541)},
    // ####### third tag of camera
    {2, (cv::Mat_<double>(3,3) << 0.9997964989749846, -0.01823058159370224, -0.008637507286197737,
        0.01891378795416365, 0.9960251217401568, 0.08704151588598906,
        0.007016356788867538, -0.08718717082955106, 0.9961672389614858)},
    // ####### fourth tag of camera
    {3, (cv::Mat_<double>(3,3) << 0.9999292112480768, -0.005291484270575522, -0.01065704870089764,
        0.006232385615797672, 0.995896448167244, 0.09028522524424336,
        0.01013557409992402, -0.09034525290286154, 0.9958589284710877)}};

std::map<int, cv::Mat> camera3_tvec_list = {
    {0, (cv::Mat_<double>(3,1) << 2.429395562570312,
        1.253897277804775,
        -2.988082526330146)},
    {1, (cv::Mat_<double>(3,1) << -1.998881211235661,
        1.211489237709884,
        -2.947910475092573)},
    {2, (cv::Mat_<double>(3,1) << 2.387706813297188,
        -0.7369125161388561,
        -2.931904799581411)},
    {3, (cv::Mat_<double>(3,1) << -1.9685160460897,
        -0.741819179213199,
        -2.898796971276075)}
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
        std::cout << "Object position from tag " << list.first << ": " << global_positions << std::endl;
        // std::cout << "R from tag " << list.first << ": " << R_obj2global << std::endl;
    }
    // get average of all_global_positions and all_R_obj2global
    cv::Mat average_global_positions = average_mat(all_global_positions);
    cv::Mat average_R_obj2global = average_mat(all_R_obj2global);
    
    //std::cout << "Object position from all tags: " << average_global_positions << std::endl;
    std::cout << "R from all tags: " << average_R_obj2global << std::endl;

    // returns
    global_position = &average_global_positions;
    global_rotation = &average_R_obj2global;

    draw_tag_rotation(image, obj2cam_rvec, obj2cam_tvec, K, distortion_coeffs);
    
    return average_global_positions;
}

