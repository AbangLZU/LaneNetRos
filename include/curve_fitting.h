//
// Created by adam on 19-4-18.
//

#ifndef SRC_CURVE_FITTING_H
#define SRC_CURVE_FITTING_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "lane_detector/Lane_Image.h"


using namespace std;

class CurveFitting {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_lane_image_;
    ros::Publisher pub_curve_;

    string lane_image_topic_;
    string curves_topic_;

    void init_parameters();
    bool lane_coef_estimate();

    void lane_fitting();

    void run();

public:
    CurveFitting();
};


#endif //SRC_CURVE_FITTING_H
