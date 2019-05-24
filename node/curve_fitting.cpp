//
// Created by adam on 19-4-18.
//

#include "curve_fitting.h"

// CurveFitting::CurveFitting():nh_() {
// }


// void CurveFitting::init_parameters() {
//     ros::NodeHandle private_nh("~");
//     private_nh.getParam("", );
// }

// void CurveFitting::run() {
//     this->init_parameters();

// }


// //Using SVD to solve the coefficients of the curve.
// bool CurveFitting::lane_coef_estimate()
// {
//     //To fitting the lance curve by using least square method
//     // 重要参数：进行多项式拟合的检测特征点阈值
//     int countThreshold = 300;
//     if ((laneLcount > countThreshold) && (laneRcount > countThreshold))
//     {
//         Eigen::VectorXd xValueL(laneLcount);
//         Eigen::VectorXd xValueR(laneRcount);
//         Eigen::MatrixXd leftMatrix(laneLcount, 3);
//         Eigen::MatrixXd rightMatrix(laneRcount, 3);

//         //left lane curve coefficients estimation
//         for (int i = 0; i < laneLcount; i++)
//         {
//             xValueL(i) = laneL[i].x;
//             leftMatrix(i, 0) = pow(laneL[i].y, 2);
//             leftMatrix(i, 1) = laneL[i].y;
//             leftMatrix(i, 2) = 1;
//         }

//         //right lane curve coefficients estimation
//         for (int i = 0; i < laneRcount; i++)
//         {
//             xValueR(i) = laneR[i].x;
//             rightMatrix(i, 0) = pow(laneR[i].y, 2);
//             rightMatrix(i, 1) = laneR[i].y;
//             rightMatrix(i, 2) = 1;
//         }
//         curveCoefL = (leftMatrix.transpose() * leftMatrix).ldlt().solve(leftMatrix.transpose() * xValueL);
//         curveCoefR = (rightMatrix.transpose() * rightMatrix).ldlt().solve(rightMatrix.transpose() * xValueR);

//         curveCoefRecordL[recordCounter] = curveCoefL;
//         curveCoefRecordR[recordCounter] = curveCoefR;
//         recordCounter = (recordCounter + 1) % 5;
//         if (initRecordCount < 5)
//             initRecordCount++;
//         failDetectFlag = false;
//         return true;
//     }
//     else
//     {
//         cerr << "[Lane Detection Algo] There is no enough detected road marks.";
//         failDetectFlag = true;
//         return false;
//     }
// }

// //To fit the lane.
// void CurveFitting::lane_fitting()
// {
//     maskImage.create(mergeImage.size().height, mergeImage.size().width, CV_8UC3);
//     maskImage = cv::Scalar(0, 0, 0);
//     curvePointsL.clear();
//     curvePointsR.clear();

//     //To average the past 5 estimated coefficients.
//     if (initRecordCount == 5)
//     {
//         curveCoefL = (curveCoefRecordL[0] + curveCoefRecordL[1] + curveCoefRecordL[2] + curveCoefRecordL[3] +
//                       curveCoefRecordL[4]) /
//                      5;
//         curveCoefR = (curveCoefRecordR[0] + curveCoefRecordR[1] + curveCoefRecordR[2] + curveCoefRecordR[3] +
//                       curveCoefRecordR[4]) /
//                      5;
//     }

//     int xL, xR;
//     for (int i = 0; i < mergeImage.size().height; i++)
//     {
//         xL = pow(i, 2) * curveCoefL(0) + i * curveCoefL(1) + curveCoefL(2);
//         xR = pow(i, 2) * curveCoefR(0) + i * curveCoefR(1) + curveCoefR(2);
//         if (xL < 0)
//             xL = 0;
//         if (xL >= mergeImage.size().width)
//             xL = mergeImage.size().width - 1;
//         if (xR < 0)
//             xR = 0;
//         if (xR >= mergeImage.size().width)
//             xR = mergeImage.size().width - 1;
//         curvePointsL.push_back(cv::Point2f(xL, i));
//         curvePointsR.push_back(cv::Point2f(xR, i));
//     }
//     leftLanePos = curvePointsL.back().x;
//     rightLanePos = curvePointsR.back().x;

//     cv::UMat curveL(curvePointsL, true);
//     curveL.convertTo(curveL, CV_32S);
//     cv::polylines(maskImage, curveL, false, cv::Scalar(255, 0, 0), 20, CV_AA);
//     cv::UMat curveR(curvePointsR, true);
//     curveR.convertTo(curveR, CV_32S);
//     cv::polylines(maskImage, curveR, false, cv::Scalar(0, 0, 255), 20, CV_AA);
//     line(maskImage, cv::Point2f(leftLanePos, 0), cv::Point2f(leftLanePos, mergeImageRGB.size().height),
//          cv::Scalar(0, 255, 0), 5);
//     line(maskImage, cv::Point2f(rightLanePos, 0), cv::Point2f(rightLanePos, mergeImageRGB.size().height),
//          cv::Scalar(0, 255, 0), 5);
//     // draw the center line
//     line(maskImage, cv::Point2f(imageCenter, 0), cv::Point2f(imageCenter, mergeImageRGB.size().height),
//          cv::Scalar(0, 255, 0), 5);
// }


int main(int argc, char **argv){
    // ros::init(argc, argv, "curve_fitting");
    // CurveFitting curveFitting;
    // ros::spin();

}

