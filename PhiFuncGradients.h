//
// Created by hyacinth on 2017/4/28.
//

#ifndef SDF2SDF_PHIFUNCGRADIENTS_H
#define SDF2SDF_PHIFUNCGRADIENTS_H

#include "common.h"
#include "PhisFunc.h"
#include "selfCross.h"
#include "myPoint.h"
Eigen::Matrix<double, 1, 6> PhiFuncGradients(double fx,double fy,double cx,double cy,
                                            cv::Mat &depth_image, const myPoint &point, Eigen::Matrix<double, 6, 1> &twist,
                                             double delta, double eta, double &weight);

#endif //SDF2SDF_PHIFUNCGRADIENTS_H