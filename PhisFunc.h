//
// Created by hyacinth on 2017/4/17.
//
#ifndef SDF2SDF_PHISFUNC_H
#define SDF2SDF_PHISFUNC_H
#include <opencv/ml.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "myPoint.h"

double PhisFunc(double fx,double fy,double cx,double cy,
               cv::Mat &depth_image, const myPoint &point, Eigen::Matrix<double, 6, 1> &twist,
                double delta, double eta, double &weight, bool needTwist);

#endif //SDF2SDF_PHISFUNC_H