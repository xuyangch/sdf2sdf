//
// Created by hyacinth on 2017/4/28.
//
#ifndef SDF2SDF_SELFCROSS_H
#define SDF2SDF_SELFCROSS_H

#include <opencv/ml.h>
#include "common.h"
#include <Eigen/Dense>

Eigen::Matrix<double , 3, 3> selfCross(Eigen::Vector4d &point);
#endif //SDF2SDF_SELFCROSS_H