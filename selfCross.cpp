//
// Created by hyacinth on 2017/4/28.
//
#include "selfCross.h"
Eigen::Matrix<double, 3, 3> selfCross(Eigen::Vector4d &point)
{
    Eigen::Matrix<double, 3, 3> ans = Eigen::MatrixXd::Zero(3,3);
    ans(0,1) = -point(2);
    ans(0,2) = point(1);
    ans(1,0) = point(2);
    ans(1,2) = -point(0);
    ans(2,0) = -point(1);
    ans(2,1) = point(0);
    return ans;
};
