//
// Created by hyacinth on 2017/4/17.
//
#include "PhisFunc.h"
#include "sophus/se3.hpp"
#include "common.h"
bool validRow(int x)
{
    return ((x>=0) && (x<=480));
}
bool validCol(int y)
{
    return ((y>=0) && (y<=640));
}
double PhisFunc(double fx,double fy,double cx,double cy,
                cv::Mat &depth_image, const myPoint &point, Eigen::Matrix<double, 6, 1> &twist,
                double delta, double eta, double &weight, bool needTwist)
{
    //transform to local coordinates
    Sophus::SE3d se (Sophus::SE3d::exp(twist));
    Eigen::Matrix<double , 4, 4> homogenous = se.matrix();
    Eigen::Vector4d trans_point (point.x, point.y, point.z, 1);
    if (needTwist)
        trans_point = homogenous * trans_point;
    myPoint t_point(trans_point(0),trans_point(1),trans_point(2));
    //calculate SDF function
    int px = int(fx*t_point.x/t_point.z + cx);
    int py = int(fy*t_point.y/t_point.z + cy);
    if (!(validRow(py) && validCol(px)))
    {
        return -999;
    }
    double trueSDF = depth_image.at<float>(py,px) - t_point.z;
    double SDF;
    if (trueSDF >= delta) SDF = 1;
    else if (trueSDF <= -delta) SDF = -1;
    else SDF = trueSDF / delta;
    weight = 0;
    if (trueSDF > -eta) weight = 1;
    return SDF;
}
