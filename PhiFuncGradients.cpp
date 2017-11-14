//
// Created by hyacinth on 2017/4/17.
//
#include "PhiFuncGradients.h"
#include "PhisFunc.h"
#include "sophus/se3.hpp"
Eigen::Matrix<double, 1, 6> PhiFuncGradients(double fx,double fy,double cx,double cy,
                      cv::Mat &depth_image, const myPoint &point, Eigen::Matrix<double, 6, 1> &twist,
                      double delta, double eta, double &weight)
{
    myPoint preX, postX;
    double prePhi, postPhi;
    //double epsilon = +1.0e-5;
    double epsilon = 4.0e-3;
    Eigen::Matrix<double, 1, 3> gradient;
    Eigen::Matrix<double, 6, 1> preTwist, postTwist;

    //partial phi / partial x
    preX = point;
    preX.x = point.x - epsilon;
    prePhi = PhisFunc(fx,fy,cx,cy,depth_image,preX,twist,delta,eta,weight,false);
    postX = point;
    postX.x = point.x + epsilon;
    postPhi = PhisFunc(fx,fy,cx,cy,depth_image,postX,twist,delta,eta,weight,false);
    gradient(0) = (postPhi - prePhi) / (2 * epsilon);
    if ((postPhi < -1) || (prePhi < -1)) gradient(0) = 0;
    //partial phi / partial y
    preX = point;
    preX.y = point.y - epsilon;
    prePhi = PhisFunc(fx,fy,cx,cy,depth_image,preX,twist,delta,eta,weight,false);
    postX = point;
    postX.y = point.y + epsilon;
    postPhi = PhisFunc(fx,fy,cx,cy,depth_image,postX,twist,delta,eta,weight,false);
    gradient(1) = (postPhi - prePhi) / (2 * epsilon);
    if ((postPhi < -1) || (prePhi < -1)) gradient(1) = 0;
    //partial phi / partial z
    preX = point;
    preX.z = point.z - epsilon;
    prePhi = PhisFunc(fx,fy,cx,cy,depth_image,preX,twist,delta,eta,weight,false);
    postX = point;
    postX.z = point.z + epsilon;
    postPhi = PhisFunc(fx,fy,cx,cy,depth_image,postX,twist,delta,eta,weight,false);
    gradient(2) = (postPhi - prePhi) / (2 * epsilon);
    if ((postPhi < -1) || (prePhi < -1)) gradient(2) = 0;

    //get the inverse of twist
    Sophus::SE3d se = Sophus::SE3d::exp(twist);
    Eigen::Matrix<double, 4, 4> inverse_homogenous = (se.inverse()).matrix();

    //apply inverse to point
    Eigen::Vector4d trans_point (point.x, point.y, point.z, 1);
    trans_point = inverse_homogenous * trans_point;
    Eigen::Matrix<double , 3, 6> concatenate_matrix;

    //concatenate the Identity and inversed point
    concatenate_matrix<<Eigen::MatrixXd::Identity(3,3),-selfCross(trans_point);

    //calculate the final twist partial
    Eigen::Matrix<double, 1, 6> twist_partial = gradient * concatenate_matrix;

    /*Eigen::Matrix<double, 1, 6> twist_partial2;

    //comparison
    //partial phi / partial

    epsilon = +1.0e-4;
    for (int i = 0; i < 6; i++)
    {
        preTwist = twist;
        preTwist(i) = twist(i) - epsilon;
        prePhi = PhisFunc(fx, fy, cx, cy, depth_image, point, preTwist, delta, eta, weight, true);
        postTwist = twist;
        postTwist(i) = postTwist(i) + epsilon;
        postPhi = PhisFunc(fx, fy, cx, cy, depth_image, point, postTwist, delta, eta, weight, true);
        twist_partial2(i) = (postPhi - prePhi) / (2 * epsilon);
    }
*/
    return  twist_partial;
}