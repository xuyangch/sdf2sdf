#include <opencv/ml.h>
#include <opencv/highgui.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv/cvaux.h>
#include "PhiFuncGradients.h"
#include "sophus/se3.hpp"
#include <pcl/common/transforms.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#define FX 570.3999633789062
#define FY 570.39996337890622
#define CX 320.0
#define CY 240.0
#define DELTA 0.002
double BETA = 0.5;
//expected object thickness
#define ETA 0.01

float SIDE_LENGTH = 0.004;

cv::Mat load_exr_depth( std::string filename ) {
    // load the image
    cv::Mat depth_map = cv::imread( filename, -1 );
    cv::cvtColor( depth_map, depth_map, CV_RGB2GRAY );

    // convert to meters
    depth_map.convertTo( depth_map, CV_32FC1, 0.001 );

    return depth_map;
}

void DepthFrameToVertex(float fx,float fy,float cx,float cy,
                        cv::Mat &depth_image, pcl::PointCloud<pcl::PointXYZ>::Ptr target_pc, bool organized)
{
    float* pixel_ptr;
    target_pc->height = (uint32_t)depth_image.rows;
    target_pc->width = (uint32_t)depth_image.cols;
    target_pc->is_dense = false;
    target_pc->resize(target_pc->height * target_pc->width);
    for (int y = 0; y < depth_image.rows; ++y)
    {
        for  (int x = 0; x < depth_image.cols; ++x)
        {
            float z = depth_image.at<float>(y,x);
            target_pc->at(x, y).x = (x - cx) * z / fx;
            target_pc->at(x, y).y = (y - cy) * z / fy;
            target_pc->at(x, y).z = z;
            if (!isinf(z))
            {
                //printf("point(%f,%f,%f)\n",target_pc->at(y,x).x,target_pc->at(y,x).y,target_pc->at(y,x).z);
            }
            //++pixel_ptr;
        }
    }
}
pcl::PointXYZ getVoxel(pcl::PointXYZ &point, pcl::PointXYZ &c, float l)
{
    float x = (float) round((1/l) * (point.x - c.x)-(0.5));
    float y = (float) round((1/l) * (point.y - c.y)-(0.5));
    float z = (float) round((1/l) * (point.z - c.z)-(0.5));
    return pcl::PointXYZ(x,y,z);
}
pcl::PointXYZ getVoxelCenter(pcl::PointXYZ point, pcl::PointXYZ c, float l)
{
    pcl::PointXYZ voxel = getVoxel(point, c, l);
    float x = (float) (l * (voxel.x + 0.5) + c.x);
    float y = (float) (l * (voxel.y + 0.5) + c.y);
    float z = (float) (l * (voxel.z + 0.5) + c.z);
    return pcl::PointXYZ(x,y,z);
}
pcl::PointXYZ getVoxelCenterByVoxel(pcl::PointXYZ point, pcl::PointXYZ c, float l)
{
    float x = (float) (l * (point.x + 0.5) + c.x);
    float y = (float) (l * (point.y + 0.5) + c.y);
    float z = (float) (l * (point.z + 0.5) + c.z);
    return pcl::PointXYZ(x,y,z);
}

void getLowerLeftAndUpperRight(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                               pcl::PointXYZ &pointll, pcl::PointXYZ &pointur)
{
    pointll.x = INFINITY; pointll.y = INFINITY; pointll.z = INFINITY;
    pointur.x = -INFINITY; pointur.y = -INFINITY; pointur.z = -INFINITY;
    for (int x = 0; x < point_cloud->height; ++x) {
        for (int y = 0; y < point_cloud->width; ++y) {
            pcl::PointXYZ point = point_cloud->at(y,x);
            if ((!isinf(point.x)) && (!isinf(-point.x)) && (point.x < pointll.x)) pointll.x = point.x;
            if ((!isinf(point.y)) && (!isinf(-point.y)) && (point.y < pointll.y)) pointll.y = point.y;
            if ((!isinf(point.z)) && (!isinf(-point.z)) && (point.z < pointll.z)) pointll.z = point.z;
            if ((!isinf(point.x)) && (!isinf(-point.x)) && (point.x > pointur.x)) pointur.x = point.x;
            if ((!isinf(point.y)) && (!isinf(-point.y)) && (point.y > pointur.y)) pointur.y = point.y;
            if ((!isinf(point.z)) && (!isinf(-point.z)) && (point.z > pointur.z)) pointur.z = point.z;
        }
    }
}

bool isValid(pcl::PointXYZ &point)
{
    return ((!isinf(point.x)) && (!isinf(point.y)) && (!isinf(point.z)) &&
            (!isinf(-point.x)) && (!isinf(-point.y)) && (!isinf(-point.z)));
}

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr ref_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tar_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_tar_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    //get depth map
    cv::Mat depth_map_ref = load_exr_depth("Synthetic_Kenny_Circle/depth_000000.exr");
    cv::Mat depth_map_tar = load_exr_depth("Synthetic_Kenny_Circle/depth_000003.exr");

    //get point cloud
    DepthFrameToVertex(FX,FY,CX,CY,depth_map_ref,ref_point_cloud,0);
    DepthFrameToVertex(FX,FY,CX,CY,depth_map_tar,tar_point_cloud,0);

    //get the lower left and upper left points
    pcl::PointXYZ pointll, pointur;
    getLowerLeftAndUpperRight(ref_point_cloud,pointll,pointur);
    printf("pointLL(%f,%f,%f)\n",pointll.x,pointll.y,pointll.z);
    printf("pointUR(%f,%f,%f)\n",pointur.x,pointur.y,pointur.z);
    /* Result
     pointLL(-0.112570,0.013829,0.449250)
     pointUR(0.013296,0.124222,0.501750)
     */

    //padding...
    pointll.x -= 2 * SIDE_LENGTH; pointll.y -= 2 * SIDE_LENGTH; pointll.z -= 2 * SIDE_LENGTH;
    pointur.x += 2 * SIDE_LENGTH; pointur.y += 2 * SIDE_LENGTH; pointur.z += 2 * SIDE_LENGTH;

    printf("After padding......\n");
    printf("pointLL(%f,%f,%f)\n",pointll.x,pointll.y,pointll.z);
    printf("pointUR(%f,%f,%f)\n",pointur.x,pointur.y,pointur.z);
    /* Result
     pointLL(-0.113570,0.012829,0.448250)
     pointUR(0.014296,0.125222,0.502750)
     */

    //initial twist
    Eigen::Matrix<double ,6,1> initial_twist = Eigen::MatrixXd::Zero(6,1);

    //optimization
    pcl::PointXYZ maxVoxel = getVoxel(pointur, pointll, SIDE_LENGTH);
    int max_x = (int) maxVoxel.x;
    int max_y = (int) maxVoxel.y;
    int max_z = (int) maxVoxel.z;
    double weight_ref, weight_tar, weight_temp;

    for (int iter = 0; iter < 60; iter ++) {
        //if (iter == 40) BETA *= 0.1;
        //if (iter >= 40) SIDE_LENGTH = 0.002;
        Eigen::Matrix<double, 6, 6> A = Eigen::MatrixXd::Zero(6,6);
        Eigen::Matrix<double, 6, 1> b = Eigen::MatrixXd::Zero(6,1);
        double error = 0;
        for (int i = 0; i < max_x; i++) {
            for (int j = 0; j < max_y; j++) {
                for (int k = 0; k < max_z; k++) {
                    pcl::PointXYZ intPoint(i, j, k);
                    pcl::PointXYZ floatPoint = getVoxelCenterByVoxel(intPoint, pointll, SIDE_LENGTH);
                    myPoint voxel(floatPoint.x, floatPoint.y, floatPoint.z);
                    double PhiRef = PhisFunc(FX, FY, CX, CY, depth_map_ref,
                                             voxel, initial_twist,
                                             DELTA, ETA, weight_ref, false);
                    double PhiTar = PhisFunc(FX, FY, CX, CY, depth_map_tar,
                                             voxel, initial_twist,
                                             DELTA, ETA, weight_tar, true);

                    if ((PhiRef < -1) || (PhiTar < -1)) continue;
                    Eigen::Matrix<double, 1, 6> gradient = PhiFuncGradients(FX, FY, CX, CY, depth_map_tar,
                                                                            voxel, initial_twist,
                                                                            DELTA, ETA, weight_temp);
                    A += gradient.transpose() * gradient;
                    b += (PhiRef -
                          PhiTar +
                          gradient * initial_twist) *
                         gradient.transpose();
                    if ((PhiRef * weight_ref - PhiTar * weight_tar) *
                        (PhiRef * weight_ref - PhiTar * weight_tar) != 0)
                    error += 0.5 * (PhiRef * weight_ref - PhiTar * weight_tar) *
                             (PhiRef * weight_ref - PhiTar * weight_tar);
                }
            }
        }
        Eigen::Matrix<double, 6, 1> inter_twist = A.inverse() * b;
        initial_twist += BETA * (inter_twist - initial_twist);
        printf("%d th, error = %lf\n", iter, error);
        printf("twist is : ");
        for (int l = 0; l < 6; l ++)
            printf("%f \n",initial_twist(l,0));
    }

    //convert the target to reference
    //get the reverse of reference position
    Sophus::SE3d se = Sophus::SE3d::exp(initial_twist);
    Eigen::Matrix<double, 4, 4> inverse_homogenous = (se.inverse()).matrix();
    pcl::transformPointCloud (*tar_point_cloud, *final_tar_point_cloud, inverse_homogenous);

    // Visualization
    pcl::visualization::PCLVisualizer viewer ("Matrix transformation");

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (ref_point_cloud, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (ref_point_cloud, source_cloud_color_handler, "ref_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (final_tar_point_cloud, 230, 20, 20); // Red
    viewer.addPointCloud (final_tar_point_cloud, transformed_cloud_color_handler, "target_cloud");

    viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "ref_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud");
    viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }


    /*
    //Visualization
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (target_point_cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    */
}