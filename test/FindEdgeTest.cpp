//
// Created by whubooks on 18-3-25.
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <sophus/se3.h>
#include <opencv2/opencv.hpp>

#include <PointCloudProject.h>

typedef Eigen::Matrix<double,3,4> Matrix34d;
typedef std::vector<Matrix34d,Eigen::aligned_allocator<Matrix34d>> Matrix34dVec;

Matrix34d LoadCalibFile(std::string filename)
{
    Matrix34d transform;

    std::ifstream ifs(filename,std::ios::in);
    if(!ifs.is_open())
        return transform;

    while(ifs.good()&&!ifs.eof())
    {
        std::string str;
        double x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12;
        ifs >> str >> x1 >> x2 >> x3 >> x4 >> x5 >> x6 >> x7 >> x8 >> x9 >> x10 >> x11 >> x12;

        if(str=="P0:")
        {
            transform << x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12;
            break;
        }
    }
    ifs.close();

    return transform;
}

float ComputeDiff(const cv::Mat &img)
{
    std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> vnormal;
    std::vector<float> vdepth;

    for(int i=0;i<2;i++)
    {
        for(int j=0;j<2;j++)
        {
            cv::Vec4f val=img.at<cv::Vec4f>(i,j);

            vnormal.push_back(Eigen::Vector3f(val[0],val[1],val[2]));
            vdepth.push_back(val[3]);
        }
    }

    Eigen::Vector3f normal_ave(0.0f,0.0f,0.0f);
    for(const Eigen::Vector3f &normal : vnormal)
        normal_ave+=normal;
    normal_ave/=9.0f;

    float normal_std=0.0f;
    for(const Eigen::Vector3f &normal : vnormal)
        normal_std+=(normal-normal_ave).transpose()*(normal-normal_ave);
    normal_std=std::sqrt(normal_std/8.0f);

    float depth_ave=std::accumulate(vdepth.begin(),vdepth.end(),0.0)/9.0;
    float depth_diff=std::abs(vdepth[4]-depth_ave);

    float lambda_depth=0.5f;
    float lambda_normal=0.5f;

    float diff=lambda_normal*normal_std+lambda_depth*depth_diff;

    return diff;

}


int main()
{
    std::string pcdfile = "voxel.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile, *cloud);

    std::string imgfile = "/home/whubooks/kitti_odometry/data_odometry_gray/sequences/00/image_0/000020.png";
    std::string calibfile = "/home/whubooks/kitti_odometry/data_odometry_calib/sequences/00/calib.txt";

    cv::Mat img_gray_origin = cv::imread(imgfile, CV_LOAD_IMAGE_GRAYSCALE);
    int rows = img_gray_origin.rows;
    int cols = img_gray_origin.cols;

    Matrix34d calib=LoadCalibFile(calibfile);

    Eigen::Matrix3d K=calib.block(0,0,3,3);

    PCM::PointCloudProject pointCloudProject;
    pointCloudProject.SetInput(cloud);
    pointCloudProject.SetK(K);
    pointCloudProject.SetImageSize(rows,cols);
    pointCloudProject.Project2();

    std::vector<cv::Vec4f> normal_depth_vec=pointCloudProject.GetVec();
    cv::Mat img_normal_depth(rows,cols,CV_32FC4,normal_depth_vec.data());
    cv::imshow("img_normal_depth",img_normal_depth);

    cv::waitKey(0);
}