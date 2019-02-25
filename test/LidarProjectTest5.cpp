//
// Created by whubooks on 18-3-24.
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

    std::vector<uchar> depth_gray_vec=pointCloudProject.GetGrayDepthVec();
    std::vector<cv::Vec3b> normal_rgb_vec=pointCloudProject.GetRGBNormalVec();
    cv::Mat img_depth_gray(rows,cols,CV_8UC1,depth_gray_vec.data());
    cv::Mat img_normal_rgb(rows,cols,CV_8UC3,normal_rgb_vec.data());
    cv::imshow("img_depth_gray",img_depth_gray);
    cv::imshow("img_normal_rgb",img_normal_rgb);

    std::vector<float> depth_vec=pointCloudProject.GetDepthVec();
    std::vector<float> depth_x_vec(rows*cols,0.0f);
    std::vector<float> depth_y_vec(rows*cols,0.0f);
    for(int i=1;i<rows-1;i++)
    {
        for(int j=1;j<cols-1;j++)
        {
            float left=depth_vec[i*cols+j-1];
            float right=depth_vec[i*cols+j+1];
            float top=depth_vec[(i-1)*cols+j];
            float bottom=depth_vec[(i+1)*cols+j];
            float curr=depth_vec[i*cols+j];

            float grad_x=right-left;
            float grad_y=bottom-top;
            float ave_diff_x=std::abs((right+curr+left)/3.0f-curr);
            float ave_diff_y=std::abs((top+curr+bottom)/3.0f-curr);

            if(ave_diff_x==0)
                depth_x_vec[i*cols+j]=0.0f;
            else
                depth_x_vec[i*cols+j]=grad_x/ave_diff_x;
            if(ave_diff_y==0)
                depth_y_vec[i*cols+j]=0.0f;

        }
    }


    cv::Mat depth_x(rows,cols,CV_32FC1);
    cv::Mat depth_y(rows,cols,CV_32FC1);




    cv::waitKey(0);
}