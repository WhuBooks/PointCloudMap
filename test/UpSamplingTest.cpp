//
// Created by whubooks on 18-3-20.
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <random>
#include <deque>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>

int main()
{
    std::string pcdfile="voxel.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile,*cloud);

    pcl::MovingLeastSquares<pcl::PointXYZI,pcl::PointXYZI>::Ptr mls(new pcl::MovingLeastSquares<pcl::PointXYZI,pcl::PointXYZI>);
    mls->setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree;
    mls->setSearchMethod(kdtree);
    mls->setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZI,pcl::PointXYZI>::SAMPLE_LOCAL_PLANE);
    mls->setSearchRadius(0.05);
    mls->setUpsamplingRadius(0.01);
    mls->setUpsamplingStepSize(0.01);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mls(new pcl::PointCloud<pcl::PointXYZI>);
    mls->process(*cloud_mls);

    pcl::io::savePCDFileASCII("voxel_mls_upsample.pcd",*cloud_mls);
}