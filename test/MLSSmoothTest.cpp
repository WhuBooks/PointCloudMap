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

#include <Util.h>

int main(int argc,char **argv)
{
    std::string pcdfile=(argc>1)?std::string(argv[1]):"voxel.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile,*cloud);
    std::cout<<"Origin Size ~ "<<cloud->points.size()<<std::endl;

    pcl::MovingLeastSquares<pcl::PointXYZI,pcl::PointXYZI>::Ptr mls(new pcl::MovingLeastSquares<pcl::PointXYZI,pcl::PointXYZI>);
    mls->setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree;
    mls->setSearchMethod(kdtree);
    mls->setSearchRadius(0.05);
    mls->setComputeNormals(true);
    mls->setPolynomialFit(true);
    mls->setPolynomialOrder(4);


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mls(new pcl::PointCloud<pcl::PointXYZI>);
    mls->process(*cloud_mls);
    std::cout<<"MlS Smooth Result Size ~ "<<cloud_mls->points.size()<<std::endl;

    std::string resultfile=Util::SplitNameWithoutExt(pcdfile)+"_mls.pcd";
    pcl::io::savePCDFileASCII(resultfile,*cloud_mls);
}