//
// Created by whubooks on 18-3-21.
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>
#include <list>
#include <algorithm>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/boundary.h>

#include <Util.h>

int main()
{
    std::string pcdfile="voxel_mls_smooth.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile,*cloud);

    std::string dir="Segmentation_"+Util::GetNameFromTime();
    Util::DirBuild(dir);

    const int num=cloud->points.size();
    double ratio=0.2;
    int index=0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_segmentation(new pcl::PointCloud<pcl::PointXYZI>);
    while(cloud->points.size()>ratio*num)
    {
        pcl::SACSegmentation<pcl::PointXYZI> sacSegmentation;
        sacSegmentation.setInputCloud(cloud);
        sacSegmentation.setOptimizeCoefficients(true);
        sacSegmentation.setMaxIterations(1000);
        sacSegmentation.setMethodType(pcl::SAC_RANSAC);
        sacSegmentation.setModelType(pcl::SACMODEL_PLANE);
        sacSegmentation.setDistanceThreshold(0.03);

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        sacSegmentation.segment(*inliers,*coefficients);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_planar(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_regular(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::ExtractIndices<pcl::PointXYZI>::Ptr extract_indices(new pcl::ExtractIndices<pcl::PointXYZI>);
        extract_indices->setInputCloud(cloud);
        extract_indices->setNegative(false);
        extract_indices->setIndices(inliers);
        extract_indices->filter(*cloud_planar);
        extract_indices->setNegative(true);
        extract_indices->filter(*cloud_regular);
        std::cout<<"Iteration ~ "<<index<<"\t extract points ~ "<<cloud_planar->points.size()<<std::endl;
        
        std::string tmpfile=dir+"/"+std::to_string(index)+".pcd";
        pcl::io::savePCDFileASCII(tmpfile,*cloud_planar);
        index++;

        for(const pcl::PointXYZI &pt : cloud_planar->points)
        {
            pcl::PointXYZI tmp;
            tmp.x=pt.x;
            tmp.y=pt.y;
            tmp.z=pt.z;
            tmp.intensity=index*5;
            cloud_segmentation->push_back(tmp);
        }

        cloud.swap(cloud_regular);
    }

    pcl::io::savePCDFileASCII("segmentation.pcd",*cloud_segmentation);
    return 1;
}