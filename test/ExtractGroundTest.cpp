//
// Created by books on 2018/3/21.
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
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/range_image/impl/range_image.hpp>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

int main()
{
    std::string pcdfile = "voxel_mls_smooth.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile, *cloud);
    std::cout<<"Cloud Size ~ "<<cloud->points.size()<<std::endl;

    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZI> progressiveMorphologicalFilter;
    progressiveMorphologicalFilter.setInputCloud(cloud);
    progressiveMorphologicalFilter.setInitialDistance(0.2f);
    progressiveMorphologicalFilter.setMaxWindowSize(20);
    progressiveMorphologicalFilter.setSlope(2.0f);
    progressiveMorphologicalFilter.setMaxDistance(3.0f);

    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    progressiveMorphologicalFilter.extract(indices->indices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*cloud,*indices,*cloud_ground);
    
    std::cout<<"Cloud Ground Size ~ "<<cloud_ground->points.size()<<std::endl;

    std::string file="ground_extract.pcd";
    pcl::io::savePCDFileASCII(file,*cloud_ground);
    return 1;
}