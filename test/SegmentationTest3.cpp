//
// Created by whubooks on 18-3-20.
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
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Util.h>

int main()
{
    std::string pcdfile = "plane.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile, *cloud);

    std::vector<pcl::PointIndices> indices_vector;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> euclideanClusterExtraction;
    euclideanClusterExtraction.setInputCloud(cloud);
    euclideanClusterExtraction.setMinClusterSize(50);
    euclideanClusterExtraction.setMaxClusterSize(10000000);
    euclideanClusterExtraction.setClusterTolerance(0.05);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree;
    euclideanClusterExtraction.setSearchMethod(kdtree);
    euclideanClusterExtraction.extract(indices_vector);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    
    std::string dir=Util::GetNameFromTime();
    Util::DirBuild(dir);

    for(int i=0;i<indices_vector.size();i++)
    {
        pcl::PointIndices indices=indices_vector[i];

        pcl::ExtractIndices<pcl::PointXYZI> extractIndices;
        extractIndices.setInputCloud(cloud);
        extractIndices.setIndices(boost::make_shared<pcl::PointIndices>(indices));
        extractIndices.setNegative(false);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_indices(new pcl::PointCloud<pcl::PointXYZI>);
        extractIndices.filter(*cloud_indices);

        std::string filename=dir+"/"+std::to_string(i)+".pcd";
        pcl::io::savePCDFileASCII(filename,*cloud_indices);
        
        for(pcl::PointXYZI &pt : cloud_indices->points)
        {
            pt.intensity=255.0f*i/indices_vector.size();
            cloud_cluster->push_back(pt);
        }
        
    }

    pcl::io::savePCDFileASCII("cluster.pcd",*cloud_cluster);
    
    return 1;
}