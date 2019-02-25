//
// Created by whubooks on 18-3-9.
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

int main()
{
    std::string pcdfile="voxel_mls_smooth.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile,*cloud);

    /// estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI,pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.setRadiusSearch(0.1);
    normalEstimation.compute(*normals);

    /// planar segmentation
    pcl::SACSegmentationFromNormals<pcl::PointXYZI,pcl::Normal> segmentationFromNormals;
    segmentationFromNormals.setOptimizeCoefficients(true);
    segmentationFromNormals.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    segmentationFromNormals.setNormalDistanceWeight(0.1);
    segmentationFromNormals.setMethodType(pcl::SAC_RANSAC);
    segmentationFromNormals.setMaxIterations(1000);
    segmentationFromNormals.setDistanceThreshold(0.1);
    segmentationFromNormals.setInputCloud(cloud);
    segmentationFromNormals.setInputNormals(normals);

    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    segmentationFromNormals.segment(*inliers_plane,*coefficients_plane);
    std::cout<<"Plane coefficients ~ "<<*coefficients_plane<<std::endl;

//    /// use ransac segmentation
//    pcl::SACSegmentation<pcl::PointXYZI> segmentation;
//    segmentation.setInputCloud(cloud);
//    segmentation.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
//    segmentation.setMethodType(pcl::SAC_RANSAC);
//    segmentation.setMaxIterations(500);
//    segmentation.setOptimizeCoefficients(true);
//    segmentation.setDistanceThreshold(0.1);
//
//    pcl::ModelCoefficients::Ptr coefficients_plane2(new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers_plane2(new pcl::PointIndices);
//    segmentation.segment(*inliers_plane2,*coefficients_plane2);

    /// extract ground
    pcl::ExtractIndices<pcl::PointXYZI> extract_ground(true);
    extract_ground.setInputCloud(cloud);
    extract_ground.setIndices(inliers_plane);
    extract_ground.setNegative(false);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZI>);
    extract_ground.filter(*cloud_ground);

    /// extract ground normals
    pcl::ExtractIndices<pcl::Normal> extract_ground_normal(true);
    extract_ground_normal.setInputCloud(normals);
    extract_ground_normal.setIndices(inliers_plane);
    extract_ground_normal.setNegative(false);
    pcl::PointCloud<pcl::Normal>::Ptr normals_ground(new pcl::PointCloud<pcl::Normal>);
    extract_ground_normal.filter(*normals_ground);

    /// extract plane indices
    pcl::IndicesConstPtr outliers_plane;
    outliers_plane=extract_ground.getRemovedIndices();
    std::cout<<outliers_plane->size()<<std::endl;

    /// extract plane
    pcl::ExtractIndices<pcl::PointXYZI> extract_plane(true);
    extract_plane.setInputCloud(cloud);
    extract_plane.setIndices(outliers_plane);
    extract_plane.setNegative(false);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
    extract_plane.filter(*cloud_plane);
    std::cout<<"Ground Cloud Size ~ "<<cloud_ground->size()<<std::endl;
    std::cout<<"Plane Cloud Size ~ "<<cloud_plane->size()<<std::endl;

    /// extract plane normals
    pcl::ExtractIndices<pcl::Normal> extract_plane_normal(true);
    extract_plane_normal.setInputCloud(normals);
    extract_plane_normal.setIndices(outliers_plane);
    extract_plane_normal.setNegative(false);
    pcl::PointCloud<pcl::Normal>::Ptr normals_plane(new pcl::PointCloud<pcl::Normal>);
    extract_plane_normal.filter(*normals_plane);

    /// save ground and  plane
    std::string planefile="plane.pcd";
    std::string groundfile="ground.pcd";
    pcl::io::savePCDFileASCII(planefile,*cloud_plane);
    pcl::io::savePCDFileASCII(groundfile,*cloud_ground);

    /// estimate boundary from plane
    pcl::PointCloud<pcl::Boundary> boundary;
    pcl::BoundaryEstimation<pcl::PointXYZI,pcl::Normal,pcl::Boundary> boundaryEstimation;
    boundaryEstimation.setInputCloud(cloud_plane);
    boundaryEstimation.setInputNormals(normals_plane);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree_plane(new pcl::search::KdTree<pcl::PointXYZI>);
    boundaryEstimation.setSearchMethod(kdtree_plane);
    boundaryEstimation.setRadiusSearch(0.05);
    boundaryEstimation.setAngleThreshold(M_PI/4);
    boundaryEstimation.compute(boundary);

    /// extract boundary
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_no_boundary(new pcl::PointCloud<pcl::PointXYZI>);
    for(int i=0;i<cloud_plane->size();i++)
    {
        uint8_t isBoundary=boundary.points[i].boundary_point;
        int tmp= static_cast<int>(isBoundary);
        if(tmp==1)
            cloud_boundary->push_back(cloud_plane->points[i]);
        else
            cloud_no_boundary->push_back(cloud_plane->points[i]);
    }

    std::cout<<"Boundary Size ~ "<<cloud_boundary->points.size()<<std::endl;
    std::cout<<"Regular Size ~ "<<cloud_no_boundary->points.size()<<std::endl;

    std::string boundaryfile="boundary.pcd";
    std::string noboundaryfile="noboundary.pcd";
    pcl::io::savePCDFileASCII(boundaryfile,*cloud_boundary);
    pcl::io::savePCDFileASCII(noboundaryfile,*cloud_no_boundary);

}