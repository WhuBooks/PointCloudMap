//
// Created by books on 2018/3/26.
//

#ifndef POINTCLOUDMAP_CLOUDUTIL_H
#define POINTCLOUDMAP_CLOUDUTIL_H

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

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

#include <sophus/se3.hpp>

#include <liblas/liblas.hpp>

#include <opencv2/opencv.hpp>

#include <Util.h>

namespace PCM
{
    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudXYZPtr;
    typedef pcl::PointCloud<pcl::PointXYZI>::Ptr CloudXYZIPtr;
    typedef pcl::PointCloud<pcl::PointXYZINormal>::Ptr CloudXYZINorPtr;
    typedef std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> PosVec;
//    typedef std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> Edge;
    typedef std::vector<cv::Point2i> Region2D;

    bool ConvertPcd2Las(const std::string &pcdfile,const std::string &lasfile);

    void WriteLas(CloudXYZPtr line_cloud,CloudXYZPtr left_cloud,CloudXYZPtr right_cloud ,const std::string &lasfile);
    void WriteLas(CloudXYZPtr line_cloud,const std::string &lasfile);

    void ConditionFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    void StatisticFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    void VoxelFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    
    void MlsFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    void RansacRemoveGround(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    std::vector<int> UniqueIndex(std::vector<int> vec);

    pcl::PointCloud<pcl::PointXYZI>::Ptr TransformCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,const Sophus::SE3f &se);

    std::vector<pcl::PointXYZI> TransformCloud(const std::vector<pcl::PointXYZI> &vec,const Sophus::SE3f &se);
    
    std::vector<int> SearchByPos(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,const PosVec &vec);

    /// calculate rotate matrix by rotate azix and rotate angle
    Eigen::Matrix3f CalRotation(const Eigen::Vector3f &azix,const float angle);

    /// calculate the rotate matrix from vec1 to vec2
    Eigen::Matrix3f CalRotation(const Eigen::Vector3f &vec1,const Eigen::Vector3f &vec2);

    PCM::CloudXYZPtr TransformCloud(PCM::CloudXYZPtr cloud,const Eigen::Matrix3f &rotation,const Eigen::Vector3f &translation);

    CloudXYZPtr SearchClosestWithinRadius(CloudXYZPtr cloud,CloudXYZPtr current);
    CloudXYZPtr SearchWithinRadius(CloudXYZPtr cloud,CloudXYZPtr current);
    CloudXYZPtr SearchKnnMeans(CloudXYZPtr cloud,CloudXYZPtr current);

    CloudXYZPtr SacLine(CloudXYZPtr cloud,std::vector<float> &coeff);
    CloudXYZPtr SacPlanar(CloudXYZPtr cloud,std::vector<float> &coeff);
    CloudXYZPtr InterpolateLine(CloudXYZPtr cloud,const std::vector<float> &coeff);
    CloudXYZPtr Project2Planar(CloudXYZPtr cloud,const std::vector<float> &coeff);

    float PlanarDensity(CloudXYZPtr cloud);
    
    
    
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ExtractEdge(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,const std::vector<std::vector<int>> &edge_vec);

}


#endif //POINTCLOUDMAP_CLOUDUTIL_H
