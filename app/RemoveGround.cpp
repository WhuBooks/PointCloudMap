//
// Created by whubooks on 18-4-6.
//

#include <iostream>
#include <vector>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

#include <CloudUtil.h>
#include <Util.h>

//int main(int argc,char **argv)
//{
//    if (argc == 1)
//    {
//        std::cerr << "Need Input Pcd File!" << std::endl;
//        return -1;
//    }
//
//    std::string pcdfile = std::string(argv[1]);
//    std::string filename = Util::SplitNameWithoutExt(pcdfile);
//
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::io::loadPCDFile(pcdfile, *cloud);
//
//    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZI> progressiveMorphologicalFilter;
//    progressiveMorphologicalFilter.setInputCloud(cloud);
//    progressiveMorphologicalFilter.setInitialDistance(0.2f);
//    progressiveMorphologicalFilter.setMaxWindowSize(20);
//    progressiveMorphologicalFilter.setSlope(2.0f);
//    progressiveMorphologicalFilter.setMaxDistance(3.0f);
//
//    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
//    progressiveMorphologicalFilter.extract(indices->indices);
//
//    pcl::ExtractIndices<pcl::PointXYZI> extract_plane(true);
//    extract_plane.setInputCloud(cloud);
//    extract_plane.setNegative(false);
//    extract_plane.setIndices(indices);
//
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZI>);
//    extract_plane.filter(*cloud_ground);
//
//    std::string tmpfile=filename+"_mor.pcd";
//    pcl::io::savePCDFileASCII(tmpfile,*cloud_ground);
//
//}


int main(int argc,char **argv)
{
    if (argc == 1)
    {
        std::cerr << "Need Input Pcd File!" << std::endl;
        return -1;
    }

    std::string pcdfile = std::string(argv[1]);
    std::string filename = Util::SplitNameWithoutExt(pcdfile);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile, *cloud);

    std::string dir=Util::GetNameFromTime();
    Util::DirBuild(dir);

    std::string tmpfile=dir+"/"+filename+".pcd";
    pcl::io::savePCDFileASCII(tmpfile,*cloud);
    std::cout<<"Save Origin Cloud ~ "<<tmpfile<<std::endl;

    int max_num=5;
    for(int i=1;i<max_num;i++)
    {
        PCM::RansacRemoveGround(cloud);

        std::string planefile=dir+"/"+filename+"_plane"+std::to_string(i)+".pcd";
        pcl::io::savePCDFileASCII(planefile,*cloud);
        std::cout<<"Save Plane Cloud ~ "<<planefile<<std::endl;
    }

    return 1;
}









//int main(int argc,char **argv)
//{
//    if (argc == 1)
//    {
//        std::cerr << "Need Input Pcd File!" << std::endl;
//        return -1;
//    }
//
//    std::string pcdfile = std::string(argv[1]);
//    std::string filename = Util::SplitNameWithoutExt(pcdfile);
//
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::io::loadPCDFile(pcdfile,*cloud);
//
//    /// estimate normals
//    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//    pcl::NormalEstimation<pcl::PointXYZI,pcl::Normal> normalEstimation;
//    normalEstimation.setInputCloud(cloud);
//    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
//    normalEstimation.setSearchMethod(kdtree);
//    normalEstimation.setRadiusSearch(0.1);
//    normalEstimation.compute(*normals);
//
//    /// planar segmentation
//    pcl::SACSegmentationFromNormals<pcl::PointXYZI,pcl::Normal> segmentationFromNormals;
//    segmentationFromNormals.setOptimizeCoefficients(true);
//    segmentationFromNormals.setModelType(pcl::SACMODEL_NORMAL_PLANE);
//    segmentationFromNormals.setNormalDistanceWeight(0.1);
//    segmentationFromNormals.setMethodType(pcl::SAC_RANSAC);
//    segmentationFromNormals.setMaxIterations(1000);
//    segmentationFromNormals.setDistanceThreshold(0.1);
//    segmentationFromNormals.setInputCloud(cloud);
//    segmentationFromNormals.setInputNormals(normals);
//
//    /// segmentation
//    pcl::ModelCoefficients::Ptr coefficients_ground(new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers_ground(new pcl::PointIndices);
//    segmentationFromNormals.segment(*inliers_ground,*coefficients_ground);
//    std::cout<<"Ground coefficients ~ "<<*coefficients_ground<<std::endl;
//
//    /// extract ground
//    pcl::ExtractIndices<pcl::PointXYZI> extract_ground(true);
//    extract_ground.setInputCloud(cloud);
//    extract_ground.setIndices(inliers_ground);
//    extract_ground.setNegative(false);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZI>);
//    extract_ground.filter(*cloud_ground);
//
//    /// extract plane indices
//    pcl::IndicesConstPtr inliers_plane;
//    inliers_plane=extract_ground.getRemovedIndices();
//
//    /// extract plane
//    pcl::ExtractIndices<pcl::PointXYZI> extract_plane(true);
//    extract_plane.setInputCloud(cloud);
//    extract_plane.setIndices(inliers_plane);
//    extract_plane.setNegative(false);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
//    extract_plane.filter(*cloud_plane);
//    std::cout<<"Ground Cloud Size ~ "<<cloud_ground->size()<<std::endl;
//    std::cout<<"Plane Cloud Size ~ "<<cloud_plane->size()<<std::endl;
//
//    /// save ground and  plane
//    std::string planefile=filename+"_plane.pcd";
//    std::string groundfile=filename+"_ground.pcd";
//    pcl::io::savePCDFileASCII(planefile,*cloud_plane);
//    pcl::io::savePCDFileASCII(groundfile,*cloud_ground);
//
//    return 1;
//}