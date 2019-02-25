//
// Created by whubooks on 18-3-12.
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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/impl/bilateral.hpp>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/impl/fast_bilateral.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/flann_search.h>
#include <sophus/se3.h>
#include <opencv2/opencv.hpp>

#include <Util.h>

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

float
G (float x, float sigma)
{
    return exp (- (x*x)/(2*sigma*sigma));
}
pcl::PointCloud<pcl::PointXYZI> BilateralFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,double sigma_s,double sigma_r)
{

    int pnumber = (int)cloud->size ();

    // Output Cloud = Input Cloud
    pcl::PointCloud<pcl::PointXYZI> outcloud = *cloud;

    // Set up KDTree
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZI>);
    tree->setInputCloud (cloud);

    // Neighbors containers
    std::vector<int> k_indices;
    std::vector<float> k_distances;

    // Main Loop
    for (int point_id = 0; point_id < pnumber; ++point_id)
    {
        float BF = 0;
        float W = 0;

        tree->radiusSearch(cloud->points[point_id], 2 * sigma_s, k_indices, k_distances);

        // For each neighbor
        for (size_t n_id = 0; n_id < k_indices.size (); ++n_id)
        {
            float id = k_indices.at (n_id);
            float dist = sqrt (k_distances.at (n_id));
            float intensity_dist = abs (cloud->points[point_id].intensity - cloud->points[id].intensity);

            float w_a = G (dist, sigma_s);
            float w_b = G (intensity_dist, sigma_r);
            float weight = w_a * w_b;

            BF += weight * cloud->points[id].intensity;
            W += weight;
        }

        outcloud.points[point_id].intensity = BF / W;
    }
    return outcloud;
}

int main()
{
    std::string pcdfile="20_filter.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile,*cloud);

    /// condition removal filter
    pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond_and(new pcl::ConditionAnd<pcl::PointXYZI>);
    pcl::FieldComparison<pcl::PointXYZI>::ConstPtr cond_1(new pcl::FieldComparison<pcl::PointXYZI>("z",pcl::ComparisonOps::GT,0.0));
    range_cond_and->addComparison(cond_1);
    pcl::FieldComparison<pcl::PointXYZI>::ConstPtr cond_2(new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::LT,1.8));
    range_cond_and->addComparison(cond_2);

    pcl::ConditionalRemoval<pcl::PointXYZI> conditionalRemoval;
    conditionalRemoval.setInputCloud(cloud);
    conditionalRemoval.setCondition(range_cond_and);
    conditionalRemoval.setKeepOrganized(true);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cond(new pcl::PointCloud<pcl::PointXYZI>);
    conditionalRemoval.filter(*cloud_cond);

//    /// staticstic filter
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> stat_removal;
//    stat_removal.setMeanK(100);
//    stat_removal.setStddevMulThresh(1);
//    stat_removal.setInputCloud(cloud_cond);
//    stat_removal.setKeepOrganized(true);
//
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_stat(new pcl::PointCloud<pcl::PointXYZI>);
//    stat_removal.filter(*cloud_stat);

//    
//    /// bilateral filter
//    pcl::BilateralFilter<pcl::PointXYZI> bilateral_filter;
//    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
//    bilateral_filter.setSearchMethod(kdtree);
//
//    bilateral_filter.setInputCloud(cloud_cond);
//    bilateral_filter.setHalfSize(5.0);
//    bilateral_filter.setStdDev(0.3);
//
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bilateral(new pcl::PointCloud<pcl::PointXYZI>);
//    bilateral_filter.applyFilter(*cloud_bilateral);
//    std::cout<<"cloud_bilateral ~ "<<cloud_bilateral->points.size()<<std::endl;

//    /// fast bilateral filter
//    pcl::FastBilateralFilter<pcl::PointXYZI>::Ptr fast_bilateral_filter(new pcl::FastBilateralFilter<pcl::PointXYZI>);
//    fast_bilateral_filter->setInputCloud(cloud_cond);
//    fast_bilateral_filter->setSigmaS(5);
//    fast_bilateral_filter->setSigmaR(0.03);
//
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bilateral(new pcl::PointCloud<pcl::PointXYZI>);
//    fast_bilateral_filter->filter(*cloud_bilateral);
//    std::cout<<"cloud_bilateral ~ "<<cloud_bilateral->points.size()<<std::endl;


    /// voxel filter
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(cloud_cond);
    voxel_filter.setLeafSize(0.05f,0.05f,0.05f);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZI>);
    voxel_filter.filter(*cloud_voxel);
//
//    /// camera model filter
//    std::string imgfile="/home/whubooks/kitti_odometry/data_odometry_gray/sequences/00/image_0/000020.png";
//    std::string calibfile="/home/whubooks/kitti_odometry/data_odometry_calib/sequences/00/calib.txt";
//
//    cv::Mat img_gray_origin=cv::imread(imgfile,CV_LOAD_IMAGE_GRAYSCALE);
////    cv::imshow("img_gray_origin",img_gray_origin);
//    Matrix34d calib=LoadCalibFile(calibfile);
//    Eigen::Matrix3d K=calib.block(0, 0, 3, 3);
//
//    double K_fx=K(0,0),K_dx=K(0,2),K_fy=K(1,1),K_dy=K(1,2);
//    double u_limit_min=-K_dx/K_fx;
//    double u_limit_max=(img_gray_origin.cols-K_dx)/K_fx;
//    double v_limit_min=-K_dy/K_fy;
//    double v_limit_max=(img_gray_origin.rows-K_dy)/K_fy;
//
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_camera(new pcl::PointCloud<pcl::PointXYZI>);
//    for(const pcl::PointXYZI & pt : cloud_voxel->points)
//    {
//        if(pt.z==0)
//            continue;
//
//        double xz_ratio=pt.x/pt.z;
//        if(xz_ratio<=u_limit_min || xz_ratio>=u_limit_max)
//            continue;
//
//        double yz_ratio=pt.y/pt.z;
//        if(yz_ratio<=v_limit_min || yz_ratio>=v_limit_max)
//            continue;
//
//        cloud_camera->push_back(pt);
//        //cloud_camera->points.push_back(pt);
//    }
//    std::cout<<cloud_camera->points.size()<<std::endl;

    /// save point cloud from each step
    pcl::io::savePCDFile("cond.pcd",*cloud_cond);
   // pcl::io::savePCDFile("stat.pcd",*cloud_stat);
   // pcl::io::savePCDFile("bilateral.pcd",*cloud_bilateral);
    pcl::io::savePCDFile("voxel.pcd",*cloud_voxel);
//    pcl::io::savePCDFile("camera.pcd",*cloud_camera);
}