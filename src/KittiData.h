//
// Created by whubooks on 18-3-25.
//

#ifndef POINTCLOUDMAP_KITTIDATA_H
#define POINTCLOUDMAP_KITTIDATA_H

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <sophus/se3.h>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>

namespace PCM
{
    class KittiData
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        KittiData(const std::string &dir);
        ~KittiData();

        static Eigen::Matrix3f LoadKFromFile(const std::string filename);

        void SetSubNum(int K);

        int GetSize() const;

        cv::Mat GetImage(int num);

        pcl::PointCloud<pcl::PointXYZI>::Ptr GetCloud(int num);

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr GetCloudWithDis(int num);

        Sophus::SE3f GetCalib();

        Sophus::SE3f GetPose(int num);

        Eigen::Matrix3d GetK();

    private:
        std::string base_dir;
        std::string velo_dir;
        std::string calib_dir;
        std::string pose_dir;
        std::string img_dir;

        int m_size;

        Eigen::Matrix3d m_K;
        Sophus::SE3f m_calib;
        std::vector<Sophus::SE3f> m_pose_vec;
    };
}


#endif //POINTCLOUDMAP_KITTIDATA_H
