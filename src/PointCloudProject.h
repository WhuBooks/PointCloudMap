//
// Created by books on 2018/3/25.
//

#ifndef POINTCLOUDMAP_POINTCLOUDPROJECT_H
#define POINTCLOUDMAP_POINTCLOUDPROJECT_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <numeric>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

//#include <sophus/se3.h>
#include <sophus/se3.hpp>


namespace PCM
{
    struct Feature
    {
        int u;
        int v;
        float u_d;
        float v_d;
        float normal_x;
        float normal_y;
        float normal_z;
    };

    class PointCloudProject
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        struct ProjectBase
        {
            float u_p;
            float v_p;
            float depth;
            float normal_x;
            float normal_y;
            float normal_z;

            bool operator<(const ProjectBase &other) const
            {
                return u_p<other.u_p ||
                        (u_p==other.u_p&&v_p<other.v_p)||
                        (u_p==other.u_p&&v_p==other.v_p&&depth<other.depth);
            }

            bool operator>(const ProjectBase &other) const
            {
                return u_p>other.u_p ||
                       (u_p==other.u_p&&v_p>other.v_p)||
                       (u_p==other.u_p&&v_p==other.v_p&&depth>other.depth);
            }

            bool operator==(const ProjectBase &other) const
            {
                return u_p==other.u_p&&v_p==other.v_p&&depth==other.depth;
            }
        };

    public:
        PointCloudProject();
        ~PointCloudProject();

        void SetInput(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
        void SetInput(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud);

        void SetK(const Eigen::Matrix3f &K);
        void SetImageSize(const int &rows,const int &cols);

        static std::vector<float> Normalize(const std::vector<float> &vec);
        static std::vector<uchar> Float2Gray(const std::vector<float> &vec);

        void Project();
        void Project2();

        void EDL();
        void EDLScale();

        std::vector<float> GetOriginDeph() const;
        std::vector<float> GetDepthVec() const;
        std::vector<uchar> GetGrayDepthVec() const ;
        std::vector<cv::Vec3f> GetNormalVec() const;
        std::vector<cv::Vec3b> GetRGBNormalVec() const;
        std::vector<Feature> GetFeature() const;
        std::vector<float> GetShadowVec() const;
        std::vector<uchar> GetGrayShadowVec(bool median_filter=false) const;
        std::vector<cv::Vec3b> GetRGBShadowVec() const;

        Eigen::Vector3f ReProject(int row,int col) const;
        

    private:
        void CameraModelFilter();
        void FillHole();

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr m_cloud;
        Eigen::Matrix3f m_K;
        int m_rows;
        int m_cols;
        std::vector<ProjectBase> m_2d_points;
        std::vector<float> m_origin_depth_vec;
        std::vector<float> m_depth_vec;
        std::vector<cv::Vec3f> m_normal_vec;
        std::vector<float> m_shadows_vec;
    };
}


#endif //POINTCLOUDMAP_POINTCLOUDPROJECT_H
