//
// Created by whubooks on 18-4-24.
//

#ifndef POINTCLOUDMAP_REGISTER_H
#define POINTCLOUDMAP_REGISTER_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/se3.hpp>

#define PCL_NO_PRECOMPILE
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace PCM {

//    struct MyPointType
//    {
//        PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
//        float test;
//        EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
//    } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment
//
//    // here we assume a XYZ + "test" (as fields)
//    POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointType,(float, x, x)(float, y, y)(float, z, z)(float, test, test))

    class Register
    {
    public:
        Register();

        ~Register();

        void AddCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,int index, const Sophus::SE3f &se);

        pcl::PointCloud<pcl::PointXYZI>::Ptr Process(const Sophus::SE3f &se);

    private:

        struct Base
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Eigen::Vector3f pos;
            int index;
            float dis;
        };

        std::vector<Base> m_data;

    };
}

#endif //POINTCLOUDMAP_REGISTER_H
