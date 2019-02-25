//
// Created by whubooks on 18-4-24.
//

#include "Register.h"

namespace PCM {

    Register::Register()
    {

    }

    Register::~Register()
    {

    }

    void Register::AddCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,int index, const Sophus::SE3f &se)
    {
        for(const pcl::PointXYZI &pt : cloud->points)
        {
            Eigen::Vector3f pos(pt.x,pt.y,pt.z);
            float dis=pos.norm();
            Eigen::Vector3f pos_trans=se*pos;

            Base base;
            base.index=index;
            base.dis=dis;
            base.pos=pos_trans;
            m_data.push_back(base);
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr Register::Process(const Sophus::SE3f &se)
    {
        std::vector<Base> data;
        data.assign(m_data.begin(),m_data.end());





        return pcl::PointCloud<pcl::PointXYZI>::Ptr();
    }


}


