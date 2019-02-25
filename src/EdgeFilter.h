//
// Created by whubooks on 18-5-2.
//

#ifndef POINTCLOUDMAP_EDGEFILTER_H
#define POINTCLOUDMAP_EDGEFILTER_H

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>

#include <CloudUtil.h>

namespace PCM {

    static int edge_pt_id=0;
    struct EdgePt
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        int id;
        Eigen::Vector3f pos;
        Eigen::Vector3f dir;
    };

    struct LinePlanar
    {
        Region2D line2d;
        Region2D left2d;
        Region2D right2D;

        CloudXYZPtr line;
        CloudXYZPtr left;
        CloudXYZPtr right;

        std::vector<EdgePt> edge_pts;
    };

    class EdgeFilter
    {
    public:

        EdgeFilter(int _rows, int _cols, const Eigen::Matrix3f &_K, const std::vector<float> &_origin_depth);

        ~EdgeFilter();

        int Filter();
        int Filter2();

        void SetCloud(CloudXYZIPtr cloud);
        void SetRegion2D(const std::vector<Region2D> &line,const std::vector<Region2D> &left,const std::vector<Region2D> &right);

        void Write(std::string dir,std::vector<uchar> img) const;
        void Write(std::string dir,std::vector<uchar> img,const Eigen::Affine3f &affine) const ;

        std::vector<CloudXYZPtr> GetLineBack() const { return m_line_back_vec; };

        std::vector<CloudXYZPtr> GetLeftEnvBack() const { return m_left_env_back_vec; };

        std::vector<CloudXYZPtr> GetRightEnvBack() const { return m_right_env_back_vec; };

        std::vector<CloudXYZPtr> GetLineSearch() const { return m_line_search_vec; };

        std::vector<CloudXYZPtr> GetLeftEnvSearch() const { return m_left_env_search_vec; };

        std::vector<CloudXYZPtr> GetRightEnvSearch() const { return m_right_env_search_vec; };

        std::vector<CloudXYZPtr> GetLineSac() const { return m_line_sac_vec; };

        std::vector<CloudXYZPtr> GetLeftEnvSac() const { return m_left_env_sac_vec; };

        std::vector<CloudXYZPtr> GetRightEnvSac() const { return m_right_env_sac_vec; };

        std::vector<CloudXYZPtr> GetLineEnvProject() const { return m_line_env_project_vec; };

        std::vector<CloudXYZPtr> GetLineEnv() const { return m_line_env_vec; };

        int GetSingleEdgeNum() const
        {
            int num=0;
            for(const int &val : m_edge_pts_type)
            {
                if(val==0)
                    num++;
            }
            return num;
        }

        int GetDoubleEdgeNum() const
        {
            int num=0;
            for(const int &val : m_edge_pts_type)
            {
                if(val==1)
                    num++;
            }
            return num;
        }

        CloudXYZPtr GetSingleXYZResult(const Eigen::Affine3f &affine=Eigen::Affine3f::Identity()) const
        {
            CloudXYZPtr result(new pcl::PointCloud<pcl::PointXYZ>);
            for(int i=0;i<m_line_env_vec.size();i++)
            {
                CloudXYZPtr line_env=m_line_env_vec[i];
                if(m_edge_pts_type[i]!=0)
                    continue;
                for(const pcl::PointXYZ &pt : line_env->points)
                {
                    result->push_back(pt);
                }
            }
            CloudXYZPtr result_trans(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*result,*result_trans,affine);

            return result_trans;
        }

        CloudXYZPtr GetDoubleXYZResult(const Eigen::Affine3f &affine=Eigen::Affine3f::Identity()) const
        {
            CloudXYZPtr result(new pcl::PointCloud<pcl::PointXYZ>);
            for(int i=0;i<m_line_env_vec.size();i++)
            {
                CloudXYZPtr line_env=m_line_env_vec[i];
                if(m_edge_pts_type[i]!=1)
                    continue;
                for(const pcl::PointXYZ &pt : line_env->points)
                {
                    result->push_back(pt);
                }
            }
            CloudXYZPtr result_trans(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*result,*result_trans,affine);

            return result_trans;
        }

        CloudXYZIPtr GetXYZIResult() const
        {
            CloudXYZIPtr result(new pcl::PointCloud<pcl::PointXYZI>);
            for(int i=0;i<m_line_env_vec.size();i++)
            {
                CloudXYZPtr line_env=m_line_env_vec[i];
                for(const pcl::PointXYZ &pt : line_env->points)
                {
                    pcl::PointXYZI tmp;
                    pcl::copyPoint(pt,tmp);
                    tmp.intensity=i;
                    result->push_back(tmp);
                }
            }
            return result;
        };

        std::vector<PCM::EdgePt> GetSingleEdgePts(const Eigen::Affine3f &affine=Eigen::Affine3f::Identity()) const {
            std::vector<PCM::EdgePt> edge_pts;
            for(int i=0;i<m_edge_pts.size();i++)
            {
                if(m_edge_pts_type[i]!=0)
                    continue;
                PCM::EdgePt edge_pt=m_edge_pts[i];
                PCM::EdgePt pt_trans;
                pt_trans.pos=affine*edge_pt.pos;
                pt_trans.pos=affine*edge_pt.dir;
                edge_pts.push_back(pt_trans);
            }
            return edge_pts;
        }

        std::vector<PCM::EdgePt> GetDoubleEdgePts(const Eigen::Affine3f &affine=Eigen::Affine3f::Identity()) const {
            std::vector<PCM::EdgePt> edge_pts;
            for(int i=0;i<m_edge_pts.size();i++)
            {
                if(m_edge_pts_type[i]!=1)
                    continue;
                PCM::EdgePt edge_pt=m_edge_pts[i];
                PCM::EdgePt pt_trans;
                pt_trans.pos=affine*edge_pt.pos;
                pt_trans.pos=affine*edge_pt.dir;
                edge_pts.push_back(pt_trans);
            }
            return edge_pts;
        }

    private:

        CloudXYZPtr BackProjectEnv(const Region2D &region_2d);
        CloudXYZPtr BackProjectLine(const Region2D &region_2d);

        CloudXYZPtr LineEnv(CloudXYZPtr line,CloudXYZPtr env,const std::vector<float> &env_coeff);

        Eigen::Matrix3f m_K;
        int m_rows;
        int m_cols;

        std::vector<float> m_origin_depth_vec;
        CloudXYZPtr m_cloud;

        std::vector<Region2D> m_line_region_vec;
        std::vector<Region2D> m_left_env_region_vec;
        std::vector<Region2D> m_right_env_region_vec;

        std::vector<int> m_index_vec;

        std::vector<Region2D> m_filter_line_region_vec;
        std::vector<Region2D> m_filter_left_env_region_vec;
        std::vector<Region2D> m_filter_right_env_region_vec;

        std::vector<CloudXYZPtr> m_line_back_vec;
        std::vector<CloudXYZPtr> m_left_env_back_vec;
        std::vector<CloudXYZPtr> m_right_env_back_vec;

        std::vector<CloudXYZPtr> m_line_search_vec;
        std::vector<CloudXYZPtr> m_left_env_search_vec;
        std::vector<CloudXYZPtr> m_right_env_search_vec;

        std::vector<CloudXYZPtr> m_line_sac_vec;
        std::vector<CloudXYZPtr> m_left_env_sac_vec;
        std::vector<CloudXYZPtr> m_right_env_sac_vec;

        std::vector<CloudXYZPtr> m_line_env_project_vec;
        std::vector<CloudXYZPtr> m_line_env_vec;

        std::vector<int> m_edge_pts_type;
        std::vector<PCM::EdgePt> m_edge_pts;

        std::vector<PCM::LinePlanar> m_line_planar_vec;

    };

}


#endif //POINTCLOUDMAP_EDGEFILTER_H
