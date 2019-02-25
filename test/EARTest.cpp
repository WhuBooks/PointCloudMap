//
// Created by whubooks on 18-3-16.
//

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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/flann_search.h>
#include <sophus/se3.h>
#include <opencv2/opencv.hpp>

#include <Util.h>
#include <chrono>

struct EarPt
{
    float x;
    float y;
    float z;

    float x_normal;
    float y_normal;
    float z_normal;

    float intensity;

    EarPt()
    {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        x_normal = 0.0f;
        y_normal = 0.0f;
        z_normal = 0.0f;
        intensity = 0.0f;
    }
    EarPt(float _x,float _y,float _z,float _x_normal,float _y_normal,float _z_normal,float _intensity)
    {
        x = _x;
        y = _y;
        z = _z;
        x_normal = _x_normal;
        y_normal = _y_normal;
        z_normal = _z_normal;
        intensity = _intensity;
    }
    EarPt(const pcl::PointXYZINormal &pt)
    {
        x = pt.x;
        y = pt.y;
        z = pt.z;
        x_normal = pt.normal_x;
        y_normal = pt.normal_y;
        z_normal = pt.normal_z;
        intensity = pt.intensity;
    }

    pcl::PointXYZINormal ToPclPt() const
    {
        pcl::PointXYZINormal pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        pt.intensity = intensity;
        pt.normal_x = x_normal;
        pt.normal_y = y_normal;
        pt.normal_z = z_normal;
        return pt;
    }

    Eigen::Vector3f Pos() const
    {
        return Eigen::Vector3f(x,y,z);
    }
    Eigen::Vector3f Normal() const
    {
        return Eigen::Vector3f(x_normal,y_normal,z_normal);
    }

};
typedef std::vector<EarPt> EarPtVec;

EarPtVec InitializeRandom(EarPtVec vec,double ratio)
{
    EarPtVec result;

    static std::default_random_engine e;
    e.seed(std::time(nullptr));
    std::uniform_int_distribution<> uniform(0,1000000);

    int n=std::ceil(vec.size()*ratio);
    for(int i=0;i<n||vec.empty();i++)
    {
        int index=uniform(e)%vec.size();
        EarPt pt=vec[index];
        result.push_back(pt);

        EarPtVec::iterator iter=vec.begin()+index;
        vec.erase(iter);
    }
    return result;
}


EarPtVec KnnSearch(const EarPtVec &vec,const EarPt &pt,int k)
{
    typedef std::pair<EarPt, double> pair_dis;
    std::vector<std::pair<EarPt, double>> map;
    for (const EarPt &tmp : vec)
    {
        if (tmp.x == pt.x && tmp.y == pt.y && tmp.z == pt.z && tmp.intensity == pt.intensity &&
            tmp.x_normal == pt.x_normal && tmp.y_normal == pt.y_normal && tmp.z_normal == pt.z_normal)
            continue;

        double dis =
                (pt.x - tmp.x) * (pt.x - tmp.x) + (pt.y - tmp.y) * (pt.y - tmp.y) + (pt.z - tmp.z) * (pt.z - tmp.z);
        map.push_back(std::make_pair(tmp, dis));
    }

    std::sort(map.begin(), map.end(), [](const pair_dis &x1, const pair_dis &x2) {
        return x1.second < x2.second;
    });

    EarPtVec result;
    for (int i = 0; i < k; i++)
        result.push_back(map[i].first);
    return result;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Bilateral(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud,
                                                     const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud_filter)
{
    const int k = 16;
    float theta = 0.5;
    theta=std::sqrt(cloud->width*cloud->width+cloud->height*cloud->height)/std::sqrt(cloud->size());
    float thetaN = 15.0f;
    float cos_thetaN = std::cos((float) M_PI * thetaN / 180.0f);

    pcl::KdTreeFLANN<pcl::PointXYZINormal> kdTreeFLANN;
    kdTreeFLANN.setInputCloud(cloud_filter);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr result(new pcl::PointCloud<pcl::PointXYZINormal>);
    for (const pcl::PointXYZINormal &pt : cloud_filter->points)
    {
        Eigen::Vector3f pos(pt.x, pt.y, pt.z);
        float normal_x = pt.normal_x;
        float normal_y = pt.normal_y;
        float normal_z = pt.normal_z;
        float curvature = pt.curvature;

        std::vector<int> vindices;
        std::vector<float> vdistance;
        kdTreeFLANN.nearestKSearch(pt, k, vindices, vdistance);

        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vpos_knn, vnormal_knn;
        for (const int &index : vindices)
        {
            pcl::PointXYZINormal pt_knn = cloud_filter->points[index];
            Eigen::Vector3f pos_knn(pt_knn.x, pt_knn.y, pt_knn.z);
            Eigen::Vector3f normal_knn(pt_knn.normal_x, pt_knn.normal_y, pt_knn.normal_z);

            vpos_knn.push_back(pos_knn);
            vnormal_knn.push_back(normal_knn);
        }

        //normal smooth
        Eigen::Vector3f normal(normal_x, normal_y, normal_z);
        Eigen::Vector3f normal_update(0.0f, 0.0f, 0.0f);
        float normal_sum = 0.0f;
        for (int j = 0; j < vpos_knn.size(); j++)
        {
            Eigen::Vector3f pos_knn = vpos_knn[j];
            Eigen::Vector3f normal_knn = vnormal_knn[j];

            float pos_product=(pos-pos_knn).transpose()*(pos-pos_knn);
            float exp_pos = std::exp(-pos_product / (theta * theta));
            float exp_normal = std::exp(
                    -std::pow((1 - normal.transpose() * normal_knn) / (1 - cos_thetaN), 2));

            normal_update += exp_pos * exp_normal * normal;
            normal_sum += exp_pos * exp_normal;
        }
        normal_update /= normal_sum;

        //position smooth
        //global smooth
        Eigen::Vector3f sum_pos_alpha_vj(0.0f,0.0f,0.0f);
        float sum_alpha_vj=0.0f;
        for(int j=0;j<cloud->points.size();j++)
        {
            pcl::PointXYZINormal pt_origin=cloud->points[j];
            Eigen::Vector3f pos_origin(pt_origin.x,pt_origin.y,pt_origin.z);

            //calculate vj and alpha
            float vj=1.0f;
            for(int jj=0;jj<cloud->points.size();jj++)
            {
                if(j==jj)
                    continue;

                pcl::PointXYZINormal pt_tmp=cloud->points[jj];
                Eigen::Vector3f pos_tmp(pt_tmp.x,pt_tmp.y,pt_tmp.z);
                vj+=std::exp(std::pow(-(pos_origin-pos_tmp).norm(),2)/(theta*theta));
            }

            //calculate alpha
            Eigen::Vector3f ketha=pos-pos_origin;
            float alpha=std::exp(std::pow(-normal_update.transpose()*ketha,2)/(theta*theta))/ketha.norm();

            sum_pos_alpha_vj+=(alpha/vj)*pos_origin;
            sum_alpha_vj+=(alpha/vj);
        }

        //local smooth
        Eigen::Vector3f sum_pos_beta_wi(0.0f,0.0f,0.0f);
        float sum_beta_wi=0.0f;
        for (int j = 0; j < vpos_knn.size(); j++)
        {
            Eigen::Vector3f pos_knn = vpos_knn[j];
            Eigen::Vector3f delta=pos-pos_knn;

            float wi=1.0f;
            //calculate wi
            for(int jj=0;jj<vpos_knn.size();jj++)
            {
                if(j==jj)
                    continue;
                Eigen::Vector3f pos_tmp=vpos_knn[jj];
                wi+=std::exp(std::pow(-(pos-pos_tmp).norm(),2)/(theta*theta));
            }

            //calculate beta
            float beta=std::exp(std::pow(delta.norm(),2));

            sum_pos_beta_wi+=beta*wi*delta;
            sum_beta_wi+=beta*wi;
        }

        Eigen::Vector3f pos_update=sum_pos_alpha_vj/sum_alpha_vj+0.45*sum_pos_beta_wi/sum_beta_wi;

        pcl::PointXYZINormal pt_bilateral;
        pt_bilateral.x=pos_update(0);
        pt_bilateral.y=pos_update(1);
        pt_bilateral.z=pos_update(2);
        pt_bilateral.normal_x=normal_update(0);
        pt_bilateral.normal_y=normal_update(1);
        pt_bilateral.normal_z=normal_update(2);
        result->push_back(pt_bilateral);
    }
    return result;
}

int main()
{
    std::string pcdfile="conv.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile,*cloud);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::NormalEstimation<pcl::PointXYZI,pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setSearchMethod(kdTree);
    normalEstimation.setKSearch(16);
    normalEstimation.compute(*normals);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::concatenateFields(*cloud,*normals,*cloud_normal);
    std::cout<<"Calculate Normal by PCA Done."<<std::endl;

    const int k = 16;
    const float theta = (float)(std::sqrt(cloud->width*cloud->width+cloud->height*cloud->height)/std::sqrt(cloud->size()));
    const float thetaN = 15.0f;
    const float cos_thetaN = std::cos((float) M_PI * thetaN / 180.0f);
    const float u=0.45;

    EarPtVec ear_pt_vec_j;
    for(const pcl::PointXYZINormal &pt : cloud_normal->points)
        ear_pt_vec_j.push_back(EarPt(pt));
    EarPtVec ear_pt_vec_i=InitializeRandom(ear_pt_vec_j,0.6);
    std::cout<<"Initialize Done."<<std::endl;

    std::vector<float> vj_vec;
    for(int i=0;i<ear_pt_vec_j.size();i++)
    {
        float vj=1.0f;
        EarPt vj_i=ear_pt_vec_j[i];
        for(int j=0;j<ear_pt_vec_j.size();j++)
        {
            if(i==j)
                continue;

            EarPt vj_j=ear_pt_vec_j[j];
            float pos_diff_norm=(vj_i.Pos()-vj_j.Pos()).norm();
            vj+=std::exp(-(pos_diff_norm*pos_diff_norm)/(theta*theta));
        }
        vj_vec.push_back(vj);
    }
    std::cout<<"Calculate Vj Done."<<std::endl;

    for(int iter=0;iter<3;iter++)
    {
        std::cout<<"Iteration ~ "<<iter<<std::endl;
        EarPtVec ear_pt_vec_i_tmp;

        std::chrono::steady_clock::time_point tp1=std::chrono::steady_clock::now();
        //smooth normal
        for(const EarPt &pt_i : ear_pt_vec_i)
        {
            EarPtVec vec_knn=KnnSearch(ear_pt_vec_i,pt_i,k);
            Eigen::Vector3f normal_update(0.0f,0.0f,0.0f);
            float sum=0.0f;
            for(const EarPt &pt_knn: vec_knn)
            {
                float pos_diff_norm=(pt_i.Pos()-pt_knn.Pos()).norm();
                float theta_pos_diff_norm=std::exp(-(pos_diff_norm*pos_diff_norm)/(theta*theta));

                float normal_product=pt_i.Normal().transpose()*pt_knn.Normal();
                float fry_normal_product=(float)std::exp(-std::pow((1-normal_product)/(1-cos_thetaN),2));

                normal_update+=theta_pos_diff_norm*fry_normal_product*pt_knn.Normal();
                sum+=theta_pos_diff_norm*fry_normal_product;
            }
            normal_update/=sum;

            EarPt pttmp=pt_i;
            pttmp.x_normal=normal_update(0);
            pttmp.y_normal=normal_update(1);
            pttmp.z_normal=normal_update(2);
            ear_pt_vec_i_tmp.push_back(pttmp);
        }
        ear_pt_vec_i.swap(ear_pt_vec_i_tmp);
        ear_pt_vec_i_tmp.clear();

        std::chrono::steady_clock::time_point tp2=std::chrono::steady_clock::now();
        std::chrono::steady_clock::duration span1=std::chrono::duration_cast<std::chrono::seconds>(tp2-tp1);
        std::cout<<"Normal Smooth Done."<<span1.count()<<std::endl;

        //smooth position
        for(int i=0;i<ear_pt_vec_i.size();i++)
        {
            EarPt pt_i=ear_pt_vec_i[i];

            Eigen::Vector3f pos_update_j(0.0f,0.0f,0.0f);
            float pos_update_j_sum=0.0f;
            for(int j=0;j<ear_pt_vec_j.size();j++)
            {
                EarPt pt_j=ear_pt_vec_j[j];
                float vj=vj_vec[j];
                float kecta_ij_norm=(pt_i.Pos()-pt_j.Pos()).norm();
                float norm_pos_diff=pt_i.Normal().transpose()*(pt_i.Pos()-pt_j.Pos());
                float alpha_ij=(float)std::exp(-std::pow(norm_pos_diff/theta,2));

                pos_update_j+=alpha_ij*vj*pt_j.Pos();
                pos_update_j_sum+=alpha_ij*vj;
            }
            pos_update_j/=pos_update_j_sum;

            float wii=1.0f;
            for(int ii=0;ii<ear_pt_vec_i.size();ii++)
            {
                if (i == ii)
                    continue;
                EarPt pt_ii=ear_pt_vec_i[ii];
                Eigen::Vector3f pos_diff=pt_i.Pos()-pt_ii.Pos();
                float pos_diff_norm=pos_diff.norm();
                wii+=std::exp(-(pos_diff_norm*pos_diff_norm)/(theta*theta));
            }

            Eigen::Vector3f pos_update_ii(0.0f,0.0f,0.0f);
            float pos_update_ii_sum=0.0f;
            for(int ii=0;ii<ear_pt_vec_i.size();ii++)
            {
                if(i==ii)
                    continue;

                EarPt pt_ii=ear_pt_vec_i[ii];
                Eigen::Vector3f pos_diff=pt_i.Pos()-pt_ii.Pos();
                float pos_diff_norm=pos_diff.norm();
                float theta_ii=std::exp(-(pos_diff_norm*pos_diff_norm)/(theta*theta));
                float beta_ii=theta_ii/pos_diff_norm;

                pos_update_ii+=beta_ii*wii*pos_diff;
                pos_update_ii_sum+=beta_ii*wii;
            }


            Eigen::Vector3f pos_update=pos_update_j/pos_update_j_sum+(u/pos_update_ii_sum)*pos_update_ii;
            EarPt pttmp=pt_i;
            pttmp.x=pos_update(0);
            pttmp.y=pos_update(1);
            pttmp.z=pos_update(2);
            ear_pt_vec_i_tmp.push_back(pttmp);
        }
        ear_pt_vec_i.swap(ear_pt_vec_i_tmp);
        std::chrono::steady_clock::time_point tp3=std::chrono::steady_clock::now();
        std::chrono::steady_clock::duration span2=std::chrono::duration_cast<std::chrono::seconds>(tp3-tp2);
        std::cout<<"Position Smooth Done."<<span2.count()<<std::endl<<std::endl;
    }

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_smooth(new pcl::PointCloud<pcl::PointXYZINormal>);
    for(const EarPt &pt : ear_pt_vec_i)
        cloud_smooth->push_back(pt.ToPclPt());

    std::string filename="ear.pcd";
    pcl::io::savePCDFileASCII(filename,*cloud_smooth);

}