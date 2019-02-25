//
// Created by whubooks on 18-5-1.
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <random>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/flann_search.h>

#include <Util.h>
#include <CloudUtil.h>

//struct EarPt
//{
//    float x;
//    float y;
//    float z;
//
//    float x_normal;
//    float y_normal;
//    float z_normal;
//
//    float intensity;
//
//    EarPt()
//    {
//        x = 0.0f;
//        y = 0.0f;
//        z = 0.0f;
//        x_normal = 0.0f;
//        y_normal = 0.0f;
//        z_normal = 0.0f;
//        intensity = 0.0f;
//    }
//    EarPt(float _x,float _y,float _z,float _x_normal,float _y_normal,float _z_normal,float _intensity)
//    {
//        x = _x;
//        y = _y;
//        z = _z;
//        x_normal = _x_normal;
//        y_normal = _y_normal;
//        z_normal = _z_normal;
//        intensity = _intensity;
//    }
//    EarPt(const pcl::PointXYZINormal &pt)
//    {
//        x = pt.x;
//        y = pt.y;
//        z = pt.z;
//        x_normal = pt.normal_x;
//        y_normal = pt.normal_y;
//        z_normal = pt.normal_z;
//        intensity = pt.intensity;
//    }
//
//    pcl::PointXYZINormal ToPclPt() const
//    {
//        pcl::PointXYZINormal pt;
//        pt.x = x;
//        pt.y = y;
//        pt.z = z;
//        pt.intensity = intensity;
//        pt.normal_x = x_normal;
//        pt.normal_y = y_normal;
//        pt.normal_z = z_normal;
//        return pt;
//    }
//
//    Eigen::Vector3f Pos() const
//    {
//        return Eigen::Vector3f(x,y,z);
//    }
//    Eigen::Vector3f Normal() const
//    {
//        return Eigen::Vector3f(x_normal,y_normal,z_normal);
//    }
//
//};
//typedef std::vector<EarPt> EarPtVec;
//
//EarPtVec InitializeRandom(EarPtVec vec,double ratio)
//{
//    EarPtVec result;
//
//    static std::default_random_engine e;
//    e.seed(std::time(nullptr));
//    std::uniform_int_distribution<> uniform(0,1000000);
//
//    int n=std::ceil(vec.size()*ratio);
//    for(int i=0;i<n||vec.empty();i++)
//    {
//        int index=uniform(e)%vec.size();
//        EarPt pt=vec[index];
//        result.push_back(pt);
//
//        EarPtVec::iterator iter=vec.begin()+index;
//        vec.erase(iter);
//    }
//    return result;
//}
//
//EarPtVec KnnSearch(const EarPtVec &vec,const EarPt &pt,int k)
//{
//    typedef std::pair<EarPt, double> pair_dis;
//    std::vector<std::pair<EarPt, double>> map;
//    for (const EarPt &tmp : vec)
//    {
//        if (tmp.x == pt.x && tmp.y == pt.y && tmp.z == pt.z && tmp.intensity == pt.intensity &&
//            tmp.x_normal == pt.x_normal && tmp.y_normal == pt.y_normal && tmp.z_normal == pt.z_normal)
//            continue;
//
//        double dis =
//                (pt.x - tmp.x) * (pt.x - tmp.x) + (pt.y - tmp.y) * (pt.y - tmp.y) + (pt.z - tmp.z) * (pt.z - tmp.z);
//        map.push_back(std::make_pair(tmp, dis));
//    }
//
//    std::sort(map.begin(), map.end(), [](const pair_dis &x1, const pair_dis &x2) {
//        return x1.second < x2.second;
//    });
//
//    EarPtVec result;
//    for (int i = 0; i < k; i++)
//        result.push_back(map[i].first);
//    return result;
//}
//
//pcl::PointCloud<pcl::PointXYZINormal>::Ptr Bilateral(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud,
//                                                     const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud_filter)
//{
//    const int k = 16;
//    float theta = 0.5;
//    theta=std::sqrt(cloud->width*cloud->width+cloud->height*cloud->height)/std::sqrt(cloud->size());
//    float thetaN = 15.0f;
//    float cos_thetaN = std::cos((float) M_PI * thetaN / 180.0f);
//
//    pcl::KdTreeFLANN<pcl::PointXYZINormal> kdTreeFLANN;
//    kdTreeFLANN.setInputCloud(cloud_filter);
//
//    pcl::PointCloud<pcl::PointXYZINormal>::Ptr result(new pcl::PointCloud<pcl::PointXYZINormal>);
//    for (const pcl::PointXYZINormal &pt : cloud_filter->points)
//    {
//        Eigen::Vector3f pos(pt.x, pt.y, pt.z);
//        float normal_x = pt.normal_x;
//        float normal_y = pt.normal_y;
//        float normal_z = pt.normal_z;
//        float curvature = pt.curvature;
//
//        std::vector<int> vindices;
//        std::vector<float> vdistance;
//        kdTreeFLANN.nearestKSearch(pt, k, vindices, vdistance);
//
//        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vpos_knn, vnormal_knn;
//        for (const int &index : vindices)
//        {
//            pcl::PointXYZINormal pt_knn = cloud_filter->points[index];
//            Eigen::Vector3f pos_knn(pt_knn.x, pt_knn.y, pt_knn.z);
//            Eigen::Vector3f normal_knn(pt_knn.normal_x, pt_knn.normal_y, pt_knn.normal_z);
//
//            vpos_knn.push_back(pos_knn);
//            vnormal_knn.push_back(normal_knn);
//        }
//
//        //normal smooth
//        Eigen::Vector3f normal(normal_x, normal_y, normal_z);
//        Eigen::Vector3f normal_update(0.0f, 0.0f, 0.0f);
//        float normal_sum = 0.0f;
//        for (int j = 0; j < vpos_knn.size(); j++)
//        {
//            Eigen::Vector3f pos_knn = vpos_knn[j];
//            Eigen::Vector3f normal_knn = vnormal_knn[j];
//
//            float pos_product=(pos-pos_knn).transpose()*(pos-pos_knn);
//            float exp_pos = std::exp(-pos_product / (theta * theta));
//            float exp_normal = std::exp(
//                    -std::pow((1 - normal.transpose() * normal_knn) / (1 - cos_thetaN), 2));
//
//            normal_update += exp_pos * exp_normal * normal;
//            normal_sum += exp_pos * exp_normal;
//        }
//        normal_update /= normal_sum;
//
//        //position smooth
//        //global smooth
//        Eigen::Vector3f sum_pos_alpha_vj(0.0f,0.0f,0.0f);
//        float sum_alpha_vj=0.0f;
//        for(int j=0;j<cloud->points.size();j++)
//        {
//            pcl::PointXYZINormal pt_origin=cloud->points[j];
//            Eigen::Vector3f pos_origin(pt_origin.x,pt_origin.y,pt_origin.z);
//
//            //calculate vj and alpha
//            float vj=1.0f;
//            for(int jj=0;jj<cloud->points.size();jj++)
//            {
//                if(j==jj)
//                    continue;
//
//                pcl::PointXYZINormal pt_tmp=cloud->points[jj];
//                Eigen::Vector3f pos_tmp(pt_tmp.x,pt_tmp.y,pt_tmp.z);
//                vj+=std::exp(std::pow(-(pos_origin-pos_tmp).norm(),2)/(theta*theta));
//            }
//
//            //calculate alpha
//            Eigen::Vector3f ketha=pos-pos_origin;
//            float alpha=std::exp(std::pow(-normal_update.transpose()*ketha,2)/(theta*theta))/ketha.norm();
//
//            sum_pos_alpha_vj+=(alpha/vj)*pos_origin;
//            sum_alpha_vj+=(alpha/vj);
//        }
//
//        //local smooth
//        Eigen::Vector3f sum_pos_beta_wi(0.0f,0.0f,0.0f);
//        float sum_beta_wi=0.0f;
//        for (int j = 0; j < vpos_knn.size(); j++)
//        {
//            Eigen::Vector3f pos_knn = vpos_knn[j];
//            Eigen::Vector3f delta=pos-pos_knn;
//
//            float wi=1.0f;
//            //calculate wi
//            for(int jj=0;jj<vpos_knn.size();jj++)
//            {
//                if(j==jj)
//                    continue;
//                Eigen::Vector3f pos_tmp=vpos_knn[jj];
//                wi+=std::exp(std::pow(-(pos-pos_tmp).norm(),2)/(theta*theta));
//            }
//
//            //calculate beta
//            float beta=std::exp(std::pow(delta.norm(),2));
//
//            sum_pos_beta_wi+=beta*wi*delta;
//            sum_beta_wi+=beta*wi;
//        }
//
//        Eigen::Vector3f pos_update=sum_pos_alpha_vj/sum_alpha_vj+0.45*sum_pos_beta_wi/sum_beta_wi;
//
//        pcl::PointXYZINormal pt_bilateral;
//        pt_bilateral.x=pos_update(0);
//        pt_bilateral.y=pos_update(1);
//        pt_bilateral.z=pos_update(2);
//        pt_bilateral.normal_x=normal_update(0);
//        pt_bilateral.normal_y=normal_update(1);
//        pt_bilateral.normal_z=normal_update(2);
//        result->push_back(pt_bilateral);
//    }
//    return result;
//}

Eigen::Vector3f Pos(const pcl::PointXYZINormal &pt)
{
    return Eigen::Vector3f(pt.x,pt.y,pt.z);
}

Eigen::Vector3f Nor(const pcl::PointXYZINormal &pt)
{
    return Eigen::Vector3f(pt.normal_x,pt.normal_y,pt.normal_z);
}

PCM::CloudXYZINorPtr Init(PCM::CloudXYZINorPtr cloud, double ratio)
{
    PCM::CloudXYZINorPtr result(new pcl::PointCloud<pcl::PointXYZINormal>);

    std::vector<pcl::PointXYZINormal> vec;
    vec.assign(cloud->points.begin(), cloud->points.end());
    unsigned seed=std::time(nullptr);
    std::shuffle(vec.begin(), vec.end(),std::default_random_engine(seed));

    for (int i = 0; i < ratio * cloud->size(); i++)
        result->push_back(cloud->points[i]);

    return result;
}

std::vector<float> CalVJVec(PCM::CloudXYZINorPtr cloud,const float &theta_2)
{
    pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZINormal>);
    flann->setInputCloud(cloud);
    flann->setSortedResults(true);

    std::vector<float> vec;
    for(int i=0;i<cloud->size();i++)
    {
        pcl::PointXYZINormal pt_i=cloud->points[i];

        std::vector<int> vindices;
        std::vector<float> vdistance;
        flann->nearestKSearch(pt_i,16,vindices,vdistance);

        float w=1.0f;
        for(int j=0;j<vindices.size();j++)
        {
            if(vdistance[j]==0)
                continue;

            float dis_j=vdistance[j];
            w+=std::exp(-dis_j*dis_j/theta_2);
        }
        vec.push_back(w);
    }
    return vec;
}

typedef std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> NorVec;

PCM::CloudXYZINorPtr NorUpdate(PCM::CloudXYZINorPtr cloud,const float &theta_2,const float &cos_thetaN)
{
    pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZINormal>);
    flann->setInputCloud(cloud);
//    flann->setSortedResults(true);

    PCM::CloudXYZINorPtr result(new pcl::PointCloud<pcl::PointXYZINormal>);
    for(int i=0;i<cloud->size();i++)
    {
        pcl::PointXYZINormal pt=cloud->points[i];

        std::vector<int> vindices;
        std::vector<float> vdistance;
        flann->nearestKSearch(pt, 16, vindices, vdistance);

        Eigen::Vector3f nor_update(0.0f,0.0f,0.0f);
        float sum=0.0000000001f;
        for (int j = 0; j < vindices.size(); j++)
        {
            if (vdistance[j] == 0)
                continue;

            pcl::PointXYZINormal pt_j = cloud->points[vindices[j]];
            float dis_j = vdistance[j];
            float theta_dis_j = std::exp(-dis_j * dis_j / theta_2);

            float nor_dot = Nor(pt).dot(Nor(pt_j));
            float fy_nor_dot = std::exp(-((1.0f - nor_dot) * (1.0f - nor_dot)) / ((1.0f - cos_thetaN) * (1.0f - cos_thetaN)));

            nor_update += theta_dis_j * fy_nor_dot * Nor(pt_j);
            sum += theta_dis_j * fy_nor_dot;
        }
        nor_update/=sum;
        nor_update.norm();

        float nx=nor_update(0),ny=nor_update(1),nz=nor_update(2);

        pcl::PointXYZINormal pt_update(pt);
        pt_update.normal_x = nor_update(0);
        pt_update.normal_y = nor_update(1);
        pt_update.normal_z = nor_update(2);
        result->push_back(pt_update);
    }
    return result;
}

NorVec PosUpdate(PCM::CloudXYZINorPtr cloud_global,PCM::CloudXYZINorPtr cloud_local,const std::vector<float> &vj_global,const float &theta_2)
{
    pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr flann_global(new pcl::KdTreeFLANN<pcl::PointXYZINormal>);
    flann_global->setInputCloud(cloud_global);

    NorVec result;
    for(int i=0;i<cloud_local->size();i++)
    {
        pcl::PointXYZINormal pt=cloud_local->points[i];

        std::vector<int> vindices;
        std::vector<float> vdistance;
        flann_global->nearestKSearch(pt,16,vindices,vdistance);

        Eigen::Vector3f pos_update(0.0f,0.0f,0.0f);
        float sum=0.0000000001f;
        for(int j=0;j<vindices.size();j++)
        {
            pcl::PointXYZINormal pt_j=cloud_global->points[vindices[j]];
            float vj_j=vj_global[vindices[j]];

            float dis_j=vdistance[j];
            if(dis_j==0)
                continue;
            float norm_dis_j=Nor(pt).dot(Pos(pt)-Pos(pt_j));
            float alpha_dis_j=std::exp(-norm_dis_j*norm_dis_j/theta_2);

            pos_update+=alpha_dis_j*vj_j*Pos(pt_j);
            sum+=alpha_dis_j*vj_j;
        }
        pos_update/=sum;
        result.push_back(pos_update);
    }
    return result;
}


NorVec PosUpdate2(PCM::CloudXYZINorPtr cloud,const std::vector<float> &vj_local,const float theta_2)
{
    pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZINormal>);
    flann->setInputCloud(cloud);
    flann->setSortedResults(true);

    NorVec result;
    for(int i=0;i<cloud->size();i++)
    {
        pcl::PointXYZINormal pt=cloud->points[i];
        float w=vj_local[i];

        std::vector<int> vindices;
        std::vector<float> vdistance;
        flann->nearestKSearch(pt,16,vindices,vdistance);

        Eigen::Vector3f pos_update(0.0f,0.0f,0.0f);
        float sum=0.0000000001f;
        for(int j=0;j<vindices.size();j++)
        {
            if(vdistance[j]==0)
                continue;

            pcl::PointXYZINormal pt_j=cloud->points[vindices[j]];
            float dis=vdistance[j];
            float theta_i=std::exp(-dis*dis/theta_2);
            float beta_i=theta_i/dis;

            pos_update+=beta_i*w*(Pos(pt)-Pos(pt_j));
            sum+=beta_i*w;
        }
        pos_update/=sum;
        result.push_back(pos_update);
    }
    return result;

}


int main(int argc,char **argv)
{
    std::string pcd_file =(argc>1)?std::string(argv[1]): "50_mls.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcd_file, *cloud);

//    PCM::ConditionFilter(cloud);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setSearchMethod(kdTree);
    normalEstimation.setKSearch(16);
    normalEstimation.compute(*normals);

    PCM::CloudXYZINorPtr cloud_normal(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_normal);
    std::cout << "Calculate Normal by PCA Done." << std::endl;

    float minx = 1000000000000.0f, maxx = -1000000000000.0f;
    float miny = 1000000000000.0f, maxy = -1000000000000.0f;
    float minz = 1000000000000.0f, maxz = -1000000000000.0f;
    for (const pcl::PointXYZINormal &pt : cloud_normal->points)
    {
        minx = minx < pt.x ? minx : pt.x;
        maxx = maxx > pt.x ? maxx : pt.x;
        miny = miny < pt.y ? miny : pt.y;
        maxy = maxy > pt.y ? maxy : pt.y;
        minz = minz < pt.z ? minz : pt.z;
        maxz = maxz > pt.z ? maxz : pt.z;
    }

    const int K = 16;
    const float theta = std::sqrt((maxx - minx) * (maxy - miny) * (maxz - minz) / cloud->size());
    const float theta_2 = theta * theta;
    const float thetaN = 15.0f;
    const float cos_thetaN = std::cos((float) M_PI * thetaN / 180.0f);
    const float u = 0.45;

    std::cout << "theta ~ " << theta << std::endl;

    std::vector<float> vj_vec = CalVJVec(cloud_normal, theta_2);
    PCM::CloudXYZINorPtr cloud_normal_init = Init(cloud_normal, 0.8);

    std::cout << "Initialize Done." << std::endl;

    std::string dir=Util::SplitNameWithoutExt(pcd_file)+Util::GetNameFromTime();
    Util::DirBuild(dir);
    dir+="/";

    PCM::CloudXYZIPtr cloud_ear(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*cloud_normal_init,*cloud_ear);

    std::string ear_file=dir+"random.pcd";
    pcl::io::savePCDFileBinary(ear_file,*cloud_ear);

    for (int iter = 0; iter < 5; iter++)
    {
        std::cout<<std::endl << "Iteration ~ " << iter << std::endl;

        /// smooth normal
        PCM::CloudXYZINorPtr cloud_normal_tmp=NorUpdate(cloud_normal_init,theta_2,cos_thetaN);
        std::cout<<"Smooth Normal Done."<<std::endl;

        std::vector<float> vj_tmp = CalVJVec(cloud_normal_tmp, theta_2);
        std::cout<<"Calculate VJ of Smooth Cloud Done."<<std::endl;

        /// smooth position
        PCM::CloudXYZINorPtr cloud_normal_smooth(new pcl::PointCloud<pcl::PointXYZINormal>);
        NorVec pos_update_global=PosUpdate(cloud_normal,cloud_normal_tmp,vj_vec,theta_2);
        NorVec pos_update_local=PosUpdate2(cloud_normal_tmp,vj_tmp,theta_2);
        assert(pos_update_global.size()==pos_update_local.size());
        for(int i=0;i<cloud_normal_tmp->size();i++)
        {
            pcl::PointXYZINormal pt=cloud_normal_tmp->points[i];

            Eigen::Vector3f pos_update=pos_update_global[i]+u*pos_update_local[i];
            pt.x=pos_update(0);
            pt.y=pos_update(1);
            pt.z=pos_update(2);
            cloud_normal_smooth->push_back(pt);
        }
        std::cout<<"Smooth Position Done."<<std::endl;

        std::swap(cloud_normal_init, cloud_normal_smooth);

        PCM::CloudXYZIPtr cloud_ear_i(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*cloud_normal_init,*cloud_ear_i);

        std::string ear_file=dir+std::to_string(iter)+".pcd";
        pcl::io::savePCDFileBinary(ear_file,*cloud_ear_i);
    }

    return 1;
}

//    for(int iter=0;iter<3;iter++)
//    {
//        std::cout<<"Iteration ~ "<<iter<<std::endl;
//        EarPtVec ear_pt_vec_i_tmp;
//
//        std::chrono::steady_clock::time_point tp1=std::chrono::steady_clock::now();
//        //smooth normal
//        for(const EarPt &pt_i : ear_pt_vec_i)
//        {
//            EarPtVec vec_knn=KnnSearch(ear_pt_vec_i,pt_i,k);
//            Eigen::Vector3f normal_update(0.0f,0.0f,0.0f);
//            float sum=0.0f;
//            for(const EarPt &pt_knn: vec_knn)
//            {
//                float pos_diff_norm=(pt_i.Pos()-pt_knn.Pos()).norm();
//                float theta_pos_diff_norm=std::exp(-(pos_diff_norm*pos_diff_norm)/(theta*theta));
//
//                float normal_product=pt_i.Normal().transpose()*pt_knn.Normal();
//                float fry_normal_product=(float)std::exp(-std::pow((1-normal_product)/(1-cos_thetaN),2));
//
//                normal_update+=theta_pos_diff_norm*fry_normal_product*pt_knn.Normal();
//                sum+=theta_pos_diff_norm*fry_normal_product;
//            }
//            normal_update/=sum;
//
//            EarPt pttmp=pt_i;
//            pttmp.x_normal=normal_update(0);
//            pttmp.y_normal=normal_update(1);
//            pttmp.z_normal=normal_update(2);
//            ear_pt_vec_i_tmp.push_back(pttmp);
//        }
//        ear_pt_vec_i.swap(ear_pt_vec_i_tmp);
//        ear_pt_vec_i_tmp.clear();
//
//        std::chrono::steady_clock::time_point tp2=std::chrono::steady_clock::now();
//        std::chrono::steady_clock::duration span1=std::chrono::duration_cast<std::chrono::seconds>(tp2-tp1);
//        std::cout<<"Normal Smooth Done."<<span1.count()<<std::endl;
//
//        //smooth position
//        for(int i=0;i<ear_pt_vec_i.size();i++)
//        {
//            EarPt pt_i=ear_pt_vec_i[i];
//
//            Eigen::Vector3f pos_update_j(0.0f,0.0f,0.0f);
//            float pos_update_j_sum=0.0f;
//            for(int j=0;j<ear_pt_vec_j.size();j++)
//            {
//                EarPt pt_j=ear_pt_vec_j[j];
//                float vj=vj_vec[j];
//                float kecta_ij_norm=(pt_i.Pos()-pt_j.Pos()).norm();
//                float norm_pos_diff=pt_i.Normal().transpose()*(pt_i.Pos()-pt_j.Pos());
//                float alpha_ij=(float)std::exp(-std::pow(norm_pos_diff/theta,2));
//
//                pos_update_j+=alpha_ij*vj*pt_j.Pos();
//                pos_update_j_sum+=alpha_ij*vj;
//            }
//            pos_update_j/=pos_update_j_sum;
//
//            float wii=1.0f;
//            for(int ii=0;ii<ear_pt_vec_i.size();ii++)
//            {
//                if (i == ii)
//                    continue;
//                EarPt pt_ii=ear_pt_vec_i[ii];
//                Eigen::Vector3f pos_diff=pt_i.Pos()-pt_ii.Pos();
//                float pos_diff_norm=pos_diff.norm();
//                wii+=std::exp(-(pos_diff_norm*pos_diff_norm)/(theta*theta));
//            }
//
//            Eigen::Vector3f pos_update_ii(0.0f,0.0f,0.0f);
//            float pos_update_ii_sum=0.0f;
//            for(int ii=0;ii<ear_pt_vec_i.size();ii++)
//            {
//                if(i==ii)
//                    continue;
//
//                EarPt pt_ii=ear_pt_vec_i[ii];
//                Eigen::Vector3f pos_diff=pt_i.Pos()-pt_ii.Pos();
//                float pos_diff_norm=pos_diff.norm();
//                float theta_ii=std::exp(-(pos_diff_norm*pos_diff_norm)/(theta*theta));
//                float beta_ii=theta_ii/pos_diff_norm;
//
//                pos_update_ii+=beta_ii*wii*pos_diff;
//                pos_update_ii_sum+=beta_ii*wii;
//            }
//
//
//            Eigen::Vector3f pos_update=pos_update_j/pos_update_j_sum+(u/pos_update_ii_sum)*pos_update_ii;
//            EarPt pttmp=pt_i;
//            pttmp.x=pos_update(0);
//            pttmp.y=pos_update(1);
//            pttmp.z=pos_update(2);
//            ear_pt_vec_i_tmp.push_back(pttmp);
//        }
//        ear_pt_vec_i.swap(ear_pt_vec_i_tmp);
//        std::chrono::steady_clock::time_point tp3=std::chrono::steady_clock::now();
//        std::chrono::steady_clock::duration span2=std::chrono::duration_cast<std::chrono::seconds>(tp3-tp2);
//        std::cout<<"Position Smooth Done."<<span2.count()<<std::endl<<std::endl;
//    }
//
//    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_smooth(new pcl::PointCloud<pcl::PointXYZINormal>);
//    for(const EarPt &pt : ear_pt_vec_i)
//        cloud_smooth->push_back(pt.ToPclPt());
//
//    std::string filename="ear.pcd";
//    pcl::io::savePCDFileASCII(filename,*cloud_smooth);

//}