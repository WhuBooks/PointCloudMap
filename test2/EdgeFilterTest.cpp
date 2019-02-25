//
// Created by whubooks on 18-4-25.
//

#include <iostream>
#include <vector>
#include <string>
#include <mutex>
#include <thread>

#include <Util.h>
#include <CloudUtil.h>
#include <PointCloudProject.h>
#include <KittiData.h>
#include <LineSupportRegion.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#define WRITERESULT 1

/// back-project region
pcl::PointCloud<pcl::PointXYZ>::Ptr BackProject(const std::vector<cv::Point2i> &region_2d,const std::vector<float> &vdepth,const Eigen::Matrix3f &K,int rows,int cols)
{
    typedef std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> PosVec;
    PosVec pos_vec;
    for(const cv::Point2i &pos_2d : region_2d)
    {
        int col=pos_2d.x;
        int row=pos_2d.y;
        float depth=vdepth[row*cols+col];

        if(depth==-1)
            continue;

        pos_vec.push_back(Eigen::Vector3f(col,row,depth));
    }

    /// apply median filter on depth to reduce the error
    bool use_median=true;
    if(use_median)
    {
        PosVec median_pos_vec;
        const int N = 2;
        for (int ii = 0; ii < pos_vec.size(); ii++)
        {
            std::vector<float> tmp_depth;
            for (const Eigen::Vector3f &pos : pos_vec)
            {
                Eigen::Vector2f v = pos.head<2>() - pos_vec[ii].head<2>();

                if (v.norm() <= N)
                    tmp_depth.push_back(pos(2));
            }

            std::sort(tmp_depth.begin(), tmp_depth.end());
            float median_depth = 0.0f;
            if (tmp_depth.size() % 2 == 1)
                median_depth = tmp_depth[(tmp_depth.size() - 1) / 2];
            else
                median_depth = (tmp_depth[tmp_depth.size() / 2] + tmp_depth[tmp_depth.size() / 2 - 1]) / 2.0f;

            float col_ii = pos_vec[ii](0);
            float row_ii = pos_vec[ii](1);
            Eigen::Vector3f pt_3d =
                    K.inverse() * Eigen::Vector3f(col_ii * median_depth, row_ii * median_depth, median_depth);
            median_pos_vec.push_back(pt_3d);
        }
        pos_vec.swap(median_pos_vec);
    }
    else
    {
        PosVec tmp_pos_vec;
        for(const Eigen::Vector3f &pos : pos_vec)
        {
            Eigen::Vector3f tmp_pos=K.inverse()*Eigen::Vector3f(pos(0)*pos(2),pos(1)*pos(2),pos(2));
            tmp_pos_vec.push_back(tmp_pos);
        }
        pos_vec.swap(tmp_pos_vec);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(const Eigen::Vector3f &pos : pos_vec)
    {
        pcl::PointXYZ pt;
        pt.x=pos(0);
        pt.y=pos(1);
        pt.z=pos(2);
        cloud->push_back(pt);
    }
    return cloud;

}

/// back-project line
pcl::PointCloud<pcl::PointXYZ>::Ptr BackProject2(const std::vector<cv::Point2i> &region_2d,const std::vector<float> &vdepth,const Eigen::Matrix3f &K,int rows,int cols)
{
    typedef std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> PosVec;
    PosVec pos_vec;
    for(const cv::Point2i &pos_2d : region_2d)
    {
        int col=pos_2d.x;
        int row=pos_2d.y;
        float depth=vdepth[row*cols+col];

        if(depth==-1)
            continue;

        pos_vec.push_back(Eigen::Vector3f(col,row,depth));
    }

    /// apply median filter on depth to reduce the error
    bool use_median=true;
    if(use_median)
    {
        PosVec median_pos_vec;
        const int N = 2;
        for (int ii = 0; ii < pos_vec.size(); ii++)
        {
            std::vector<float> tmp_depth;
            for (int jj = std::max(0, ii - N); jj <= std::min(ii + N, (int) pos_vec.size() - 1); jj++)
            {
                float depth_jj = pos_vec[jj](2);
                tmp_depth.push_back(depth_jj);
            }

            std::sort(tmp_depth.begin(), tmp_depth.end());
            float median_depth = 0.0f;
            if (tmp_depth.size() % 2 == 1)
                median_depth = tmp_depth[(tmp_depth.size() - 1) / 2];
            else
                median_depth = (tmp_depth[tmp_depth.size() / 2] + tmp_depth[tmp_depth.size() / 2 - 1]) / 2.0f;

            float col_ii = pos_vec[ii](0);
            float row_ii = pos_vec[ii](1);
            Eigen::Vector3f pt_3d =
                    K.inverse() * Eigen::Vector3f(col_ii * median_depth, row_ii * median_depth, median_depth);
            median_pos_vec.push_back(pt_3d);
        }
        pos_vec.swap(median_pos_vec);
    }
    else
    {
        PosVec tmp_pos_vec;
        for(const Eigen::Vector3f &pos : pos_vec)
        {
            Eigen::Vector3f tmp_pos=K.inverse()*Eigen::Vector3f(pos(0)*pos(2),pos(1)*pos(2),pos(2));
            tmp_pos_vec.push_back(tmp_pos);
        }
        pos_vec.swap(tmp_pos_vec);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(const Eigen::Vector3f &pos : pos_vec)
    {
        pcl::PointXYZ pt;
        pt.x=pos(0);
        pt.y=pos(1);
        pt.z=pos(2);
        cloud->push_back(pt);
    }
    return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr SacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if(cloud->empty())
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::SACSegmentation<pcl::PointXYZ> sac;
    sac.setInputCloud(cloud);
    sac.setOptimizeCoefficients(true);
    sac.setModelType(pcl::SACMODEL_LINE);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setDistanceThreshold(0.05f);
    sac.setMaxIterations(10000);

    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficents(new pcl::ModelCoefficients);
    sac.segment(*indices, *coefficents);
    if(indices->indices.size()<3)
    {
        std::cerr<<"Sac Line Failed."<<std::endl;
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_edge(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *indices, *cloud_edge);
    return cloud_edge;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SacLine2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::vector<float> &coeff)
{
    if(cloud->empty())
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::SACSegmentation<pcl::PointXYZ> sac;
    sac.setInputCloud(cloud);
    sac.setOptimizeCoefficients(true);
    sac.setModelType(pcl::SACMODEL_LINE);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setDistanceThreshold(0.05f);
    sac.setMaxIterations(10000);

    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficents(new pcl::ModelCoefficients);
    sac.segment(*indices, *coefficents);
    if(indices->indices.size()<3)
    {
        std::cerr<<"Sac Line Failed."<<std::endl;
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }
    coeff.assign(coefficents->values.begin(),coefficents->values.end());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_edge(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *indices, *cloud_edge);
    return cloud_edge;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr InterpolateLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if(cloud->empty())
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::SACSegmentation<pcl::PointXYZ> sac;
    sac.setInputCloud(cloud);
    sac.setOptimizeCoefficients(true);
    sac.setModelType(pcl::SACMODEL_LINE);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setDistanceThreshold(0.05f);
    sac.setMaxIterations(10000);

    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficents(new pcl::ModelCoefficients);
    sac.segment(*indices, *coefficents);

    if(indices->indices.size()<3 || coefficents->values.size()!=6)
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_edge(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *indices, *cloud_edge);

    Eigen::Vector3f line_pt(coefficents->values[0],coefficents->values[1],coefficents->values[2]);
    Eigen::Vector3f line_dir(coefficents->values[3],coefficents->values[4],coefficents->values[5]);
    float max_dis=-100000000000.0f,min_dis=10000000000000.0f;
    for(const pcl::PointXYZ &pt : cloud_edge->points)
    {
        Eigen::Vector3f vec=Eigen::Vector3f(pt.x,pt.y,pt.z)-line_pt;
        float dis=line_dir.dot(vec)/line_dir.norm();
        min_dis=(min_dis<dis)?min_dis:dis;
        max_dis=(max_dis>dis)?max_dis:dis;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_straghit_edge(new pcl::PointCloud<pcl::PointXYZ>);
    for(float dis=min_dis;dis<=max_dis;dis+=0.05f)
    {
        Eigen::Vector3f pos=line_pt+dis*line_dir;
        pcl::PointXYZ pt;
        pt.x=pos(0);
        pt.y=pos(1);
        pt.z=pos(2);
        cloud_straghit_edge->push_back(pt);
    }
    return cloud_straghit_edge;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SacPlanar(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::vector<float> &coeff)
{
    if(cloud->empty())
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::SACSegmentation<pcl::PointXYZ> sac;
    sac.setInputCloud(cloud);
    sac.setOptimizeCoefficients(true);
    sac.setModelType(pcl::SACMODEL_PLANE);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setDistanceThreshold(0.03f);
    sac.setMaxIterations(10000);

    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficents(new pcl::ModelCoefficients);
    sac.segment(*indices, *coefficents);

    if(indices->indices.size()<5)
    {
        std::cerr<<"Sac Planar Failed."<<std::endl;
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    coeff.assign(coefficents->values.begin(),coefficents->values.end());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_planar(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *indices, *cloud_planar);
    return cloud_planar;
}

std::vector<cv::Point2f> Cloud2Pixel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const Eigen::Matrix3f &K)
{
    std::vector<cv::Point2f> pixels;
    for(const pcl::PointXYZ &pt : cloud->points)
    {
        Eigen::Vector3f pos(pt.x,pt.y,pt.z);
        Eigen::Vector3f pos_pixel=K*pos;

        Eigen::Vector2f pixel=pos_pixel.head<2>()/pos_pixel(2);
        pixels.push_back(cv::Point2f(pixel(0),pixel(1)));
    }
    return pixels;
}

/// only search the nearest point
pcl::PointCloud<pcl::PointXYZ>::Ptr Search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr target)
{
    if(target->empty())
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    flann->setInputCloud(cloud);

    const float radius=0.05f;
    std::vector<int> search_indices;
    for(const pcl::PointXYZ &pt : target->points)
    {
        std::vector<int> vindices;
        std::vector<float> vdistance;
        flann->radiusSearch(pt,radius,vindices,vdistance);

        if(vindices.empty())
            continue;

        search_indices.push_back(vindices.front());
    }

    std::sort(search_indices.begin(),search_indices.end());
    std::vector<int> unique_indices;
    int last_index=-100000000;
    for(const int &index : search_indices)
    {
        if(last_index!=index)
            unique_indices.push_back(index);
        last_index=index;
    }

    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    indices->indices.swap(unique_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_search(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud,*indices,*cloud_search);
    return cloud_search;
}

/// search the point within a radius
pcl::PointCloud<pcl::PointXYZ>::Ptr Search3(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr target)
{
    if(target->empty())
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    flann->setInputCloud(cloud);

    const float radius=0.05f;
    std::vector<int> search_indices;
    for(const pcl::PointXYZ &pt : target->points)
    {
        std::vector<int> vindices;
        std::vector<float> vdistance;
        flann->radiusSearch(pt,radius,vindices,vdistance);

        if(vindices.empty())
            continue;

//        search_indices.push_back(vindices.front());
        search_indices.insert(search_indices.end(),vindices.begin(),vindices.end());
    }

    std::sort(search_indices.begin(),search_indices.end());
    std::vector<int> unique_indices;
    int last_index=-100000000;
    for(const int &index : search_indices)
    {
        if(last_index!=index)
            unique_indices.push_back(index);
        last_index=index;
    }

    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    indices->indices.swap(unique_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_search(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud,*indices,*cloud_search);
    return cloud_search;
}


/// knn search
pcl::PointCloud<pcl::PointXYZ>::Ptr Search2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr target)
{
    if (target->empty())
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    flann->setInputCloud(cloud);

    const int K = 3;
    PCM::PosVec pos_vec;
    for (const pcl::PointXYZ &pt : target->points)
    {
        std::vector<int> vindices;
        std::vector<float> vdistance;
        flann->nearestKSearch(pt, K, vindices, vdistance);

        float x = 0.0f, y = 0.0f, z = 0.0f;
        Eigen::Vector3f pos(0.0f, 0.0f, 0.0f);
        for (const int &index : vindices)
        {
            pcl::PointXYZ pt_index = cloud->points[index];
            pos += Eigen::Vector3f(pt_index.x, pt_index.y, pt_index.z);
        }
        pos /= K;
        pos_vec.push_back(pos);
    }

    std::sort(pos_vec.begin(), pos_vec.end(), [](const Eigen::Vector3f &pos1, const Eigen::Vector3f &pos2) {
        return pos1(0) < pos2(0) || (pos1(0) == pos2(0) && pos1(1) < pos2(1)) ||
               (pos1(0) == pos2(0) && pos1(1) == pos2(1) && pos1(2) < pos2(2));
    });

    float x_last=1000000000.0f,y_last=1000000000.0f,z_last=100000000000.0f;
    PCM::CloudXYZPtr result(new pcl::PointCloud<pcl::PointXYZ>);
    for(const Eigen::Vector3f &pos : pos_vec)
    {
        if(x_last!=pos(0) || y_last!=pos(1) || z_last!=pos(2))
            result->push_back(pcl::PointXYZ(pos(0),pos(1),pos(2)));
        x_last=pos(0);
        y_last=pos(1);
        z_last=pos(2);
    }
    return result;
}


PCM::CloudXYZPtr Project2Planar(const std::vector<float> &coeff,PCM::CloudXYZPtr cloud)
{
    float nx=coeff[0],ny=coeff[1],nz=coeff[2],p=coeff[3];

    PCM::CloudXYZPtr project(new pcl::PointCloud<pcl::PointXYZ>);
    for(const pcl::PointXYZ &pt : cloud->points)
    {
        float x=pt.x,y=pt.y,z=pt.z;
        float t=(nx*x+ny*y+nz*z+p)/(nx*nx+ny*ny+nz*nz);

        float xp=x-nx*t;
        float yp=y-ny*t;
        float zp=z-nz*t;

        pcl::PointXYZ tmp(xp,yp,zp);
        project->push_back(tmp);
    }

    return project;
}

float PlanarDensity(PCM::CloudXYZPtr cloud)
{
    float minx=1000000000.0f,maxx=-10000000000.0f;
    float miny=1000000000.0f,maxy=-10000000000.0f;
    float minz=1000000000.0f,maxz=-10000000000.0f;

    for(const pcl::PointXYZ &pt : cloud->points)
    {
        minx=minx<pt.x?minx:pt.x;
        maxx=maxx>pt.x?maxx:pt.x;
        miny=miny<pt.y?miny:pt.y;
        maxy=maxy>pt.y?maxy:pt.y;
        minz=minz<pt.z?minz:pt.z;
        maxz=maxz>pt.z?maxz:pt.z;
    }

    float volume=(maxx-minx)*(maxy-miny)*(maxz-minz);
    return cloud->size()/(volume+0.00000001f);

}


/// using line and env ransac a new planar, then compare it with the env's planar
PCM::CloudXYZPtr LinePlanar(PCM::CloudXYZPtr line,PCM::CloudXYZPtr env,const std::vector<float> &planar_coeff)
{
    if(line->empty() || env->empty())
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    PCM::CloudXYZPtr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(const pcl::PointXYZ &pt : line->points)
        cloud->push_back(pt);
    for(const pcl::PointXYZ &pt : env->points)
        cloud->push_back(pt);

    std::vector<float> cloud_coeff;
    PCM::CloudXYZPtr cloud_sac=SacPlanar(cloud,cloud_coeff);

    PCM::CloudXYZPtr outliers(new pcl::PointCloud<pcl::PointXYZ>);
    float cloud_nx=cloud_coeff[0],cloud_ny=cloud_coeff[1],cloud_nz=cloud_coeff[2],cloud_p=cloud_coeff[3];
    for(const pcl::PointXYZ &pt : env->points)
    {
        float dis=cloud_nx*pt.x+cloud_ny*pt.y+cloud_nz*pt.z+cloud_p;
        if(dis<0.03f)
            continue;

        outliers->push_back(pt);
    }

    /// try ransac a planar using outliers
    std::vector<float> outliers_coeff;
    PCM::CloudXYZPtr outliers_sac_planar=SacPlanar(outliers,outliers_coeff);

    if(outliers_sac_planar->empty())
    {
        PCM::CloudXYZPtr outliers_sac_line=SacLine(outliers);
        /// if outliers can't ransac line, then the outliers is too less
        if(outliers_sac_line->empty())
        {
            std::cout<<"Line Project Planar."<<std::endl;
            PCM::CloudXYZPtr line_project_planar=Project2Planar(planar_coeff,line);
            PCM::CloudXYZPtr line_env=Search2(env,line_project_planar);
            return line_env;
        }
        std::cout<<"Outliers Ransac Line."<<std::endl;
        return outliers_sac_line;
    }

    std::cout<<"Line Project Outliers."<<std::endl;
    PCM::CloudXYZPtr line_project_outliers=Project2Planar(outliers_coeff,line);
    PCM::CloudXYZPtr line_env=Search2(outliers_sac_planar,line_project_outliers);
    return line_env;

}

/// using line and line_env ransac a new planar, then compare it with the env's planar
PCM::CloudXYZPtr LinePlanar2(PCM::CloudXYZPtr line,PCM::CloudXYZPtr env,const std::vector<float> &planar_coeff)
{
    if(line->empty() || env->empty())
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    /// try ransac a planar using line and line search
    PCM::CloudXYZPtr line_project_planar=Project2Planar(planar_coeff,line);
    PCM::CloudXYZPtr line_env=Search2(env,line_project_planar);
    PCM::CloudXYZPtr line_add(new pcl::PointCloud<pcl::PointXYZ>);
    for(const pcl::PointXYZ &pt : line->points)
        line_add->push_back(pt);
    for(const pcl::PointXYZ &pt : line_env->points)
        line_add->push_back(pt);
    std::vector<float> line_add_sac_planar_coeff;
    PCM::CloudXYZPtr line_add_sac_planar=SacPlanar(line_env,line_add_sac_planar_coeff);
    if(line_add_sac_planar->empty())
    {
        std::cout<<"Line Project Planar1."<<std::endl;
        return line_env;
    }

    /// get outliers of planar from line_add_planar
    PCM::CloudXYZPtr outliers(new pcl::PointCloud<pcl::PointXYZ>);
    float cloud_nx=line_add_sac_planar_coeff[0],cloud_ny=line_add_sac_planar_coeff[1];
    float cloud_nz=line_add_sac_planar_coeff[2],cloud_p=line_add_sac_planar_coeff[3];
    for(const pcl::PointXYZ &pt : env->points)
    {
        float dis=cloud_nx*pt.x+cloud_ny*pt.y+cloud_nz*pt.z+cloud_p;
        if(dis<0.03f)
            continue;
        outliers->push_back(pt);
    }

    /// try ransac a planar using outliers
    std::vector<float> outliers_coeff;
    PCM::CloudXYZPtr outliers_sac_planar=SacPlanar(outliers,outliers_coeff);
    if(outliers_sac_planar->empty())
    {
        PCM::CloudXYZPtr outliers_sac_line=SacLine(outliers);
        /// if outliers can't ransac line, then the outliers is too less
        if(outliers_sac_line->empty())
        {
            std::cout<<"Line Project Planar2."<<std::endl;
            return line_env;
        }
        std::cout<<"Outliers Ransac Line."<<std::endl;
        return outliers_sac_line;
    }

    std::cout<<"Line Project Outliers."<<std::endl;
    PCM::CloudXYZPtr line_project_outliers=Project2Planar(outliers_coeff,line);
    PCM::CloudXYZPtr line_outliers=Search2(outliers_sac_planar,line_project_outliers);
    return line_outliers;

}

/// using line and line_env ransac a new planar, compare its angle difference with the env's planar
PCM::CloudXYZPtr LinePlanar3(PCM::CloudXYZPtr line,PCM::CloudXYZPtr env,const std::vector<float> &planar_coeff)
{
    if(line->empty() || env->empty())
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    PCM::CloudXYZPtr line_project_env=Project2Planar(planar_coeff,line);

    std::vector<float> line_coeff;
    PCM::CloudXYZPtr line_sac=SacLine2(line_project_env,line_coeff);

    /// compute the search direction
    Eigen::Vector3f nor1(line_coeff[3],line_coeff[4],line_coeff[5]);
    Eigen::Vector3f nor2(planar_coeff[0],planar_coeff[1],planar_coeff[2]);
    Eigen::Vector3f dir=nor1.cross(nor2);

    PCM::CloudXYZPtr line_project_search(new pcl::PointCloud<pcl::PointXYZ>);
    for(const pcl::PointXYZ &pt_proj : line_project_env->points)
    {
        std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>> pos_dis;
        for(const pcl::PointXYZ &pt_env : env->points)
        {
            Eigen::Vector3f dir_tmp(pt_env.x-pt_proj.x,pt_env.y-pt_proj.y,pt_env.z-pt_proj.z);
            float angle=std::acos(dir_tmp.dot(dir)/(dir.norm()*dir_tmp.norm()))*180.0f/M_PI;
            float dis=dir_tmp.dot(dir)/dir.norm();
            if(angle<20.0f || angle>160.0f)
                pos_dis.push_back(Eigen::Vector4f(pt_env.x,pt_env.y,pt_env.z,dis));
        }

        if(pos_dis.empty())
            continue;

        /// median filter by distance
        std::sort(pos_dis.begin(),pos_dis.end(),[](const Eigen::Vector4f &p1,const Eigen::Vector4f &p2){
           return p1(3)<p2(3);
        });
        if(pos_dis.size()%2==0)
        {
            Eigen::Vector4f pos_median=(pos_dis[pos_dis.size()/2-1]+pos_dis[pos_dis.size()/2])/2.0f;
            line_project_search->push_back(pcl::PointXYZ(pos_median(0),pos_median(1),pos_median(2)));
        }
        else
        {
            Eigen::Vector4f pos_median=pos_dis[(pos_dis.size()-1)/2];
            line_project_search->push_back(pcl::PointXYZ(pos_median(0),pos_median(1),pos_median(2)));
        }
    }
    std::cout<<"Project Search In Planar ~ "<<line_project_search->size()<<std::endl;
    if(line_project_search->size()<5)
    {
        line_project_search->clear();
        line_project_search=Search2(env,line_project_env);
    }

    return line_project_search;

}

int main(int argc,char **argv)
{
    std::string pcd_file = (argc > 1) ? std::string(argv[1]) : "50_mls.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcd_file, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pos(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *cloud_pos);

    std::string calib_file = "../data/calib.txt";
    Eigen::Matrix3f K = PCM::KittiData::LoadKFromFile(calib_file);
    std::cout << "K ~ \n" << K << std::endl;

    std::string img_file = "../data/000050.png";
    cv::Mat img = cv::imread(img_file, cv::IMREAD_GRAYSCALE);
    int img_rows = img.rows;
    int img_cols = img.cols;
    std::cout << "Image Size ~ [ " << img_rows << " , " << img_cols << " ]" << std::endl;

#if WRITERESULT
    std::string dir="Filter"+Util::GetNameFromTime();
    Util::DirBuild(dir);
#else
    /// cloud viewer thread
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line_region(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_line_region(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right_line_region(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_env_region(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_env_region(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right_env_region(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("viewer"));
    visualizer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, 1);
    visualizer->initCameraParameters();

    visualizer->addPointCloud(cloud_line_region, "line_region");
    visualizer->addPointCloud(cloud_left_line_region, "left_line_region");
    visualizer->addPointCloud(cloud_right_line_region, "right_line_region");

    visualizer->addPointCloud(cloud_env_region, "env_region");
    visualizer->addPointCloud(cloud_left_env_region, "left_env_region");
    visualizer->addPointCloud(cloud_right_env_region, "right_env_region");

    std::thread edge_show = std::thread([&]() {
        std::cout << "Start Cloud Viewer!" << std::endl;
        while (!visualizer->wasStopped())
        {
            visualizer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << "Stop Cloud Viewer!" << std::endl;
    });
#endif

    PCM::PointCloudProject project;
    project.SetInput(cloud);
    project.SetK(K);
    project.SetImageSize(img_rows, img_cols);
    project.Project2();
    project.EDLScale();
    std::vector<float> origin_depth_vec = project.GetOriginDeph();
    std::vector<uchar> gray_shadow_vec = project.GetGrayShadowVec(true);

    cv::Mat img_lsr(img_rows, img_cols, CV_8UC1, gray_shadow_vec.data());
    cv::cvtColor(img_lsr, img_lsr, CV_GRAY2BGR);

    PCM::LineSupportRegion lsr(img_rows, img_cols, gray_shadow_vec);
    lsr.Detect3();

    std::vector<PCM::Line2D> line_vec = lsr.GetLine();
    std::vector<std::vector<cv::Point2i>> line_region_2d_vec = lsr.GetLineRegion();

    std::vector<PCM::Line2D> left_line_vec = lsr.GetLeftLine();
    std::vector<std::vector<cv::Point2i>> left_line_region_2d_vec = lsr.GetLeftLineRegion();

    std::vector<PCM::Line2D> right_line_vec = lsr.GetRightLine();
    std::vector<std::vector<cv::Point2i>> right_line_region_2d_vec = lsr.GetRightLineRegion();

    std::vector<std::vector<cv::Point2i>> env_region_2d_vec = lsr.GetEnvRegion();
    std::vector<std::vector<cv::Point2i>> left_env_region_2d_vec = lsr.GetLeftEnvRegion();
    std::vector<std::vector<cv::Point2i>> right_env_region_2d_vec = lsr.GetRightEnvRegion();
    std::cout << "Project and Detect ~ " << line_region_2d_vec.size() << std::endl<<std::endl;

    /// back project threshold
    const int line_back_project_thres = 10;
    const int env_back_project_thres=30;

    /// inliers ration threshold
    const double line_inlier_ratio_thres=0.3;
    const double env_inlier_ratio_thres=0.5;

    const double left_right_density_ratio_thres=0.5;

    for (int i = 0; i < line_vec.size(); i++)
    {
        std::cout <<std::endl<< "Iteration ~ " << i << std::endl;
        PCM::Line2D line_i = line_vec[i];
        std::vector<cv::Point2i> line_region_2d_i = line_region_2d_vec[i];

        PCM::Line2D left_line_i = left_line_vec[i];
        std::vector<cv::Point2i> left_line_region_2d_i = left_line_region_2d_vec[i];

        PCM::Line2D right_line_i = right_line_vec[i];
        std::vector<cv::Point2i> right_line_region_2d_i = right_line_region_2d_vec[i];

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line_region_i = BackProject2(line_region_2d_i, origin_depth_vec, K, img_rows, img_cols);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_line_region_i = BackProject2(left_line_region_2d_i, origin_depth_vec, K, img_rows, img_cols);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right_line_region_i = BackProject2(right_line_region_2d_i, origin_depth_vec, K, img_rows, img_cols);

        std::vector<cv::Point2i> env_region_2d_i = env_region_2d_vec[i];
        std::vector<cv::Point2i> left_env_region_2d_i = left_env_region_2d_vec[i];
        std::vector<cv::Point2i> right_env_region_2d_i = right_env_region_2d_vec[i];

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_env_region_i = BackProject(env_region_2d_i, origin_depth_vec, K, img_rows, img_cols);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_env_region_i = BackProject(left_env_region_2d_i, origin_depth_vec, K, img_rows, img_cols);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right_env_region_i = BackProject(right_env_region_2d_i, origin_depth_vec, K, img_rows, img_cols);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line_search_region_i = Search(cloud_pos, cloud_line_region_i);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_env_search_region_i = Search3(cloud_pos, cloud_left_env_region_i);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right_env_search_region_i = Search3(cloud_pos, cloud_right_env_region_i);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line_sac_region_i = SacLine(cloud_line_search_region_i);

        std::vector<float> left_env_coeff,right_env_coeff;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_env_sac_region_i = SacPlanar(cloud_left_env_search_region_i,left_env_coeff);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right_env_sac_region_i = SacPlanar(cloud_right_env_search_region_i,right_env_coeff);

        std::size_t d_c = line_region_2d_i.size();
        std::size_t d_l = left_env_region_2d_i.size();
        std::size_t d_r = right_env_region_2d_i.size();

        std::size_t b_c = cloud_line_region_i->size();
        std::size_t b_l = cloud_left_env_region_i->size();
        std::size_t b_r = cloud_right_env_region_i->size();

        std::size_t n_c = cloud_line_search_region_i->size();
        std::size_t n_l = cloud_left_env_search_region_i->size();
        std::size_t n_r = cloud_right_env_search_region_i->size();


        std::size_t i_c = cloud_line_sac_region_i->size();
        std::size_t i_l = cloud_left_env_sac_region_i->size();
        std::size_t i_r = cloud_right_env_sac_region_i->size();

//        std::cout << "[ d_c d_l d_r ] ~ [ " << d_c << " " << d_l << " " << d_r << " ]" << std::endl;
//        std::cout << "[ b_c b_l b_r ] ~ [ " << b_c << " " << b_l << " " << b_r << " ]" << std::endl;
//        std::cout << "[ n_c n_l n_r ] ~ [ " << n_c << " " << n_l << " " << n_r << " ]" << std::endl;
//        std::cout << "[ i_c i_l i_r ] ~ [ " << i_c << " " << i_l << " " << i_r << " ]" << std::endl;

        float ir_c=(float)i_c/n_c;
        float ir_l=(float)i_l/n_l;
        float ir_r=(float)i_r/n_r;

        bool flag_c=n_c>=line_back_project_thres && ir_c>=line_inlier_ratio_thres;
        bool flag_l=n_l>=env_back_project_thres && ir_l>=env_inlier_ratio_thres;
        bool flag_r=n_r>=env_back_project_thres && ir_r>=env_inlier_ratio_thres;

        if(!flag_c)
        {
            std::cout<<"Line Failed ~ [ "<<n_c<<" "<<ir_c<<" ]"<<std::endl;
            continue;
        }
        if(!flag_l && !flag_r)
        {
            std::cout<<"Envelop Failed ~ [ "<<n_l<<" "<<n_r<<std::setprecision(5)<<ir_l<<" "<<ir_r<<" ]"<<std::endl;
            continue;
        }

        PCM::CloudXYZPtr cloud_line_env_i(new pcl::PointCloud<pcl::PointXYZ>);
        float left_env_density = PlanarDensity(cloud_left_env_sac_region_i);
        float right_env_density = PlanarDensity(cloud_right_env_sac_region_i);
//        std::cout << " Planar Density ~ [ " << std::setprecision(5) << left_env_density << " " << right_env_density << " ]" << std::endl;

        if((!flag_l && flag_r) || (left_env_density < left_right_density_ratio_thres * right_env_density))
        {
            std::cout << "Using Right Envelop." << std::endl;
//            PCM::CloudXYZPtr cloud_line_project_right_env = Project2Planar(right_env_coeff, cloud_line_search_region_i);
//            cloud_line_env_i = Search2(cloud_right_env_sac_region_i, cloud_line_project_right_env);

            cloud_line_env_i=LinePlanar3(cloud_line_search_region_i,cloud_right_env_sac_region_i,right_env_coeff);
        }
        else if(flag_l && !flag_r || (right_env_density < left_right_density_ratio_thres * left_env_density))
        {
            std::cout << "Using Left Envelop." << std::endl;
//            PCM::CloudXYZPtr cloud_line_project_left_env = Project2Planar(left_env_coeff, cloud_line_search_region_i);
//            cloud_line_env_i = Search2(cloud_left_env_sac_region_i, cloud_line_project_left_env);

            cloud_line_env_i=LinePlanar3(cloud_line_search_region_i,cloud_left_env_sac_region_i,left_env_coeff);
        }
        else
            continue;

        if(cloud_line_env_i->size()<5)
        {
            std::cout<<"Line Search Too Less ~ "<<cloud_line_env_i->size()<<std::endl;
            continue;
        }

        PCM::CloudXYZPtr cloud_line_env_sac_i=InterpolateLine(cloud_line_env_i);

        cv::Mat img_line;
        img_lsr.copyTo(img_line);
        cv::line(img_line, line_i.s, line_i.e, cv::Scalar(0, 0, 255));
        cv::line(img_line, left_line_i.s, left_line_i.e, cv::Scalar(0, 255, 0));
        cv::line(img_line, right_line_i.s, right_line_i.e, cv::Scalar(255, 0, 0));


        std::vector<cv::Point2f> project_line_search_region_i=Cloud2Pixel(cloud_line_search_region_i,K);
        cv::Mat img_project_line;
        img_lsr.copyTo(img_project_line);
        for(const cv::Point2f &pt_2d : project_line_search_region_i)
            cv::circle(img_project_line,pt_2d,1,cv::Scalar(0,0,255));


        std::vector<cv::Point2f> project_left_env_search_region_i=Cloud2Pixel(cloud_left_env_search_region_i,K);
        cv::Mat img_project_left_env;
        img_lsr.copyTo(img_project_left_env);
        for(const cv::Point2f &pt_2d : project_left_env_search_region_i)
            cv::circle(img_project_left_env,pt_2d,1,cv::Scalar(0,255,0));


        std::vector<cv::Point2f> project_right_env_search_region_i=Cloud2Pixel(cloud_right_env_search_region_i,K);
        cv::Mat img_project_right_env;
        img_lsr.copyTo(img_project_right_env);
        for(const cv::Point2f &pt_2d : project_right_env_search_region_i)
            cv::circle(img_project_right_env,pt_2d,1,cv::Scalar(255,0,0));


        std::vector<cv::Point2f> project_line_sac_region_i=Cloud2Pixel(cloud_line_sac_region_i,K);
        cv::Mat img_sac_line;
        img_lsr.copyTo(img_sac_line);
        for(const cv::Point2f &pt_2d : project_line_sac_region_i)
            cv::circle(img_sac_line,pt_2d,1,cv::Scalar(0,0,255));


        std::vector<cv::Point2f> project_left_env_sac_region_i=Cloud2Pixel(cloud_left_env_sac_region_i,K);
        cv::Mat img_sac_left_env;
        img_lsr.copyTo(img_sac_left_env);
        for(const cv::Point2f &pt_2d : project_left_env_sac_region_i)
            cv::circle(img_sac_left_env,pt_2d,1,cv::Scalar(0,255,0));


        std::vector<cv::Point2f> project_right_env_sac_region_i=Cloud2Pixel(cloud_right_env_sac_region_i,K);
        cv::Mat img_sac_right_env;
        img_lsr.copyTo(img_sac_right_env);
        for(const cv::Point2f &pt_2d : project_right_env_sac_region_i)
            cv::circle(img_sac_right_env,pt_2d,1,cv::Scalar(255,0,0));


#if WRITERESULT
        std::string tmp_dir=dir+"/"+std::to_string(i)+"_";
//        Util::DirBuild(tmp_dir);

        cv::imwrite(tmp_dir+"img_line.png",img_line);
//        cv::imwrite(tmp_dir+"img_project_line.png",img_project_line);
//        cv::imwrite(tmp_dir+"img_project_left_env.png",img_project_left_env);
//        cv::imwrite(tmp_dir+"img_project_right_env.png",img_project_right_env);
//        cv::imwrite(tmp_dir+"img_sac_line.png",img_sac_line);
//        cv::imwrite(tmp_dir+"img_sac_left_env.png",img_sac_left_env);
//        cv::imwrite(tmp_dir+"img_sac_right_env.png",img_sac_right_env);


//        if(!cloud_line_search_region_i->empty())
//        {
//            pcl::io::savePCDFileBinary(tmp_dir+"line_search_region.pcd",*cloud_line_search_region_i);
//        }
//        if(!cloud_left_env_search_region_i->empty())
//        {
//            pcl::io::savePCDFileBinary(tmp_dir+"left_env_search_region.pcd",*cloud_left_env_search_region_i);
//        }
//        if(!cloud_right_env_search_region_i->empty())
//        {
//            pcl::io::savePCDFileBinary(tmp_dir+"right_env_search_region.pcd",*cloud_right_env_search_region_i);
//        }
//        if(!cloud_line_sac_region_i->empty())
//        {
//            pcl::io::savePCDFileBinary(tmp_dir+"line_sac_region.pcd",*cloud_line_sac_region_i);
//        }
//        if(!cloud_left_env_sac_region_i->empty())
//        {
//            pcl::io::savePCDFileBinary(tmp_dir+"left_env_sac_region.pcd",*cloud_left_env_sac_region_i);
//        }
//        if(!cloud_right_env_sac_region_i->empty())
//        {
//            pcl::io::savePCDFileBinary(tmp_dir+"right_env_sac_region.pcd",*cloud_right_env_sac_region_i);
//        }

//        PCM::WriteLas(cloud_line_region_i,cloud_left_env_region_i,cloud_right_env_region_i,tmp_dir+"project.las");
//        PCM::WriteLas(cloud_line_search_region_i,cloud_left_env_search_region_i,cloud_right_env_search_region_i,tmp_dir+"search.las");
        PCM::WriteLas(cloud_line_sac_region_i,cloud_left_env_sac_region_i,cloud_right_env_sac_region_i,tmp_dir+"sac.las");
        PCM::WriteLas(cloud_line_env_i,tmp_dir+"search.las");
        PCM::WriteLas(cloud_line_env_sac_i,tmp_dir+"filter.las");
#else
        cv::imshow("img_line", img_line);
        cv::imshow("img_project_line", img_project_line);
        cv::imshow("img_project_left_env", img_project_left_env);
        cv::imshow("img_project_right_env", img_project_right_env);
        cv::imshow("img_sac_line", img_sac_line);
        cv::imshow("img_sac_left_env", img_sac_left_env);
        cv::imshow("img_sac_right_env", img_sac_right_env);

        if (!cloud_line_sac_region_i->empty())
        {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_line_region_color_i(cloud_line_sac_region_i, 250, 0, 0);
            visualizer->updatePointCloud(cloud_line_sac_region_i, cloud_line_region_color_i, "line_region");
            visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "line_region");
        }
//            if (!cloud_left_line_region_i->empty())
//            {
//                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_left_line_region_color_i(
//                        cloud_left_line_region_i, 0, 250, 0);
//                visualizer->updatePointCloud(cloud_left_line_region_i, cloud_left_line_region_color_i, "left_line_region");
//                visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"left_line_region");
//            }

//            if (!cloud_right_line_region_i->empty())
//            {
//                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_right_line_region_color_i(
//                        cloud_right_line_region_i, 0, 0, 250);
//                visualizer->updatePointCloud(cloud_right_line_region_i, cloud_right_line_region_color_i, "right_line_region");
//                visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"right_line_region");
//            }

//            if (!cloud_env_region_i->empty())
//            {
//                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_env_region_color_i(cloud_env_region_i,
//                                                                                                          120, 120, 120);
//                visualizer->updatePointCloud(cloud_env_region_i, cloud_env_region_color_i, "env_region");
//            }

        if (!cloud_left_env_sac_region_i->empty())
        {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_left_env_region_color_i(cloud_left_env_sac_region_i, 0, 250, 0);
            visualizer->updatePointCloud(cloud_left_env_sac_region_i, cloud_left_env_region_color_i, "left_env_region");
        }

        if (!cloud_right_env_sac_region_i->empty())
        {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_right_env_region_color_i(cloud_right_env_sac_region_i, 0, 0, 250);
            visualizer->updatePointCloud(cloud_right_env_sac_region_i, cloud_right_env_region_color_i, "right_env_region");
        }


        cv::waitKey(0);
#endif

    }

}