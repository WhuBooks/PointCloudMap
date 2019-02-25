//
// Created by whubooks on 18-4-5.
// paper : A Fast Edge Extraction Method for Mobile Lidar Points
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>
#include <list>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Util.h>
#include <CloudUtil.h>

struct Point
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3f pos;
    float intensity;
    Eigen::Vector3f grad;
    float lambda;
};


float StdDev(const std::vector<float> &vector)
{
    int size=vector.size();
    float ave=std::accumulate(vector.begin(),vector.end(),0.0f)/size;

    float square_sum=0.0f;
    for(const float &tmp : vector)
        square_sum+=(tmp-ave)*(tmp-ave);

    float std_dev=std::sqrt(square_sum/size);
    return std_dev;
}

float StdDevEig(float val1,float val2,float val3)
{
    std::vector<float> vec{val1, val2, val3};
    return StdDev(vec);
}

/// judge two vector's include angle less than 0.1*pi
bool SimilarVec(const Eigen::Vector3f &vec1,const Eigen::Vector3f &vec2)
{
    const float threshold=0.2f*M_PI;

    float product=vec1.transpose()*vec2;
    float angle=std::acos(product/(vec1.norm()*vec2.norm()));
    return angle<threshold;
}

void WriteEigCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,const std::string &file)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_nor(new pcl::PointCloud<pcl::PointXYZI>);

    float min_r=10000000000,max_r=-1000000000000;
    for(const pcl::PointXYZI &pt : cloud->points)
    {
        float r=pt.intensity;
        min_r=(min_r<r)?min_r:r;
        max_r=(max_r>r)?max_r:r;
    }

    for(const pcl::PointXYZI &pt : cloud->points)
    {
        pcl::PointXYZI p;
        p.x=pt.x;
        p.y=pt.y;
        p.z=pt.z;
        p.intensity=(pt.intensity-min_r)/(max_r-min_r);
        cloud_nor->push_back(p);
    }

    pcl::io::savePCDFileASCII(file,*cloud_nor);

}

void WritePointCloud(const std::vector<Point> &pts,const std::string &filename)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for(const Point &pt : pts)
    {
        pcl::PointXYZI p;
        p.x=pt.pos(0);
        p.y=pt.pos(1);
        p.z=pt.pos(2);
        p.intensity=pt.intensity;
        cloud->push_back(p);
    }
    pcl::io::savePCDFileASCII(filename,*cloud);
}


int main(int argc,char **argv)
{
    std::string pcdfile = (argc > 1) ? std::string(argv[1]) : "50_mls.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile, *cloud);
    std::cout << "Origin Point Cloud Size ~ " << cloud->size() << std::endl;

    PCM::ConditionFilter(cloud);

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    flann->setInputCloud(cloud);
    const int K = 8;
    const float radius = 0.1f;

    /// compute edge index
    std::vector<float> vedgeindex;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_edge_index(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < cloud->size(); i++)
    {
        pcl::PointXYZI pt = cloud->points[i];

        std::vector<int> vindices;
        std::vector<float> vdistance;
        flann->radiusSearch(pt, radius, vindices, vdistance);

        if (vindices.size() < 5)
            continue;

        Eigen::Vector3f pos_knn(0.0f, 0.0f, 0.0f);
        for (const int &index : vindices)
        {
            if (i == index)
                continue;
            pcl::PointXYZI pt_knn = cloud->points[index];
            pos_knn += Eigen::Vector3f(pt_knn.x, pt_knn.y, pt_knn.z);
        }
        pos_knn /= (vindices.size() - 1);

        Eigen::Vector3f pos_pt(pt.x, pt.y, pt.z);
        float edgeindex_pt = (pos_pt - pos_knn).norm() / radius;

        vedgeindex.push_back(edgeindex_pt);
        cloud_tmp->push_back(pt);

        pcl::PointXYZI p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = edgeindex_pt;
        cloud_edge_index->push_back(p);
    }
    std::swap(cloud, cloud_tmp);
    std::cout << "Edge Index Cloud Size ~ " << cloud->size() << std::endl;
    std::cout << "Std of Edge-Index ~ " << StdDev(vedgeindex) << std::endl;

    std::string dir = "Edge_" + Util::GetNameFromTime();
    Util::DirBuild(dir);
    std::string edge_file = dir + "/" + Util::SplitNameWithoutExt(pcdfile) + "_edge_index.pcd";
    pcl::io::savePCDFileBinary(edge_file, *cloud_edge_index);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_candidate(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<Point> pts_candidate;
    flann->setInputCloud(cloud);
    /// compute gradient and its eigen value, then select candidate edge point
    const float threshold = 100;
    for (std::size_t i = 0; i < cloud->size(); i++)
    {
        pcl::PointXYZI pt = cloud->points[i];
        float edgeindex_pt = vedgeindex[i];

        std::vector<int> vindices;
        std::vector<float> vdistance;
        //flann->nearestKSearch(pt, K+1, vindices, vdistance);
        flann->radiusSearch(pt,radius,vindices,vdistance);
        float dis_std = StdDev(vdistance);

        std::vector<float> vgradx, vgrady, vgradz;
        std::vector<float> vfactor;
        for (int j = 0; j < vindices.size(); j++)
        {
            int index = vindices[j];
            float dis_knn = vdistance[j];

            if (i == index)
                continue;

            pcl::PointXYZI pt_knn = cloud->points[index];
            float edgeindex_knn = vedgeindex[index];

            float grad_x_tmp = (edgeindex_pt - edgeindex_knn) * (pt.x - pt_knn.x) / dis_knn;
            float grad_y_tmp = (edgeindex_pt - edgeindex_knn) * (pt.y - pt_knn.y) / dis_knn;
            float grad_z_tmp = (edgeindex_pt - edgeindex_knn) * (pt.z - pt_knn.z) / dis_knn;

            float factor = std::exp(-dis_knn * dis_knn / (2 * dis_std * dis_std)) / (dis_std * std::sqrt(2 * M_PI));
            vfactor.push_back(factor);

            vgradx.push_back(grad_x_tmp);
            vgrady.push_back(grad_y_tmp);
            vgradz.push_back(grad_z_tmp);
        }

        float grad_x_pt = -100000000000, grad_y_pt = -100000000000, grad_z_pt = -10000000000;
        float grad_x_factor = 1.0f, grad_y_factor = 1.0f, grad_z_factor = 1.0f;
        for (int jjj = 0; jjj < vgradx.size(); jjj++)
        {
            if (grad_x_pt < vgradx[jjj])
            {
                grad_x_pt = vgradx[jjj];
                grad_x_factor = vfactor[jjj];
            }

            if (grad_y_pt < vgrady[jjj])
            {
                grad_y_pt = vgrady[jjj];
                grad_y_factor = vfactor[jjj];
            }

            if (grad_z_pt < vgradz[jjj])
            {
                grad_z_pt = vgradz[jjj];
                grad_z_factor = vfactor[jjj];
            }
        }

//        Eigen::Vector3f grad_vector(grad_x_factor*grad_x_pt,grad_y_factor*grad_y_pt,grad_z_factor*grad_z_pt);
        Eigen::Vector3f grad_vector(grad_x_pt, grad_y_pt, grad_z_pt);
        Eigen::Matrix3f grad_matrix = grad_vector * grad_vector.transpose();

//        Eigen::EigenSolver<Eigen::Matrix3f> solver(grad_matrix);
//        Eigen::Matrix3f D = solver.pseudoEigenvalueMatrix();
//        Eigen::Matrix3f V = solver.pseudoEigenvectors();
//
//        float lambda1 = D(0, 0);
//        float lambda2 = D(1, 1);
//        float lambda3 = D(2, 2);

//        if(lambda1==0)
//            lambda1+=1e-10;
//        if(lambda2==0)
//            lambda2+=1e-10;
//        if(lambda3==0)
//            lambda3+=1e-10;

        float trace = grad_matrix.trace();
        float determidant = grad_matrix.determinant();

//        float lambda_sum = lambda1 + lambda2 + lambda3;
//        float lambda_product = std::abs(lambda1 * lambda2 * lambda3);

        if (determidant != 0)
        {
//            float val=(lambda_sum*lambda_sum*lambda_sum)/lambda_product;
            float val = (trace * trace * trace) / determidant;
            if (val > threshold)
            {
                cloud_candidate->push_back(pt);

                Point p;
                p.pos = Eigen::Vector3f(pt.x, pt.y, pt.z);
                p.grad = Eigen::Vector3f(grad_x_pt, grad_y_pt, grad_z_pt);
                p.grad.normalize();
                p.lambda = edgeindex_pt;
                p.intensity = pt.intensity;
                pts_candidate.push_back(p);
            }
        }
    }
    std::cout << "Candidate Edge Point Size ~ " << cloud_candidate->size() << std::endl;
    std::string candidate_file = dir + "/" + Util::SplitNameWithoutExt(pcdfile) + "_edge_candidate.pcd";
    pcl::io::savePCDFileBinary(candidate_file, *cloud_candidate);

    // return 1;

    /// non-maximum suppression
    std::vector<Point> pts_nms;
    for (int i = 0; i < pts_candidate.size(); i++)
    {
        Point pt = pts_candidate[i];

        /// search nearest point along the gradient direction
        Point pt_grad1, pt_grad2;
        float min_dis1 = 100000000, min_dis2 = 100000000;
        for (int j = 0; j < pts_candidate.size(); j++)
        {
            if (i == j)
                continue;

            Point pt_j = pts_candidate[j];
            Eigen::Vector3f vec_ij = pt_j.pos - pt.pos;
            float dis_ij = vec_ij.norm();
            if (dis_ij > 0.5)
                continue;

            if (SimilarVec(vec_ij, pt.grad))
            {
                if (dis_ij < min_dis1)
                {
                    min_dis1 = dis_ij;
                    pt_grad1 = pt_j;
                }
            }

            if (SimilarVec(vec_ij, -pt.grad))
            {
                if (dis_ij < min_dis2)
                {
                    min_dis2 = dis_ij;
                    pt_grad2 = pt_j;
                }
            }
        }

        /// if find point along gradient
        if (min_dis1 < 0.5 && min_dis2 < 0.5)
        {
            if (pt.lambda > pt_grad1.lambda && pt.lambda > pt_grad2.lambda)
            {
                pts_nms.push_back(pt);
            }
        }
    }
    std::cout << "NMS Result Size ~ " << pts_nms.size() << std::endl;
    if (pts_nms.empty())
        return 2;

    /// save nms result
    std::string nmsfile = dir + "/" + Util::SplitNameWithoutExt(pcdfile) + "_nms.pcd";
    WritePointCloud(pts_nms, nmsfile);
    return 1;
}