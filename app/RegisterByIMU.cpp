//
// Created by books on 18-1-17.
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <KittiData.h>
#include <Util.h>

bool Contain(const std::vector<int> &vec,int index)
{
    for(const int &i : vec)
    {
        if(index==i)
            return true;
    }
    return false;
}

float CalMedian(std::vector<float> vec)
{
    std::sort(vec.begin(),vec.end());
    int num=vec.size();

    float val=0.0f;
    if(num%2==0)
        val= 0.5f*(vec[num/2-1]+vec[num/2]);
    else
        val=vec[(num-1)/2];
    return val;

}

void MergeCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,int start_index,int end_index)
{
    pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZINormal>);
    flann->setInputCloud(cloud);

    /// merge
    const float radius=0.5f;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_merge(new pcl::PointCloud<pcl::PointXYZINormal>);
    for(const pcl::PointXYZINormal &pt : cloud->points)
    {
        /// ignore the first and last five frame
        if((int)pt.normal_x>end_index-5 || (pt.normal_x<start_index+5))
        {
            ///cloud_merge->push_back(pt);
            continue;
        }

        std::vector<int> vindices;
        std::vector<float> vdistance;
        flann->radiusSearch(pt,radius,vindices,vdistance);

        std::vector<int> index_vec;
        std::vector<pcl::PointXYZINormal> pts_knn;
        std::vector<float> dis_knn;
        for(int i=0;i<vindices.size();i++)
        {
            pcl::PointXYZINormal pt_knn=cloud->points[vindices[i]];
            if(Contain(index_vec,(int)pt_knn.normal_x))
                continue;
            index_vec.push_back((int)pt_knn.normal_x);
            dis_knn.push_back(vdistance[i]);
            pts_knn.push_back(pt_knn);
        }
        
        /// if an point scanned less than 3 times, then consider it as a negative point
        if(index_vec.size()<3)
            continue;

        /// if the median distance of this point and other frame's point is too large, thn consider it as a negative point
        if(CalMedian(dis_knn)>0.6f*radius)
            continue;
        
        /// fusion point from different frame by their local distance
        Eigen::Vector3f pos(0.0f,0.0f,0.0f);
        float curvature_sum=0.0f;
        for(const pcl::PointXYZINormal &pt_knn : pts_knn)
        {
            Eigen::Vector3f pos_knn(pt_knn.x,pt_knn.y,pt_knn.z);
            pos_knn*=(pt_knn.curvature*pt_knn.curvature);

            pos+=pos_knn;
            curvature_sum+=(pt_knn.curvature*pt_knn.curvature);
        }
        pos/=curvature_sum;

        pcl::PointXYZINormal pt_merge=pt;
        pt_merge.x=pos(0);
        pt_merge.y=pos(1);
        pt_merge.z=pos(2);

        cloud_merge->push_back(pt_merge);
    }
    std::cout<<"Origin Cloud Size ~ "<<cloud->size()<<std::endl;
    std::swap(cloud,cloud_merge);
    std::cout<<"Merge Cloud Size ~ "<<cloud->size()<<std::endl;

}

int main(int argc,char **argv)
{
//    std::string velo_dir="/home/whubooks/kitti_odometry/data_odometry_velodyne/sequences/";
//    std::string gray_dir="/home/whubooks/kitti_odometry/data_odometry_gray/sequences/";
//    std::string calib_dir="/home/whubooks/kitti_odometry/data_odometry_calib/sequences/";
//    std::string pose_dir="/home/whubooks/kitti_odometry/data_odometry_poses/poses/";

    std::string base_dir="/home/whubooks/kitti_odometry/";
    PCM::KittiData kittiData(base_dir);
    kittiData.SetSubNum(0);

    int num=kittiData.GetSize();

    /// simiply add different frame
    int index=50;
    if(argc>1)
    {
        std::stringstream ss;
        ss<<argv[1];
        ss>>index;
    }
    std::cout<<"Register Index ~ "<<index<<std::endl;

    int start_index=std::max(index-30,0);
    int end_index=std::min(index+70,num);

    Sophus::SE3f se3_index=kittiData.GetPose(index);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_index(new pcl::PointCloud<pcl::PointXYZINormal>);
    for(int i=start_index;i<end_index;i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_i=kittiData.GetCloud(i);
        Sophus::SE3f se3_i=kittiData.GetPose(i);

        for(const pcl::PointXYZI &pt : cloud_i->points)
        {
            Eigen::Vector3f pos_i(pt.x,pt.y,pt.z);
            Eigen::Vector3f pos_0=se3_i*pos_i;
            Eigen::Vector3f pos_index=se3_index.inverse()*pos_0;

            pcl::PointXYZINormal p_index;
            p_index.x=pos_index(0);
            p_index.y=pos_index(1);
            p_index.z=pos_index(2);
            p_index.intensity=pt.intensity;
            p_index.normal_x=i;
            p_index.normal_y=i;
            p_index.normal_z=i;
            p_index.curvature=pos_i.norm();
            cloud_index->push_back(p_index);
        }

        std::cout<<"Current Frame ~ "<<i<<std::endl;
    }

    MergeCloud(cloud_index,start_index,end_index);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZI>);
    for(const pcl::PointXYZINormal &pt : cloud_index->points)
    {
        pcl::PointXYZI p;
        p.x=pt.x;
        p.y=pt.y;
        p.z=pt.z;
        p.intensity=pt.intensity;
        cloud_all->push_back(p);
    }

    std::string pcdfile="Register/"+std::to_string(index)+".pcd";
    pcl::io::savePCDFileBinary(pcdfile,*cloud_all);

    return 1;

}