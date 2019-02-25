//
// Created by whubooks on 18-3-15.
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sophus/se3.h>

#include <Util.h>

struct Point
{
    int id;
    double x;
    double y;
    double z;
    double r;
};
typedef std::vector<Point> DataFrame;

DataFrame Load_cpp(std::string filename)
{
    DataFrame ptVec;
    std::ifstream ifs(filename, std::ios::in | std::ios::binary);
    if (!ifs.is_open())
        return ptVec;

    int id = 0;
    while (ifs.good() && !ifs.eof())
    {
        Point pt;
        pt.id = id++;
        float x=0.0f,y=0.0f,z=0.0f,r=0.0f;
        ifs.read(reinterpret_cast<char *>(&x), sizeof(float));
        ifs.read(reinterpret_cast<char *>(&y), sizeof(float));
        ifs.read(reinterpret_cast<char *>(&z), sizeof(float));
        ifs.read(reinterpret_cast<char *>(&r), sizeof(float));

        pt.x=(double)x;
        pt.y=(double)y;
        pt.z=(double)z;
        pt.r=(double)r;
        ptVec.push_back(pt);
    }
    ifs.close();

    if(!ptVec.empty())
        ptVec.erase(ptVec.end()-1);

    return ptVec;
}

typedef Eigen::Matrix<double,3,4> Matrix34d;
typedef std::vector<Matrix34d,Eigen::aligned_allocator<Matrix34d>> Matrix34dVec;

Matrix34d LoadCalibFile(std::string filename)
{
    Matrix34d transform;

    std::ifstream ifs(filename,std::ios::in);
    if(!ifs.is_open())
        return transform;

    while(ifs.good()&&!ifs.eof())
    {
        std::string str;
        double x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12;
        ifs >> str >> x1 >> x2 >> x3 >> x4 >> x5 >> x6 >> x7 >> x8 >> x9 >> x10 >> x11 >> x12;

        if(str=="Tr:")
        {
            transform << x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12;
            break;
        }
    }
    ifs.close();

    return transform;
}

Matrix34dVec LoadPoseFile(std::string filename)
{
    Matrix34dVec vec;

    std::ifstream ifs(filename,std::ios::in);
    if(!ifs.is_open())
        return vec;

    while(ifs.good()&&!ifs.eof())
    {
        double x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12;
        ifs >> x1 >> x2 >> x3 >> x4 >> x5 >> x6 >> x7 >> x8 >> x9 >> x10 >> x11 >> x12;

        Matrix34d pose;
        pose<<x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12;

        vec.push_back(pose);
    }

    if(!vec.empty())
        vec.erase(vec.end()-1);

    return vec;

};

std::vector<double> LoadTimeFile(std::string filename)
{
    std::vector<double> timeVec;

    std::ifstream ifs(filename,std::ios::in);
    if(!ifs.is_open())
        return timeVec;

    while(ifs.good()&&!ifs.eof())
    {
        double time=0.0;
        ifs>>time;
        timeVec.push_back(time);
    }

    if(!timeVec.empty())
        timeVec.erase(timeVec.end()-1);

    return timeVec;
}

int ConvertName(const std::string &filename)
{
    std::string tmp=Util::SplitNameWithoutExt(filename);
    std::stringstream ss;
    ss<<tmp;

    int val=0;
    ss>>val;
    return val;
}

DataFrame FilterFrame(const DataFrame &source)
{
    DataFrame target;

    for(const Point &pt : source)
    {
        /// ignore point distance more than 40
        double dist_thres=40.0;
        if(pt.x*pt.x+pt.y*pt.y+pt.z*pt.z>dist_thres*dist_thres)
            continue;

        /// ignore point below the ground
        if(pt.z<-1.9)
            continue;

        /// ignore point y large than 15 or less than -15
        if(pt.y<-15 || pt.y>15)
            continue;

        target.push_back(pt);
    }

    return target;

}


pcl::PointCloud<pcl::PointXYZINormal>::Ptr Filter(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::KdTreeFLANN<pcl::PointXYZINormal> kdTreeFLANN;
    kdTreeFLANN.setInputCloud(cloud);

    /// filter repeat point by it's distance to local origin
    double radius=0.05;
    for(const pcl::PointXYZINormal &pt : cloud->points)
    {
        std::vector<int> vindices;
        std::vector<float> vdistance;

        kdTreeFLANN.radiusSearch(pt,radius,vindices,vdistance);

        bool flag=true;
        for(const int &knn_index : vindices)
        {
            flag=flag&&(cloud->points[knn_index].curvature>=pt.curvature);
        }

        if(flag)
            cloud_filter->push_back(pt);
    }
    return cloud_filter;
}

int main(int argc,char **argv)
{
    std::string velo_dir="/home/whubooks/kitti_odometry/data_odometry_velodyne/sequences/";
    std::string gray_dir="/home/whubooks/kitti_odometry/data_odometry_gray/sequences/";
    std::string calib_dir="/home/whubooks/kitti_odometry/data_odometry_calib/sequences/";
    std::string pose_dir="/home/whubooks/kitti_odometry/data_odometry_poses/poses/";

    std::string calibfile = calib_dir+"00/calib.txt";
    std::string posefile = pose_dir+"00.txt";
    std::string timefile = calib_dir+"00/times.txt";

    Matrix34d transform = LoadCalibFile(calibfile);
    std::cout << "T ~ \n" << transform << std::endl;

    Eigen::Matrix3d rotation = transform.block(0, 0, 3, 3);
    std::cout << "R ~ \n" << rotation << std::endl;

    Eigen::Vector3d translation = transform.block(0, 3, 3, 1);
    std::cout << "t ~ \n" << translation << std::endl;

    Sophus::SE3 t_laser_camera(rotation, translation);

    Matrix34dVec poseVec = LoadPoseFile(posefile);

    std::vector<double> timeVec = LoadTimeFile(timefile);

    std::string dir = velo_dir+"00/velodyne";
    std::vector<std::string> vfile = Util::GetFiles(dir);

    assert(vfile.size() == poseVec.size() && vfile.size() == timeVec.size());

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    int select_index=50;
    if(argc>1)
    {
        std::stringstream ss;
        ss<<std::string(argv[1]);
        ss>>select_index;
    }
    std::cout<<"Select Index ~ "<<select_index<<std::endl;

    Matrix34d pose_select=poseVec[select_index];
    Eigen::Matrix3d rotSelect=pose_select.block(0,0,3,3);
    Eigen::Vector3d traSelect=pose_select.block(0,3,3,1);
    Sophus::SE3 t_select_first(rotSelect,traSelect);

    for (const std::string &filename : vfile)
    {
        int index = ConvertName(filename);
//        if(index<std::max(select_index-10,0))
//            continue;
//
//        if(index>select_index+20)
//            break;

        DataFrame frame = Load_cpp(filename);

        ///get gps time and transform to first camera coordinate system
        Matrix34d pose = poseVec[index];
        Eigen::Matrix3d rotFirst = pose.block(0, 0, 3, 3);
        Eigen::Vector3d traFirst = pose.block(0, 3, 3, 1);
        Sophus::SE3 t_now_first(rotFirst, traFirst);


        for (const Point &pt : frame)
        {
            /// distance to local origin
            double dis_to_center=std::sqrt(pt.x*pt.x+pt.y*pt.y+pt.z*pt.z);

            ///transform point cloud to left camera coordinate system
            Eigen::Vector3d ptPos(pt.x, pt.y, pt.z);
            Eigen::Vector3d ptPosTrans = t_laser_camera * ptPos;

            /// transform point cloud to first camera coordinate system
            Eigen::Vector3d ptPosFirst = t_now_first * ptPosTrans;

            /// transform point cloud to select camera coordinate system
            Eigen::Vector3d ptPosSelect=t_select_first.inverse()*ptPosFirst;

            double x = ptPosSelect(0);
            double y = ptPosSelect(1);
            double z = ptPosSelect(2);

            /// remove repeat point within threshold 0.5
            pcl::PointXYZINormal pointXYZID;
            pointXYZID.x=(float)x;
            pointXYZID.y=(float)y;
            pointXYZID.z=(float)z;
            pointXYZID.intensity=(float)pt.r;
            pointXYZID.curvature=(float)dis_to_center;

            cloud->push_back(pointXYZID);
        }

        if(index%100==0&&index!=0)
        {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_filter=Filter(cloud);
            std::cout<<"Cloud Size ~ "<<cloud->points.size()<<std::endl;
            std::cout<<"Cloud Filter Size ~ "<<cloud_filter->points.size()<<std::endl;
            cloud->swap(*cloud_filter);
            std::cout<<"*************"<<std::endl<<std::endl;
        }

    }

//    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZINormal>);
//    pcl::KdTreeFLANN<pcl::PointXYZINormal> kdTreeFLANN;
//    kdTreeFLANN.setInputCloud(cloud);
//
//    /// filter repeat point by it's distance to local origin
//    double radius=0.05;
//    for(const pcl::PointXYZINormal &pt : cloud->points)
//    {
//        std::vector<int> vindices;
//        std::vector<float> vdistance;
//
//        kdTreeFLANN.radiusSearch(pt,radius,vindices,vdistance);
//
//        bool flag=true;
//        for(const int &knn_index : vindices)
//        {
//            flag=flag||(cloud->points[knn_index].curvature>=pt.curvature);
//        }
//
//        if(flag)
//            cloud_filter->push_back(pt);
//
//    }


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_io(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter_io(new pcl::PointCloud<pcl::PointXYZI>);

    for(const pcl::PointXYZINormal &pt : cloud->points)
    {
        pcl::PointXYZI ptTmp;
        ptTmp.x=pt.x;
        ptTmp.y=pt.y;
        ptTmp.z=pt.z;
        ptTmp.intensity=pt.intensity;
        cloud_io->push_back(ptTmp);
    }

//    for(const pcl::PointXYZINormal &pt : cloud_filter->points)
//    {
//        pcl::PointXYZI ptTmp;
//        ptTmp.x=pt.x;
//        ptTmp.y=pt.y;
//        ptTmp.z=pt.z;
//        ptTmp.intensity=pt.intensity;
//        cloud_filter_io->push_back(ptTmp);
//    }
//    std::cout<<"Cloud Origin ~ "<<cloud_io->size()<<std::endl;
//    std::cout<<"Cloud Filter ~ "<<cloud_filter_io->size()<<std::endl;

    std::string pcdfilename=std::to_string(select_index)+".pcd";
    pcl::io::savePCDFileASCII(pcdfilename, *cloud_io);

//    std::string pcdfilterfile=std::to_string(select_index)+"_filter.pcd";
//    pcl::io::savePCDFile(pcdfilterfile,*cloud_filter_io);

    return 1;

}