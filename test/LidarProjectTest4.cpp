//
// Created by whubooks on 18-3-23.
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <sophus/se3.h>
#include <opencv2/opencv.hpp>

#include <Util.h>

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

        if(str=="P0:")
        {
            transform << x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12;
            break;
        }
    }
    ifs.close();

    return transform;
}


struct Cell
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector2i pixel;
    float depth;
    Eigen::Vector3f normal;
};

int main()
{
    std::string pcdfile="voxel.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile,*cloud);

    /// estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI,pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.setRadiusSearch(0.1);
    normalEstimation.compute(*normals);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::concatenateFields(*cloud,*normals,*cloud_normal);

    std::string imgfile="/home/whubooks/kitti_odometry/data_odometry_gray/sequences/00/image_0/000020.png";
    std::string calibfile="/home/whubooks/kitti_odometry/data_odometry_calib/sequences/00/calib.txt";

    cv::Mat img_gray_origin=cv::imread(imgfile,CV_LOAD_IMAGE_GRAYSCALE);
    int rows=img_gray_origin.rows;
    int cols=img_gray_origin.cols;

    /// extract subset by camera view
    Matrix34d calib=LoadCalibFile(calibfile);
    Eigen::Matrix3d K=calib.block(0, 0, 3, 3);

    double K_fx=K(0,0),K_dx=K(0,2),K_fy=K(1,1),K_dy=K(1,2);
    double u_limit_min=-K_dx/K_fx;
    double u_limit_max=(img_gray_origin.cols-K_dx)/K_fx;
    double v_limit_min=-K_dy/K_fy;
    double v_limit_max=(img_gray_origin.rows-K_dy)/K_fy;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_camera(new pcl::PointCloud<pcl::PointXYZINormal>);
    for(const pcl::PointXYZINormal & pt : cloud_normal->points)
    {
        if(pt.z==0)
            continue;

        double xz_ratio=pt.x/pt.z;
        if(xz_ratio<=u_limit_min || xz_ratio>=u_limit_max)
            continue;

        double yz_ratio=pt.y/pt.z;
        if(yz_ratio<=v_limit_min || yz_ratio>=v_limit_max)
            continue;

        cloud_camera->points.push_back(pt);
    }
    std::cout<<"Cloud Camera Size ~ "<<cloud_camera->points.size()<<std::endl;


    std::vector<int> vindex;
    std::map<int,int> index_map;
    /// get depth and normal image
    std::vector<float> vdepth(rows * cols, 0.0);
    std::vector<cv::Vec3f> vnormal(rows*cols,cv::Vec3f(0.0f,0.0f,0.0f));

    for (const pcl::PointXYZINormal &pt : cloud_camera->points)
    {
        Eigen::Vector3d xyz(pt.x, pt.y, pt.z);
        Eigen::Vector3d uv_z = K * xyz;

        int pt_row = std::floor(uv_z(1) / pt.z);
        int pt_col = std::floor(uv_z(0) / pt.z);

        int index = pt_row * cols + pt_col;
        if (vdepth[index] == 0.0 || vdepth[index] > pt.z)
        {
            index_map[index]=index_map[index]+1;
            vdepth[index] = pt.z;
            Eigen::Vector3f normal(pt.normal_x,pt.normal_y,pt.normal_z);
            normal.normalize();
            vnormal[index]=cv::Vec3f(normal(0),normal(1),normal(2));
        }
    }
    std::cout<<"Project Index Map Size ~ "<<index_map.size()<<std::endl;
    std::cout<<"Image Size ~ "<<rows*cols<<std::endl;

    /// optimize z to [0,1]
    float zmin = *(std::min_element(vdepth.begin(), vdepth.end()));
    float zmax = *(std::max_element(vdepth.begin(), vdepth.end()));
    for (float &val : vdepth)
        val = (val - zmin) / (zmax - zmin);

    std::ofstream ofs("lidar_project4.txt");
    for(const double &val : vdepth)
    {
        ofs<<val<<std::endl;
    }
    ofs.close();

    std::cout<<"min depth ~ "<<zmin<<std::endl;
    std::cout<<"max depth ~ "<<zmax<<std::endl;


    /// convert normal to rgb for show
    std::vector<cv::Vec3b> vrgb;
    for(const cv::Vec3f &normal : vnormal)
    {
        uchar r=cv::saturate_cast<uchar>(255*normal(0));
        uchar g=cv::saturate_cast<uchar>(255*normal(1));
        uchar b=cv::saturate_cast<uchar>(255*normal(2));
        vrgb.push_back(cv::Vec3b(r,g,b));
    }

    cv::Mat img_depth(rows, cols, CV_32FC1, vdepth.data());
    cv::imshow("img_depth", img_depth);
    cv::Mat img_normal(rows,cols,CV_8UC3,vrgb.data());
    cv::imshow("img_normal",img_normal);


    std::vector<Cell> vcell;
    for(int i=0;i<vdepth.size();i++)
    {
        int u=i%cols;
        int v=std::floor(i/cols);
        Eigen::Vector2i pixel(u,v);
        float depth=vdepth[i];
        cv::Vec3f tmp=vnormal[i];
        if(tmp[0]==0&&tmp[1]==0&&tmp[2]==0)
            continue;
        Eigen::Vector3f normal(tmp[0],tmp[1],tmp[2]);
        Cell cell;
        cell.depth=depth;
        cell.pixel=pixel;
        cell.normal=normal;
        vcell.push_back(cell);
    }
    std::cout<<"Cell Size ~ "<<vcell.size()<<std::endl;



    cv::waitKey(0);

}