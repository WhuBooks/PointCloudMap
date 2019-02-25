//
// Created by whubooks on 18-3-11.
//
//
/// convert origin point cloud to depth image without limiting camera's view angle


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
#include <sophus/se3.h>
#include <opencv2/opencv.hpp>

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

int main()
{
    std::string velo_dir="/home/whubooks/kitti_odometry/data_odometry_velodyne/sequences/";
    std::string gray_dir="/home/whubooks/kitti_odometry/data_odometry_gray/sequences/";
    std::string calib_dir="/home/whubooks/kitti_odometry/data_odometry_calib/sequences/";

    std::string velo_file=velo_dir+"00/velodyne/000000.bin";
    std::string gray_file=gray_dir+"00/image_0/000000.png";
    std::string calib_file=calib_dir+"00/calib.txt";


    Matrix34d transform = LoadCalibFile(calib_file);
    std::cout << "T ~ \n" << transform << std::endl;

    Eigen::Matrix3d rotation = transform.block(0, 0, 3, 3);
    std::cout << "R ~ \n" << rotation << std::endl;

    Eigen::Vector3d translation = transform.block(0, 3, 3, 1);
    std::cout << "t ~ \n" << translation << std::endl;

    Sophus::SE3 t_laser_camera(rotation, translation);

    std::vector<Point> vpt=Load_cpp(velo_file);
    std::vector<Point> vpt_camera;
    for(const Point &pt : vpt)
    {
        Eigen::Vector3d pos(pt.x,pt.y,pt.z);
        Eigen::Vector3d trans_pos=t_laser_camera*pos;
        Point pt_camera;
        pt_camera.x=trans_pos(0);
        pt_camera.y=trans_pos(1);
        pt_camera.z=trans_pos(2);
        pt_camera.r=pt.r;

        if(pt_camera.z<0)
            continue;

        vpt_camera.push_back(pt_camera);
    }

    double xmin=100000,ymin=1000000,xmax=-100000,ymax=-100000;
    for(const Point &pt:vpt_camera)
    {
        xmin=(xmin<pt.x)?xmin:pt.x;
        ymin=(ymin<pt.y)?ymin:pt.y;
        xmax=(xmax>pt.x)?xmax:pt.x;
        ymax=(ymax>pt.y)?ymax:pt.y;
    }
    std::cout<<"Point Cloud Range ~ ["<<xmin<<"\t"<<ymin<<"\t"<<xmax<<"\t"<<ymax<<"]"<<std::endl;

    double res=0.03;
    int rows=std::ceil((ymax-ymin)/res);
    int cols=std::ceil((xmax-xmin)/res);
    std::vector<double> vdepth(rows*cols,-1.0);

    double zmin=100000,zmax=-10000;
    for(const Point &pt : vpt_camera)
    {
        if(pt.z<0)
            continue;

        int pt_row=std::floor((ymax-pt.y)/res);
        int pt_col=std::floor((xmax-pt.x)/res);

        int index=pt_row*cols+pt_col;
        if(vdepth[index]==-1 || vdepth[index]>pt.z)
        {
            vdepth[index]=pt.z;
            zmin=(zmin<pt.z)?zmin:pt.z;
            zmax=(zmax>pt.z)?zmax:pt.z;
        }
    }

    for(double &val : vdepth)
    {
        val=(val-zmin)/(zmax-zmin);
    }

    cv::Mat img_depth(rows,cols,CV_64FC1,vdepth.data());
    cv::imshow("img_depth",img_depth);


    cv::Mat img_diff(rows-2,cols-2,CV_64FC1);
    double mindiff=10000,maxdiff=-10000;
    for(int i=1;i<rows-1;i++)
    {
        double *prev_row=img_depth.ptr<double>(i-1);
        double *curr_row=img_depth.ptr<double>(i);
        double *next_row=img_depth.ptr<double>(i+1);

        double *diff_row=img_diff.ptr<double>(i-1);

        for(int j=1;j<cols-1;j++)
        {
            double upper=prev_row[j];
            double left=curr_row[j-1];
            double right=curr_row[j+1];
            double bottom=next_row[j];

            double val=curr_row[j];
            double diff_val=0.0;
            diff_val+=std::max(0.0,upper-val);
            diff_val+=std::max(0.0,bottom-val);
            diff_val+=std::max(0.0,left-val);
            diff_val+=std::max(0.0,right-val);
            diff_val=std::exp(-100*diff_val);

            mindiff=mindiff<diff_val?mindiff:diff_val;
            maxdiff=maxdiff>diff_val?maxdiff:diff_val;

            diff_row[j-1]=diff_val;
        }
    }
    cv::imshow("img_diff",img_diff);


    cv::Mat img_gray(img_diff.rows,img_diff.cols,CV_8UC1);
    for(int i=0;i<img_diff.rows;i++)
    {
        double * diff_row=img_diff.ptr<double>(i);
        uchar * gray_row=img_gray.ptr<uchar>(i);
        for(int j=0;j<img_diff.cols;j++)
        {
            double diff_val=diff_row[j];
            uchar gray_val=(uchar)std::floor(255*(diff_val-mindiff)/(maxdiff-mindiff));
            gray_row[j]=gray_val;
        }
    }
    cv::imshow("img_gray",img_gray);

    cv::Ptr<cv::LineSegmentDetector> lsd_ptr=cv::createLineSegmentDetector();
    std::vector<cv::Vec4i> vlines;
    lsd_ptr->detect(img_gray,vlines);
    cv::Mat img_lsd(img_diff.rows,img_diff.cols,CV_8UC1);
    for(const cv::Vec4i line : vlines)
    {
        cv::Point2i line_s(line[0],line[1]);
        cv::Point2i line_e(line[2],line[3]);
        cv::line(img_lsd,line_s,line_e,cv::Scalar(255));
    }
    cv::imshow("img_lsd",img_lsd);

    cv::waitKey(0);
}