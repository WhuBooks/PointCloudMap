//
// Created by whubooks on 18-3-12.
//

/// filter origin point cloud with camera's view angle
/// convert filter cloud to depth image

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
#include <pcl/range_image/range_image_planar.h>
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


int main()
{
    std::string pcdfile="voxel.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile,*cloud);

    std::string imgfile="/home/whubooks/kitti_odometry/data_odometry_gray/sequences/00/image_0/000020.png";
    std::string calibfile="/home/whubooks/kitti_odometry/data_odometry_calib/sequences/00/calib.txt";

    cv::Mat img_gray_origin=cv::imread(imgfile,CV_LOAD_IMAGE_GRAYSCALE);
    int rows=img_gray_origin.rows;
    int cols=img_gray_origin.cols;
//    cv::imshow("img_gray_origin",img_gray_origin);
    Matrix34d calib=LoadCalibFile(calibfile);
    Eigen::Matrix3d K=calib.block(0, 0, 3, 3);

    double K_fx=K(0,0),K_dx=K(0,2),K_fy=K(1,1),K_dy=K(1,2);
    double u_limit_min=-K_dx/K_fx;
    double u_limit_max=(img_gray_origin.cols-K_dx)/K_fx;
    double v_limit_min=-K_dy/K_fy;
    double v_limit_max=(img_gray_origin.rows-K_dy)/K_fy;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_camera(new pcl::PointCloud<pcl::PointXYZI>);
    for(const pcl::PointXYZI & pt : cloud->points)
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

    double xmin=100000,ymin=1000000,xmax=-100000,ymax=-100000;
    for(const pcl::PointXYZI &pt : cloud_camera->points)
    {
        xmin=(xmin<pt.x)?xmin:pt.x;
        ymin=(ymin<pt.y)?ymin:pt.y;
        xmax=(xmax>pt.x)?xmax:pt.x;
        ymax=(ymax>pt.y)?ymax:pt.y;
    }

    double zmin=100000,zmax=-10000;
    std::vector<double> vdepth(rows*cols,0.0);
    for(const pcl::PointXYZI &pt : cloud_camera->points)
    {

        Eigen::Vector3d xyz(pt.x,pt.y,pt.z);
        Eigen::Vector3d uv_z=K*xyz;

        int pt_row=std::floor(uv_z(1)/pt.z);
        int pt_col=std::floor(uv_z(0)/pt.z);

        int index=pt_row*img_gray_origin.cols+pt_col;
        if(vdepth[index]==0.0 || vdepth[index]>pt.z)
        {
            vdepth[index]=pt.z;
            zmin=(zmin<pt.z)?zmin:pt.z;
            zmax=(zmax>pt.z)?zmax:pt.z;
        }
    }

    for(double &val : vdepth)
    {
       // val= cv::saturate_cast<uchar>((val-zmin)/(zmax-zmin)*255);
        val=(val-zmin)/(zmax-zmin);
    }

    cv::Mat img_depth(rows,cols,CV_64FC1,vdepth.data());
   // cv::blur(img_depth,img_depth,cv::Size(5,5));
//    cv::Mat kernel=(cv::Mat_<double>(3,3)<<0,-1,0,-1,5,-1,0,-1,0);
//    cv::filter2D(img_depth,img_depth,img_depth.depth(),kernel);
    cv::imshow("img_depth",img_depth);


    std::vector<uchar> vdepth_gray;
    for(const double &val : vdepth)
    {
        uchar tmp=cv::saturate_cast<uchar>(val*255);
        vdepth_gray.push_back(tmp);
    }
    cv::Mat img_depth_gray(rows,cols,CV_8UC1,vdepth_gray.data());
    //cv::blur(img_depth_gray,img_depth_gray,cv::Size(3,3));
    cv::medianBlur(img_depth_gray,img_depth_gray,3);
//    cv::Mat kernel=(cv::Mat_<double>(3,3)<<0,-1,0,-1,5,-1,0,-1,0);
//    cv::filter2D(img_depth,img_depth,img_depth.depth(),kernel);
    cv::Mat img_depth_gray_bilateral(rows,cols,CV_8UC1);
    cv::bilateralFilter(img_depth_gray,img_depth_gray_bilateral,15,25,10);
    cv::imshow("img_depth_gray",img_depth_gray);
    cv::imshow("img_depth_gray_bilateral",img_depth_gray_bilateral);

    cv::Mat img_canny(img_depth_gray.rows,img_depth_gray.cols,CV_8UC1);
    cv::Canny(img_depth_gray,img_canny,50,200);
    cv::imshow("img_canny",img_canny);

    cv::Ptr<cv::LineSegmentDetector> lsd_ptr=cv::createLineSegmentDetector();
    std::vector<cv::Vec4i> vlines;
    lsd_ptr->detect(img_depth_gray,vlines);
    cv::Mat img_lsd(img_depth_gray.rows,img_depth_gray.cols,CV_8UC1);
    for(const cv::Vec4i &line : vlines)
    {
        cv::Point2i line_s(line[0],line[1]);
        cv::Point2i line_e(line[2],line[3]);

        cv::line(img_lsd,line_s,line_e,cv::Scalar(255));
        cv::line(img_gray_origin,line_s,line_e,cv::Scalar(255));
    }
    cv::imshow("img_lsd",img_lsd);
    cv::imshow("img_gray_origin",img_gray_origin);

    cv::Mat img_gray_origin_process(img_gray_origin);
    cv::blur(img_gray_origin_process,img_gray_origin_process,cv::Size(3,3));
    cv::Canny(img_gray_origin_process,img_gray_origin_process,20,200);
    std::vector<cv::Vec4i> vlines_origin;
    lsd_ptr->detect(img_gray_origin_process,vlines_origin);
    cv::Mat img_lsd_origin(rows,cols,CV_8UC1);
    img_lsd_origin.setTo(0);
    for(const cv::Vec4i &line : vlines_origin)
    {
        cv::Point2i line_s(line[0],line[1]);
        cv::Point2i line_e(line[2],line[3]);

        cv::line(img_lsd_origin,line_s,line_e,cv::Scalar(255));
    }
    cv::imshow("img_lsd_origin",img_lsd_origin);


//    cv::Mat img_diff(rows-2,cols-2,CV_64FC1);
//    double mindiff=10000,maxdiff=-10000;
//    for(int i=1;i<rows-1;i++)
//    {
//        double *prev_row=img_depth.ptr<double>(i-1);
//        double *curr_row=img_depth.ptr<double>(i);
//        double *next_row=img_depth.ptr<double>(i+1);
//
//        double *diff_row=img_diff.ptr<double>(i-1);
//
//        for(int j=1;j<cols-1;j++)
//        {
//            double upper=prev_row[j];
//            double left=curr_row[j-1];
//            double right=curr_row[j+1];
//            double bottom=next_row[j];
//
//            double val=curr_row[j];
//            double diff_val=0.0;
//            diff_val+=std::max(0.0,upper-val);
//            diff_val+=std::max(0.0,bottom-val);
//            diff_val+=std::max(0.0,left-val);
//            diff_val+=std::max(0.0,right-val);
//            diff_val=std::exp(-100*diff_val);
//
//            mindiff=mindiff<diff_val?mindiff:diff_val;
//            maxdiff=maxdiff>diff_val?maxdiff:diff_val;
//
//            diff_row[j-1]=diff_val;
//        }
//    }
//    //cv::blur(img_diff,img_diff,cv::Size(3,3));
//    cv::imshow("img_diff",img_diff);
//
//    cv::Mat img_gray(img_diff.rows,img_diff.cols,CV_8UC1);
//    for(int i=0;i<img_diff.rows;i++)
//    {
//        double * diff_row=img_diff.ptr<double>(i);
//        uchar * gray_row=img_gray.ptr<uchar>(i);
//        for(int j=0;j<img_diff.cols;j++)
//        {
//            double diff_val=diff_row[j];
//            uchar gray_val=(uchar)std::floor(255*(diff_val-mindiff)/(maxdiff-mindiff));
//            gray_row[j]=gray_val;
//        }
//    }
//    //cv::blur(img_gray,img_gray,cv::Size(3,3));
//    //cv::GaussianBlur(img_gray,img_gray,cv::Size(3,3),0,0);
//    cv::imshow("img_gray",img_gray);
//
//    cv::Mat img_canny(img_diff.rows,img_diff.cols,CV_8UC1);
//    cv::Canny(img_gray,img_canny,10,200);
//    cv::imshow("img_canny",img_canny);
//
//    cv::Ptr<cv::LineSegmentDetector> lsd_ptr=cv::createLineSegmentDetector();
//    std::vector<cv::Vec4i> vlines;
//    lsd_ptr->detect(img_canny,vlines);
//    cv::Mat img_lsd(img_diff.rows,img_diff.cols,CV_8UC1);
//    for(const cv::Vec4i line : vlines)
//    {
//        cv::Point2i line_s(line[0],line[1]);
//        cv::Point2i line_e(line[2],line[3]);
//        cv::line(img_lsd,line_s,line_e,cv::Scalar(255));
//    }
//    cv::imshow("img_lsd",img_lsd);
//
//    for(const cv::Vec4i &line : vlines)
//    {
//        cv::Point2i line_s(line[0]+1,line[1]+1);
//        cv::Point2i line_e(line[2]+1,line[3]+1);
//        cv::line(img_gray_origin,line_s,line_e,cv::Scalar(255));
//    }
//    cv::imshow("img_gray_origin",img_gray_origin);

//    std::vector<Eigen::Matrix<double,6,1>,Eigen::aligned_allocator<Eigen::Matrix<double,6,1>>> vlines_3d;
//    for(const cv::Vec4i &line : vlines)
//    {
//        int u_s=line[0]+1;
//        int v_s=line[1]+1;
//        double x_s=v_s*res+xmin;
//        double y_s=u_s*res+ymin;
//        double z_s=img_depth.at<double>(u_s,v_s)*(zmax-zmin)+zmin;
//
//
//        int u_e=line[2]+1;
//        int v_e=line[3]+1;
//        double x_e=v_e*res+xmin;
//        double y_e=u_e*res+ymin;
//        double z_e=img_depth.at<double>(u_e,v_e)*(zmax-zmin)+zmin;
//
//        Eigen::Matrix<double,6,1> line_3d;
//        line_3d<<x_s,y_s,z_s,x_e,y_e,z_e;
//        vlines_3d.push_back(line_3d);
//    }
//
//    cv::Mat img_back(img_gray_origin.rows,img_gray_origin.cols,CV_8UC1);
//    std::vector<cv::Vec4i> vline_project;
//    for(const Eigen::Matrix<double,6,1> &line : vlines_3d)
//    {
//        Eigen::Vector3d s=line.block(0,0,3,1);
//        Eigen::Vector3d e=line.block(3,0,3,1);
//
//        Eigen::Vector3d s_uv=K*s;
//        Eigen::Vector3d e_uv=K*e;
//
//        int u_s=(int)(s_uv(0)/s_uv(2));
//        int v_s=(int)(s_uv(1)/s_uv(2));
//        int u_e=(int)(e_uv(0)/e_uv(2));
//        int v_e=(int)(e_uv(0)/e_uv(2));
//
//        std::cout<<"Pixel Line ~ [ "<<u_s<<","<<v_s<<" ]\t[ "<<u_e<<","<<v_e<<" ]"<<std::endl;
//
//        cv::line(img_back,cv::Point2i(u_s,v_s),cv::Point2i(u_e,v_e),cv::Scalar(255));
//    }
//
//    cv::imshow("img_back",img_back);

    cv::waitKey(0);
}