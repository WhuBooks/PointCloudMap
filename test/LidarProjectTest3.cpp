//
// Created by whubooks on 18-3-15.
//

/// convert the point cloud with limiting camera view to depth image and intensity image

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
    std::string pcdfile = "camera.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile, *cloud);

    std::string imgfile = "/home/whubooks/kitti_odometry/data_odometry_gray/sequences/00/image_0/000020.png";
    std::string calibfile = "/home/whubooks/kitti_odometry/data_odometry_calib/sequences/00/calib.txt";

    cv::Mat img_camera = cv::imread(imgfile, CV_LOAD_IMAGE_GRAYSCALE);
    cv::imshow("img_camera",img_camera);
    int rows = img_camera.rows;
    int cols = img_camera.cols;
    Matrix34d calib = LoadCalibFile(calibfile);
    Eigen::Matrix3d K = calib.block(0, 0, 3, 3);
    double K_fx = K(0, 0), K_dx = K(0, 2), K_fy = K(1, 1), K_dy = K(1, 2);

    std::vector<double> vdepth(rows * cols, 0.0);
    std::vector<double> vintensity(rows * cols, 0.0);

    for (const pcl::PointXYZI &pt : cloud->points)
    {
        Eigen::Vector3d xyz(pt.x, pt.y, pt.z);
        Eigen::Vector3d uv_z = K * xyz;

        int pt_row = std::floor(uv_z(1) / pt.z);
        int pt_col = std::floor(uv_z(0) / pt.z);

        int index = pt_row * cols + pt_col;
        if (vdepth[index] == 0.0 || vdepth[index] > pt.z)
        {
            vdepth[index] = pt.z;
            vintensity[index] = pt.intensity;
        }
    }

    double zmin = *(std::min_element(vdepth.begin(), vdepth.end()));
    double zmax = *(std::max_element(vdepth.begin(), vdepth.end()));
    double rmin = *(std::min_element(vintensity.begin(), vintensity.end()));
    double rmax = *(std::max_element(vintensity.begin(), vintensity.end()));

    for (double &val : vdepth)
        val = (val - zmin) / (zmax - zmin);

    for(double &val : vintensity)
        val=(val-rmin)/(rmax-rmin);

    cv::Mat img_depth(rows, cols, CV_64FC1, vdepth.data());
    cv::imshow("img_depth", img_depth);
    cv::Mat img_intensity(rows,cols,CV_64FC1,vintensity.data());
    cv::imshow("img_intensity",img_intensity);


    cv::Mat img_depth_show(rows, cols, CV_8UC1);
    cv::Mat img_intensity_show(rows,cols,CV_8UC1);
    for(int i=0;i<rows;i++)
    {
        double * row_depth=img_depth.ptr<double>(i);
        double * row_intensity=img_intensity.ptr<double>(i);
        uchar * row_depth_show=img_depth_show.ptr<uchar>(i);
        uchar * row_intensity_show=img_intensity_show.ptr<uchar>(i);

        for(int j=0;j<cols;j++)
        {
            row_depth_show[j]=cv::saturate_cast<uchar>(row_depth[j]*255);
            row_intensity_show[j]=cv::saturate_cast<uchar>(row_intensity[j]*255);
        }
    }

    cv::medianBlur(img_depth_show,img_depth_show,3);
    cv::medianBlur(img_intensity_show,img_intensity_show,3);

    cv::imshow("img_depth_show", img_depth_show);
    cv::imwrite("img_depth.png",img_depth_show);
    cv::imshow("img_intensity_show",img_intensity_show);

    cv::Mat img_camera_canny(rows,cols,CV_8UC1);
    cv::Mat img_depth_canny(rows,cols,CV_8UC1);
    cv::Mat img_intensity_canny(rows,cols,CV_8UC1);

    cv::Canny(img_camera,img_camera_canny,20,180);
    cv::Canny(img_depth_show,img_depth_canny,20,180);
    cv::Canny(img_intensity_show,img_intensity_canny,20,180);

    cv::imshow("img_camera_canny",img_camera_canny);
    cv::imshow("img_depth_canny",img_depth_canny);
    cv::imshow("img_intensity_canny",img_intensity_canny);

    cv::Ptr<cv::LineSegmentDetector> lsd_ptr=cv::createLineSegmentDetector();
    std::vector<cv::Vec4i> vline_depth,vline_intensity,vline_camera;
    lsd_ptr->detect(img_depth_show,vline_depth);
    lsd_ptr->detect(img_intensity_show,vline_intensity);
    lsd_ptr->detect(img_camera,vline_camera);

    cv::Mat img_depth_lsd(rows,cols,CV_8UC1);
    cv::Mat img_intensity_lsd(rows,cols,CV_8UC1);
    cv::Mat img_camera_lsd(rows,cols,CV_8UC1);
    for(const cv::Vec4i &line : vline_depth)
        cv::line(img_depth_lsd,cv::Point(line[0],line[1]),cv::Point(line[2],line[3]),cv::Scalar(255));
    for(const cv::Vec4i &line : vline_intensity)
        cv::line(img_intensity_lsd,cv::Point(line[0],line[1]),cv::Point(line[2],line[3]),cv::Scalar(255));
    for(const cv::Vec4i &line : vline_camera)
        cv::line(img_camera_lsd,cv::Point(line[0],line[1]),cv::Point(line[2],line[3]),cv::Scalar(255));
    cv::imshow("img_camera_lsd",img_camera_lsd);
    cv::imshow("img_depth_lsd",img_depth_lsd);
    cv::imshow("img_intensity_lsd",img_intensity_lsd);

    cv::waitKey(0);
}