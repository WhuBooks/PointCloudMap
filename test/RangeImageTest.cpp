//
// Created by books on 2018/3/20.
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
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/range_image/impl/range_image.hpp>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/png_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <sophus/se3.h>
#include <opencv2/opencv.hpp>

cv::Mat ImagePlanar2Mat(pcl::RangeImagePlanar::Ptr ptr)
{
    int imgW=1241,imgH=376;
    cv::Mat img(imgH,imgW,CV_32FC1);
    for(const pcl::PointWithRange &pt : ptr->points)
    {
        float x=pt.x;
        float y=pt.y;
        float z=pt.z;
        int img_x=0,img_y=0;
        float range=0.0f;
        ptr->getImagePoint(Eigen::Vector3f(x,y,z),img_x,img_y,range);
        float val=img.at<float>(img_y,img_x);
        if(val>z||val==0)
            img.at<float>(img_y,img_x)=z;
    }
    return img;
}

cv::Mat FCTo8UC1(const cv::Mat &img)
{
    cv::Mat result(img.rows, img.cols, CV_8UC1);

    double minVal,maxval;
    int minId,maxId;
    cv::minMaxIdx(img,&minVal,&maxval,&minId,&maxId);
    for (int i = 0; i < img.rows; i++)
    {
        for(int j=0;j<img.cols;j++)
        {
            float val=img.at<float>(i,j);
            uchar val2=cv::saturate_cast<uchar>((val-minVal)/(maxval-minVal)*255);
            result.at<uchar>(i,j)=val2;
        }
    }
    return result;
}

int main()
{
    std::string pcdfile = "voxel.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile, *cloud);

    // We now want to create a range image from the above point cloud, with a 1deg angular resolution
    float angularResolution = (float) (0.1f * (M_PI / 180.0f));  //   1.0 degree in radians
    float maxAngleWidth = (float) (360.0f * (M_PI / 180.0f));  // 180.0 degree in radians
    float maxAngleHeight = (float) (180.0f * (M_PI / 180.0f));  // 180.0 degree in radians
    Eigen::Matrix4f T;
    T << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    Eigen::Affine3f sensorPose(T);
    //Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel = 0.00;
    float minRange = 0.0f;
    int borderSize = 1;

    pcl::RangeImagePlanar::Ptr range_image_planar_ptr(new pcl::RangeImagePlanar);
    int imgW=1241,imgH=376;
   // range_image_planar_ptr->createFromPointCloud(*cloud,angularResolution,maxAngleWidth,maxAngleHeight,sensorPose,coordinate_frame,noiseLevel,minRange,borderSize);
   range_image_planar_ptr->createFromPointCloudWithFixedSize(*cloud, imgW, imgH, 607.1928, 185.2157, 718.8560, 718.8560, sensorPose, coordinate_frame, noiseLevel, minRange);
   // pcl::io::saveRangeImagePlanarFilePNG("imagePlanar.png", *range_image_planar_ptr);

    cv::Mat frame=ImagePlanar2Mat(range_image_planar_ptr);
    cv::Mat gray=FCTo8UC1(frame);
    cv::imshow("frame",gray);
    cv::imwrite("img.png",gray);

    pcl::io::savePNGFile("imagePlanar.png",*range_image_planar_ptr,"z");

//    pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
//    range_image_ptr->createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight,
//                                          sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    cv::waitKey(0);
    pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
    range_image_widget.showRangeImage(*range_image_planar_ptr);

    //getchar();
    while (!range_image_widget.wasStopped())
    {
        range_image_widget.spinOnce();
        // Sleep 100ms to go easy on the CPU.
        pcl_sleep(0.1);
    }
}