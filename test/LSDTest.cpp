//
// Created by whubooks on 18-3-11.
//

#include <iostream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/line_descriptor.hpp>


int main()
{
    std::string filename="/home/whubooks/kitti_odometry/data_odometry_gray/sequences/00/image_0/000000.png";

    cv::Mat img=cv::imread(filename,cv::IMREAD_GRAYSCALE);
    cv::Mat img_canny(img.rows,img.cols,CV_8UC1);
    cv::Mat img_draw(img.rows,img.cols,CV_8UC1);

   // cv::imshow("Origin",img);

    cv::blur(img,img,cv::Size(3,3));
    cv::Canny(img,img_canny,50,200,3);
    cv::imshow("Canny",img_canny);

    cv::Ptr<cv::LineSegmentDetector> lsd_ptr=cv::createLineSegmentDetector();
    std::vector<cv::Vec4i> lines;
    lsd_ptr->detect(img_canny,lines);
   // lsd_ptr->drawSegments(img_draw,lines);
    std::cout<<"Lsd Lines Size ~ "<<lines.size()<<std::endl;

    for(const cv::Vec4i &line : lines)
    {
        cv::Point2d pt_s(line[0],line[1]);
        cv::Point2d pt_e(line[2],line[3]);
        cv::line(img_draw,pt_s,pt_e,cv::Scalar(255));
    }

    cv::imshow("LSD",img_draw);
    cv::waitKey(0);
}