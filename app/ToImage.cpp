//
// Created by whubooks on 18-4-23.
//

#include <iostream>
#include <vector>
#include <string>
#include <pcl/common/transforms.h>
#include <Util.h>
#include <CloudUtil.h>
#include <PointCloudProject.h>
#include <KittiData.h>
#include <LineSupportRegion.h>

int main(int argc,char **argv)
{
    std::string pcd_file = (argc > 1) ? std::string(argv[1]) : "50_mls.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcd_file,*cloud);

    std::string calib_file = "../data/calib.txt";
    Eigen::Matrix3f K = PCM::KittiData::LoadKFromFile(calib_file);
    std::cout<<"K ~ \n"<<K<<std::endl;

    std::string img_file = "../data/000050.png";
    cv::Mat img = cv::imread(img_file, cv::IMREAD_GRAYSCALE);
    int img_rows = img.rows;
    int img_cols = img.cols;
    std::cout<<"Image Size ~ [ "<<img_rows<<" , "<<img_cols<<" ]"<<std::endl;

    float angle=20.0f;
    Eigen::Affine3f affine = Eigen::Affine3f::Identity();
    affine.rotate(Eigen::AngleAxisf(angle * M_PI / 180.0f, Eigen::Vector3f::UnitY()));
    affine.translate(Eigen::Vector3f(0.0f, 0.0f, 0.0f));

    PCM::CloudXYZIPtr cloud_trans(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*cloud, *cloud_trans, affine);


    PCM::PointCloudProject project;
    project.SetInput(cloud_trans);
    project.SetK(K);
    project.SetImageSize(img_rows, img_cols);
    project.Project2();
    project.EDLScale();
    std::vector<uchar> gray_shadow_vec = project.GetGrayShadowVec(true);
    std::vector<uchar> gray_depth_vec=project.GetGrayDepthVec();
    cv::Mat img_shadow(img_rows, img_cols, CV_8UC1, gray_shadow_vec.data());
    cv::Mat img_depth(img_rows,img_cols,CV_8UC1,gray_depth_vec.data());

    PCM::LineSupportRegion lsr(img_rows, img_cols, gray_shadow_vec);
    lsr.SetThres(20,10);
    lsr.Detect3();

    std::vector<PCM::Line2D> line_vec = lsr.GetLine();
    std::vector<std::vector<cv::Point2i>> line_region_2d_vec = lsr.GetLineRegion();

    std::vector<PCM::Line2D> left_line_vec = lsr.GetLeftLine();
    std::vector<std::vector<cv::Point2i>> left_line_region_2d_vec = lsr.GetLeftLineRegion();

    std::vector<PCM::Line2D> right_line_vec = lsr.GetRightLine();
    std::vector<std::vector<cv::Point2i>> right_line_region_2d_vec = lsr.GetRightLineRegion();

    std::vector<PCM::Envelop2D> env_vec=lsr.GetEnvelop();
    std::vector<std::vector<cv::Point2i>> env_region_2d_vec = lsr.GetEnvRegion();
    std::vector<std::vector<cv::Point2i>> left_env_region_2d_vec = lsr.GetLeftEnvRegion();
    std::vector<std::vector<cv::Point2i>> right_env_region_2d_vec = lsr.GetRightEnvRegion();
    std::cout << "Project and Detect ~ " << line_region_2d_vec.size() << std::endl<<std::endl;

    cv::Mat img_line;
    img_shadow.copyTo(img_line);
    cv::cvtColor(img_line,img_line,CV_GRAY2BGR);
    for(int i=0;i<line_vec.size();i++)
    {
        PCM::Line2D line=line_vec[i];
        cv::line(img_line,line.s,line.e,cv::Scalar(0,0,255));
    }

    cv::Mat img_env;
    img_shadow.copyTo(img_env);
    cv::cvtColor(img_env,img_env,CV_GRAY2BGR);
    for(int i=0;i<env_vec.size();i++)
    {
        PCM::Envelop2D env=env_vec[i];

        cv::line(img_env,env.lt,env.rt,cv::Scalar(255,0,0));
        cv::line(img_env,env.rt,env.rb,cv::Scalar(255,0,0));
        cv::line(img_env,env.rb,env.lb,cv::Scalar(255,0,0));
        cv::line(img_env,env.lb,env.lt,cv::Scalar(255,0,0));
    }

    cv::imwrite("img_depth.png",img_depth);
    cv::imwrite("img_shadow.png",img_shadow);
    cv::imwrite("img_line.png",img_line);
    cv::imwrite("img_env.png",img_env);
    return 1;
}