//
// Created by books on 2018/5/15.
//

#include <iostream>
#include <vector>
#include <string>
#include <mutex>
#include <thread>

#include <Util.h>
#include <CloudUtil.h>
#include <PointCloudProject.h>
#include <KittiData.h>
#include <LineSupportRegion.h>

#include <pcl/visualization/pcl_visualizer.h>

int main(int argc,char **argv)
{
    std::string pcd_file = (argc > 1) ? std::string(argv[1]) : "50_mls.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcd_file, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pos(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *cloud_pos);

    std::string calib_file = "../data/calib.txt";
    Eigen::Matrix3f K = PCM::KittiData::LoadKFromFile(calib_file);
    std::cout << "K ~ \n" << K << std::endl;

    std::string img_file = "../data/000050.png";
    cv::Mat img = cv::imread(img_file, cv::IMREAD_GRAYSCALE);
    int img_rows = img.rows;
    int img_cols = img.cols;
    std::cout << "Image Size ~ [ " << img_rows << " , " << img_cols << " ]" << std::endl;

    Eigen::Affine3f affine = Eigen::Affine3f::Identity();
    affine.rotate(Eigen::AngleAxisf(angle * M_PI / 180.0f, Eigen::Vector3f::UnitY()));
    affine.translate(Eigen::Vector3f(0.0f, 0.0f, 0.0f));

    PCM::CloudXYZIPtr cloud_trans(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*cloud, *cloud_trans, affine);

    /// project into depth and shadow image
    PCM::PointCloudProject project;
    project.SetInput(cloud_trans);
    project.SetK(K);
    project.SetImageSize(img_rows, img_cols);
    project.Project2();
    project.EDLScale();
    std::vector<float> origin_depth_vec = project.GetOriginDeph();
    std::vector<uchar> gray_shadow_vec = project.GetGrayShadowVec(true);

    /// line segment detect
    PCM::LineSupportRegion lsr(img_rows, img_cols, gray_shadow_vec);
    lsr.SetThres(0, 0);
    lsr.Detect3();
    std::vector<PCM::Line2D> line_vec = lsr.GetLine();
    std::vector<PCM::Region2D> line_region_vec = lsr.GetLineRegion();
    std::vector<PCM::Region2D> left_env_region_vec = lsr.GetLeftEnvRegion();
    std::vector<PCM::Region2D> right_env_region_vec = lsr.GetRightEnvRegion();
    std::cout << "Angle " << angle << " Detect 2D Edge ~ " << line_vec.size() << std::endl;

    /// back-project line segment to origin cloud and find possible 3D edge
    PCM::EdgeFilter filter(img_rows, img_cols, K, origin_depth_vec);
    filter.SetCloud(cloud_trans);
    filter.SetRegion2D(line_region_vec, left_env_region_vec, right_env_region_vec);
    int edge_num = filter.Filter2();

    std::vector<CloudXYZPtr> line_back_vec=filter.GetLineBack();
    std::vector<CloudXYZPtr> left_back_vec=filter.GetLeftEnvBack();
    std::vector<CloudXYZPtr> right_back_vec=filter.GetRightEnvBack();

    std::vector<CloudXYZPtr> line_search_vec=filter.GetLineSearch();
    std::vector<CloudXYZPtr> left_search_vec=filter.GetLeftEnvSearch();
    std::vector<CloudXYZPtr> right_search_vec=filter.GetRightEnvSearch();

    std::vector<CloudXYZPtr> line_sac_vec=filter.GetLineSac();
    std::vector<CloudXYZPtr> left_sac_vec=filter.GetLeftEnvSac();
    std::vector<CloudXYZPtr> right_sac_vec=filter.GetRightEnvBack();

    std::vector<CloudXYZPtr> line_project_vec=filter.GetLineEnvProject();
    std::vector<CloudXYZPtr> line_vec=filter.GetLineEnv();


    for(int i=0;i<edge_num;i++)
    {




    }

}