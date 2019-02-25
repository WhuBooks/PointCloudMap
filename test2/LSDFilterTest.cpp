//
// Created by whubooks on 18-5-4.
//

#include <iostream>
#include <vector>
#include <string>

#include <Util.h>
#include <CloudUtil.h>
#include <PointCloudProject.h>
#include <KittiData.h>
#include <LineSupportRegion.h>
#include <EdgeFilter.h>

PCM::CloudXYZPtr BackProjectEnv(const PCM::Region2D &region_2d,const std::vector<float> &m_origin_depth,const Eigen::Matrix3f &m_K,int m_rows,int m_cols)
{
    PCM::PosVec pos_vec;
    for (const cv::Point2i &pos_2d : region_2d)
    {
        int col = pos_2d.x;
        int row = pos_2d.y;
        float depth = m_origin_depth[row * m_cols + col];

        if (depth == -1)
            continue;

        pos_vec.push_back(Eigen::Vector3f(col, row, depth));
    }

    /// apply median filter on depth to reduce the error
    bool use_median = true;
    if (use_median)
    {
        PCM::PosVec median_pos_vec;
        const int N = 2;
        for (int ii = 0; ii < pos_vec.size(); ii++)
        {
            std::vector<float> tmp_depth;
            for (const Eigen::Vector3f &pos : pos_vec)
            {
                Eigen::Vector2f v = pos.head<2>() - pos_vec[ii].head<2>();

                if (v.norm() <= N)
                    tmp_depth.push_back(pos(2));
            }

            std::sort(tmp_depth.begin(), tmp_depth.end());
            float median_depth = 0.0f;
            if (tmp_depth.size() % 2 == 1)
                median_depth = tmp_depth[(tmp_depth.size() - 1) / 2];
            else
                median_depth = (tmp_depth[tmp_depth.size() / 2] + tmp_depth[tmp_depth.size() / 2 - 1]) / 2.0f;

            float col_ii = pos_vec[ii](0);
            float row_ii = pos_vec[ii](1);
            Eigen::Vector3f pt_3d =
                    m_K.inverse() * Eigen::Vector3f(col_ii * median_depth, row_ii * median_depth, median_depth);
            median_pos_vec.push_back(pt_3d);
        }
        pos_vec.swap(median_pos_vec);
    }
    else
    {
        PCM::PosVec tmp_pos_vec;
        for (const Eigen::Vector3f &pos : pos_vec)
        {
            Eigen::Vector3f tmp_pos = m_K.inverse() * Eigen::Vector3f(pos(0) * pos(2), pos(1) * pos(2), pos(2));
            tmp_pos_vec.push_back(tmp_pos);
        }
        pos_vec.swap(tmp_pos_vec);
    }

    PCM::CloudXYZPtr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const Eigen::Vector3f &pos : pos_vec)
    {
        pcl::PointXYZ pt(pos(0), pos(1), pos(2));
        cloud->push_back(pt);
    }
    return cloud;
    
}

PCM::CloudXYZPtr BackProjectLine(const PCM::Region2D &region_2d,const std::vector<float> &m_origin_depth,const Eigen::Matrix3f &m_K,int m_rows,int m_cols)
{
    PCM::PosVec pos_vec;
    for (const cv::Point2i &pos_2d : region_2d)
    {
        int col = pos_2d.x;
        int row = pos_2d.y;
        float depth = m_origin_depth[row * m_cols + col];

        if (depth == -1)
            continue;

        pos_vec.push_back(Eigen::Vector3f(col, row, depth));
    }

    /// apply median filter on depth to reduce the error
    bool use_median = true;
    if (use_median)
    {
        PCM::PosVec median_pos_vec;
        const int N = 2;
        for (int ii = 0; ii < pos_vec.size(); ii++)
        {
            std::vector<float> tmp_depth;
            for (int jj = std::max(0, ii - N); jj <= std::min(ii + N, (int) pos_vec.size() - 1); jj++)
            {
                float depth_jj = pos_vec[jj](2);
                tmp_depth.push_back(depth_jj);
            }

            std::sort(tmp_depth.begin(), tmp_depth.end());
            float median_depth = 0.0f;
            if (tmp_depth.size() % 2 == 1)
                median_depth = tmp_depth[(tmp_depth.size() - 1) / 2];
            else
                median_depth = (tmp_depth[tmp_depth.size() / 2] + tmp_depth[tmp_depth.size() / 2 - 1]) / 2.0f;

            float col_ii = pos_vec[ii](0);
            float row_ii = pos_vec[ii](1);
            Eigen::Vector3f pt_3d =
                    m_K.inverse() * Eigen::Vector3f(col_ii * median_depth, row_ii * median_depth, median_depth);
            median_pos_vec.push_back(pt_3d);
        }
        pos_vec.swap(median_pos_vec);
    }
    else
    {
        PCM::PosVec tmp_pos_vec;
        for (const Eigen::Vector3f &pos : pos_vec)
        {
            Eigen::Vector3f tmp_pos = m_K.inverse() * Eigen::Vector3f(pos(0) * pos(2), pos(1) * pos(2), pos(2));
            tmp_pos_vec.push_back(tmp_pos);
        }
        pos_vec.swap(tmp_pos_vec);
    }

    PCM::CloudXYZPtr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const Eigen::Vector3f &pos : pos_vec)
    {
        pcl::PointXYZ pt(pos(0), pos(1), pos(2));
        cloud->push_back(pt);
    }
    return cloud;
}

int main(int argc,char **argv)
{
    std::string pcd_file = (argc > 1) ? std::string(argv[1]) : "50_mls.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcd_file, *cloud);

    PCM::CloudXYZPtr cloud_pos(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud,*cloud_pos);

    std::string calib_file = "../data/calib.txt";
    Eigen::Matrix3f K = PCM::KittiData::LoadKFromFile(calib_file);
    std::cout << "K ~ \n" << K << std::endl;

    std::string img_file = "../data/000050.png";
    cv::Mat img = cv::imread(img_file, cv::IMREAD_GRAYSCALE);
    int img_rows = img.rows;
    int img_cols = img.cols;
    std::cout << "Image Size ~ [ " << img_rows << " , " << img_cols << " ]" << std::endl;

    PCM::PointCloudProject project;
    project.SetInput(cloud);
    project.SetK(K);
    project.SetImageSize(img_rows, img_cols);
    project.Project2();
    project.EDLScale();
    std::vector<float> origin_depth_vec = project.GetOriginDeph();
    std::vector<uchar> gray_shadow_vec = project.GetGrayShadowVec(true);
    cv::Mat img_lsr(img_rows, img_cols, CV_8UC1, gray_shadow_vec.data());
    cv::cvtColor(img_lsr,img_lsr,CV_GRAY2BGR);

    PCM::LineSupportRegion lsr(img_rows, img_cols, gray_shadow_vec);
    lsr.SetThres(0, 0);
    lsr.Detect3();
    std::cout << "Project and Detect Done." << std::endl;

    std::vector<PCM::Line2D> line_vec = lsr.GetLine();
    std::vector<PCM::Region2D> line_region_2d_vec = lsr.GetLineRegion();

    std::vector<PCM::Line2D> left_line_vec = lsr.GetLeftLine();
    std::vector<std::vector<cv::Point2i>> left_line_region_2d_vec = lsr.GetLeftLineRegion();

    std::vector<PCM::Line2D> right_line_vec = lsr.GetRightLine();
    std::vector<PCM::Region2D> right_line_region_2d_vec = lsr.GetRightLineRegion();

    std::vector<PCM::Envelop2D> env_vec = lsr.GetEnvelop();
    std::vector<PCM::Region2D> env_region_2d_vec = lsr.GetEnvRegion();
    std::vector<PCM::Region2D> left_env_region_2d_vec = lsr.GetLeftEnvRegion();
    std::vector<PCM::Region2D> right_env_region_2d_vec = lsr.GetRightEnvRegion();
    
    for(int i=0;i<line_vec.size();i++)
    {
        PCM::Line2D line = line_vec[i];
        PCM::Region2D line_region = line_region_2d_vec[i];
        PCM::Line2D left_line = left_line_vec[i];
        PCM::Region2D left_line_region = left_line_region_2d_vec[i];
        PCM::Line2D right_line = right_line_vec[i];
        PCM::Region2D right_line_region = right_line_region_2d_vec[i];

        PCM::CloudXYZPtr line_region_back = BackProjectLine(line_region, origin_depth_vec, K, img_rows, img_cols);
        PCM::CloudXYZPtr left_line_region_back = BackProjectLine(left_line_region, origin_depth_vec, K, img_rows, img_cols);
        PCM::CloudXYZPtr right_line_region_back = BackProjectLine(right_line_region, origin_depth_vec, K, img_rows, img_cols);

        PCM::Region2D env_region = env_region_2d_vec[i];
        PCM::Region2D left_env_region = left_env_region_2d_vec[i];
        PCM::Region2D right_env_region = right_env_region_2d_vec[i];

        PCM::CloudXYZPtr env_region_back = BackProjectEnv(env_region, origin_depth_vec, K, img_rows, img_cols);
        PCM::CloudXYZPtr left_env_region_back = BackProjectEnv(left_env_region, origin_depth_vec, K, img_rows, img_cols);
        PCM::CloudXYZPtr right_env_region_back = BackProjectEnv(right_env_region, origin_depth_vec, K, img_rows, img_cols);

    }


}