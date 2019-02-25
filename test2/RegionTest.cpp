//
// Created by whubooks on 18-4-25.
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

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

/// back-project region
pcl::PointCloud<pcl::PointXYZ>::Ptr BackProject(const std::vector<cv::Point2i> &region_2d,const std::vector<float> &vdepth,const Eigen::Matrix3f &K,int rows,int cols)
{
    typedef std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> PosVec;
    PosVec pos_vec;
    for(const cv::Point2i &pos_2d : region_2d)
    {
        int col=pos_2d.x;
        int row=pos_2d.y;
        float depth=vdepth[row*cols+col];

        if(depth==-1)
            continue;

        pos_vec.push_back(Eigen::Vector3f(col,row,depth));
    }

    /// apply median filter on depth to reduce the error
    bool use_median=true;
    if(use_median)
    {
        PosVec median_pos_vec;
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
                    K.inverse() * Eigen::Vector3f(col_ii * median_depth, row_ii * median_depth, median_depth);
            median_pos_vec.push_back(pt_3d);
        }
        pos_vec.swap(median_pos_vec);
    }
    else
    {
        PosVec tmp_pos_vec;
        for(const Eigen::Vector3f &pos : pos_vec)
        {
            Eigen::Vector3f tmp_pos=K.inverse()*Eigen::Vector3f(pos(0)*pos(2),pos(1)*pos(2),pos(2));
            tmp_pos_vec.push_back(tmp_pos);
        }
        pos_vec.swap(tmp_pos_vec);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(const Eigen::Vector3f &pos : pos_vec)
    {
        pcl::PointXYZ pt;
        pt.x=pos(0);
        pt.y=pos(1);
        pt.z=pos(2);
        cloud->push_back(pt);
    }
    return cloud;

}

/// back-project line
pcl::PointCloud<pcl::PointXYZ>::Ptr BackProject2(const std::vector<cv::Point2i> &region_2d,const std::vector<float> &vdepth,const Eigen::Matrix3f &K,int rows,int cols)
{
    typedef std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> PosVec;
    PosVec pos_vec;
    for(const cv::Point2i &pos_2d : region_2d)
    {
        int col=pos_2d.x;
        int row=pos_2d.y;
        float depth=vdepth[row*cols+col];

        if(depth==-1)
            continue;

        pos_vec.push_back(Eigen::Vector3f(col,row,depth));
    }

    /// apply median filter on depth to reduce the error
    bool use_median=true;
    if(use_median)
    {
        PosVec median_pos_vec;
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
                    K.inverse() * Eigen::Vector3f(col_ii * median_depth, row_ii * median_depth, median_depth);
            median_pos_vec.push_back(pt_3d);
        }
        pos_vec.swap(median_pos_vec);
    }
    else
    {
        PosVec tmp_pos_vec;
        for(const Eigen::Vector3f &pos : pos_vec)
        {
            Eigen::Vector3f tmp_pos=K.inverse()*Eigen::Vector3f(pos(0)*pos(2),pos(1)*pos(2),pos(2));
            tmp_pos_vec.push_back(tmp_pos);
        }
        pos_vec.swap(tmp_pos_vec);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(const Eigen::Vector3f &pos : pos_vec)
    {
        pcl::PointXYZ pt;
        pt.x=pos(0);
        pt.y=pos(1);
        pt.z=pos(2);
        cloud->push_back(pt);
    }
    return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr Search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr target)
{
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    flann->setInputCloud(cloud);

    const float radius=0.05f;
    std::vector<int> search_indices;
    for(const pcl::PointXYZ &pt : target->points)
    {
        std::vector<int> vindices;
        std::vector<float> vdistance;
        flann->radiusSearch(pt,radius,vindices,vdistance);

        search_indices.insert(search_indices.end(),vindices.begin(),vindices.end());
    }

    std::sort(search_indices.begin(),search_indices.end());
    std::vector<int> unique_indices;
    int last_index=-100000000;
    for(const int &index : search_indices)
    {
        if(last_index!=index)
            unique_indices.push_back(index);
        last_index=index;
    }

    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    indices->indices.swap(unique_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_search(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud,*indices,*cloud_search);
    return cloud_search;
}

std::vector<cv::Point2f> Cloud2Pixel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const Eigen::Matrix3f &K)
{
    std::vector<cv::Point2f> pixels;
    for(const pcl::PointXYZ &pt : cloud->points)
    {
        Eigen::Vector3f pos(pt.x,pt.y,pt.z);
        Eigen::Vector3f pos_pixel=K*pos;

        Eigen::Vector2f pixel=pos_pixel.head<2>()/pos_pixel(2);
        pixels.push_back(cv::Point2f(pixel(0),pixel(1)));
    }
    return pixels;
}


int main(int argc,char **argv)
{
    std::string pcd_file = (argc > 1) ? std::string(argv[1]) : "50_stat.pcd";
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

    /// cloud viewer thread
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line_region(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_line_region(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right_line_region(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_env_region(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_env_region(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right_env_region(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("viewer"));
    visualizer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, 1);
    visualizer->initCameraParameters();

    visualizer->addPointCloud(cloud_line_region, "line_region");
    visualizer->addPointCloud(cloud_left_line_region, "left_line_region");
    visualizer->addPointCloud(cloud_right_line_region, "right_line_region");

    visualizer->addPointCloud(cloud_env_region, "env_region");
    visualizer->addPointCloud(cloud_left_env_region, "left_env_region");
    visualizer->addPointCloud(cloud_right_env_region, "right_env_region");

    std::thread edge_show = std::thread([&]() {
        std::cout << "Start Cloud Viewer!" << std::endl;
        while (!visualizer->wasStopped())
        {
            visualizer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << "Stop Cloud Viewer!" << std::endl;
    });

    PCM::PointCloudProject project;
    project.SetInput(cloud);
    project.SetK(K);
    project.SetImageSize(img_rows, img_cols);
    project.Project2();
    project.EDLScale();
    std::vector<float> origin_depth_vec = project.GetOriginDeph();
    std::vector<uchar> gray_shadow_vec = project.GetGrayShadowVec(true);

    cv::Mat img_lsr(img_rows, img_cols, CV_8UC1, gray_shadow_vec.data());
    cv::cvtColor(img_lsr, img_lsr, CV_GRAY2BGR);

    PCM::LineSupportRegion lsr(img_rows, img_cols, gray_shadow_vec);
    lsr.Detect3();

    std::vector<PCM::Line2D> line_vec = lsr.GetLine();
    std::vector<std::vector<cv::Point2i>> line_region_2d_vec = lsr.GetLineRegion();

    std::vector<PCM::Line2D> left_line_vec = lsr.GetLeftLine();
    std::vector<std::vector<cv::Point2i>> left_line_region_2d_vec = lsr.GetLeftLineRegion();

    std::vector<PCM::Line2D> right_line_vec = lsr.GetRightLine();
    std::vector<std::vector<cv::Point2i>> right_line_region_2d_vec = lsr.GetRightLineRegion();

    std::vector<std::vector<cv::Point2i>> env_region_2d_vec=lsr.GetEnvRegion();
    std::vector<std::vector<cv::Point2i>> left_env_region_2d_vec=lsr.GetLeftEnvRegion();
    std::vector<std::vector<cv::Point2i>> right_env_region_2d_vec=lsr.GetRightEnvRegion();

    std::cout << "Project and Detect ~ " << line_region_2d_vec.size() << std::endl;

    for (int i = 0; i < line_vec.size(); i++)
    {
        PCM::Line2D line_i = line_vec[i];
        std::vector<cv::Point2i> region_2d_i = line_region_2d_vec[i];

        PCM::Line2D left_line_i = left_line_vec[i];
        std::vector<cv::Point2i> left_line_region_2d_i = left_line_region_2d_vec[i];

        PCM::Line2D right_line_i = right_line_vec[i];
        std::vector<cv::Point2i> right_line_region_2d_i = right_line_region_2d_vec[i];

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line_region_i = BackProject2(region_2d_i, origin_depth_vec, K, img_rows, img_cols);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_line_region_i = BackProject2(left_line_region_2d_i, origin_depth_vec, K, img_rows, img_cols);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right_line_region_i = BackProject2(right_line_region_2d_i, origin_depth_vec, K, img_rows, img_cols);

        std::vector<cv::Point2i> env_region_2d_i=env_region_2d_vec[i];
        std::vector<cv::Point2i> left_env_region_2d_i=left_env_region_2d_vec[i];
        std::vector<cv::Point2i> right_env_region_2d_i=right_env_region_2d_vec[i];

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_env_region_i = BackProject(env_region_2d_i,origin_depth_vec,K,img_rows,img_cols);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_env_region_i = BackProject(left_env_region_2d_i,origin_depth_vec,K,img_rows,img_cols);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right_env_region_i = BackProject(right_env_region_2d_i,origin_depth_vec,K,img_rows,img_cols);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line_search_region_i=Search(cloud_pos,cloud_line_region_i);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_region_search_region_i=Search(cloud_pos,cloud_left_env_region_i);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right_region_search_region_i=Search(cloud_pos,cloud_right_env_region_i);

        {
            if (!cloud_line_search_region_i->empty())
            {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_line_region_color_i(cloud_line_region_i,
                                                                                                     250, 0, 0);
                visualizer->updatePointCloud(cloud_line_region_i, cloud_line_region_color_i, "line_region");
                visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"line_region");
            }
//            if (!cloud_left_line_region_i->empty())
//            {
//                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_left_line_region_color_i(
//                        cloud_left_line_region_i, 0, 250, 0);
//                visualizer->updatePointCloud(cloud_left_line_region_i, cloud_left_line_region_color_i, "left_line_region");
//                visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"left_line_region");
//            }
//            if (!cloud_right_line_region_i->empty())
//            {
//                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_right_line_region_color_i(
//                        cloud_right_line_region_i, 0, 0, 250);
//                visualizer->updatePointCloud(cloud_right_line_region_i, cloud_right_line_region_color_i, "right_line_region");
//                visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"right_line_region");
//            }

//            if (!cloud_env_region_i->empty())
//            {
//                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_env_region_color_i(cloud_env_region_i,
//                                                                                                          120, 120, 120);
//                visualizer->updatePointCloud(cloud_env_region_i, cloud_env_region_color_i, "env_region");
//            }
            if (!cloud_left_region_search_region_i->empty())
            {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_left_env_region_color_i(
                        cloud_left_region_search_region_i, 0, 250, 0);
                visualizer->updatePointCloud(cloud_left_region_search_region_i, cloud_left_env_region_color_i, "left_env_region");
            }
            if (!cloud_right_region_search_region_i->empty())
            {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_right_env_region_color_i(
                        cloud_right_region_search_region_i, 0, 0, 250);
                visualizer->updatePointCloud(cloud_right_region_search_region_i, cloud_right_env_region_color_i, "right_env_region");
            }
        }

//        cv::Mat img_line;
//        img_lsr.copyTo(img_line);
//        cv::line(img_line, line_i.s, line_i.e, cv::Scalar(0, 0, 250));
//        cv::line(img_line, left_line_i.s, left_line_i.e, cv::Scalar(0, 250, 0));
//        cv::line(img_line, right_line_i.s, right_line_i.e, cv::Scalar(250, 0, 0));
//        cv::imshow("img_line", img_line);
//
//        cv::Mat img_left_region;
//        img_lsr.copyTo(img_left_region);
//        for(const cv::Point2i &pt : left_env_region_2d_i)
//            cv::circle(img_left_region,pt,1,cv::Scalar(0,250,0));
//        cv::imshow("img_left_region",img_left_region);
//
//        cv::Mat img_right_region;
//        img_lsr.copyTo(img_right_region);
//        for(const cv::Point2i &pt : right_env_region_2d_i)
//            cv::circle(img_right_region,pt,1,cv::Scalar(250,0,0));
//        cv::imshow("img_right_region",img_right_region);

        std::vector<cv::Point2f> project_line_search_region_i=Cloud2Pixel(cloud_line_region_i,K);
        cv::Mat img_project_line;
        img_lsr.copyTo(img_project_line);
        for(const cv::Point2f &pt_2d : project_line_search_region_i)
            cv::circle(img_project_line,pt_2d,1,cv::Scalar(0,0,255));
        cv::imshow("img_project_line", img_project_line);

        std::vector<cv::Point2f> project_left_env_search_region_i=Cloud2Pixel(cloud_left_region_search_region_i,K);
        cv::Mat img_project_left_env;
        img_lsr.copyTo(img_project_left_env);
        for(const cv::Point2f &pt_2d : project_left_env_search_region_i)
            cv::circle(img_project_left_env,pt_2d,1,cv::Scalar(0,255,0));
        cv::imshow("img_project_left_env", img_project_left_env);

        std::vector<cv::Point2f> project_right_env_search_region_i=Cloud2Pixel(cloud_right_region_search_region_i,K);
        cv::Mat img_project_right_env;
        img_lsr.copyTo(img_project_right_env);
        for(const cv::Point2f &pt_2d : project_right_env_search_region_i)
            cv::circle(img_project_right_env,pt_2d,1,cv::Scalar(255,0,0));
        cv::imshow("img_project_right_env", img_project_right_env);

        cv::waitKey(0);
    }
    visualizer->removeAllPointClouds();
    visualizer->close();

}