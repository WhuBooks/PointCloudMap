//
// Created by whubooks on 18-4-23.
//

#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <mutex>

#include <pcl/common/transforms.h>

#include <Util.h>
#include <CloudUtil.h>
#include <PointCloudProject.h>
#include <KittiData.h>
#include <LineSupportRegion.h>
#include <EdgeFilter.h>

void WriteEdgePt(const std::vector<PCM::EdgePt> &pts,const std::string &filename)
{
    std::ofstream ofs(filename);
    for(const PCM::EdgePt &pt : pts)
    {
        Eigen::Vector3f pt_pos=pt.pos;
        Eigen::Vector3f pt_dir=pt.dir;
        ofs<<std::setiosflags(std::ios::fixed);
        ofs<<std::setprecision(0)<<pt.id<<"\t";
        ofs<<std::setprecision(8)<<pt_pos(0)<<"\t";
        ofs<<std::setprecision(8)<<pt_pos(1)<<"\t";
        ofs<<std::setprecision(8)<<pt_pos(2)<<"\t";
        ofs<<std::setprecision(8)<<pt_dir(0)<<"\t";
        ofs<<std::setprecision(8)<<pt_dir(1)<<"\t";
        ofs<<std::setprecision(8)<<pt_dir(2)<<std::endl;
    }
    ofs.close();
}


int main(int argc,char **argv)
{
    std::string pcd_file = (argc > 1) ? std::string(argv[1]) : "50_mls.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcd_file, *cloud);

    std::string calib_file = "../data/calib.txt";
    Eigen::Matrix3f K = PCM::KittiData::LoadKFromFile(calib_file);
    std::cout << "K ~ \n" << K << std::endl;

    std::string img_file = "../data/000050.png";
    cv::Mat img = cv::imread(img_file, cv::IMREAD_GRAYSCALE);
    int img_rows = img.rows;
    int img_cols = img.cols;
    std::cout << "Image Size ~ [ " << img_rows << " , " << img_cols << " ]" << std::endl;

    std::string dir =Util::SplitNameWithoutExt(pcd_file) + Util::GetNameFromTime();
    Util::DirBuild(dir);

    PCM::CloudXYZPtr edge_single_all(new pcl::PointCloud<pcl::PointXYZ>);
    PCM::CloudXYZPtr edge_double_all(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<PCM::EdgePt> edge_single_pts_all,edge_double_pts_all;
    std::mutex edge_pts_mutex;

    std::function<void(float)> func=[&](float angle) {
        if (angle >= 360.0f)
            return;

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
        std::cout << "Angle " << angle << " Filter 3D Edge ~ " << edge_num << std::endl;
        std::cout << "Angle " << angle << " Filter Single Edge ~ " << filter.GetSingleEdgeNum() << std::endl;
        std::cout << "Angle " << angle << " Filter Double Edge ~ " << filter.GetDoubleEdgeNum() << std::endl;


        std::string tmp_dir = dir + "/" + std::to_string(angle);
        Util::DirBuild(tmp_dir);

        /// write image and line-planar cloud
        filter.Write(tmp_dir, gray_shadow_vec, affine.inverse());

        /// write edge cloud
        PCM::CloudXYZPtr cloud_single_edge = filter.GetSingleXYZResult(affine.inverse());
        std::string edge_single_file = tmp_dir + "/single_edge.las";
        PCM::WriteLas(cloud_single_edge, edge_single_file);
        PCM::CloudXYZPtr cloud_double_edge=filter.GetDoubleXYZResult(affine.inverse());
        std::string edge_double_file=tmp_dir+"/double_edge.las";
        PCM::WriteLas(cloud_double_edge,edge_double_file);

        /// write edge point
        std::vector<PCM::EdgePt> edge_single_pts = filter.GetSingleEdgePts(affine.inverse());
        std::string edge_single_pts_file = tmp_dir + "/single_edge_pt.txt";
        WriteEdgePt(edge_single_pts, edge_single_pts_file);
        std::vector<PCM::EdgePt> edge_double_pts=filter.GetDoubleEdgePts(affine.inverse());
        std::string edge_double_pts_file=tmp_dir+"/double_edge_pt.txt";
        WriteEdgePt(edge_double_pts,edge_double_pts_file);

        {
            std::unique_lock<std::mutex> lock(edge_pts_mutex);
            edge_single_all->insert(edge_single_all->end(), cloud_single_edge->begin(), cloud_single_edge->end());
            edge_double_all->insert(edge_double_all->end(), cloud_double_edge->begin(), cloud_double_edge->end());
            edge_single_pts_all.insert(edge_single_pts_all.end(), edge_single_pts.begin(), edge_single_pts.end());
            edge_double_pts_all.insert(edge_double_pts_all.end(), edge_double_pts.begin(), edge_double_pts.end());
            lock.unlock();
        }
    };

    const float angle_step = 20.0f;
    for(float cur_angle=0.0f;cur_angle<360.0f;cur_angle=cur_angle+5*angle_step)
    {
        std::cout << "*******************************" << std::endl;

        float angle1 = cur_angle;
        float angle2 = cur_angle + angle_step;
        float angle3 = cur_angle + angle_step * 2;
        float angle4 = cur_angle + angle_step * 3;
        float angle5 = cur_angle + angle_step * 4;

        std::thread thread1(func,angle1);
        std::thread thread2(func,angle2);
        std::thread thread3(func,angle3);
        std::thread thread4(func,angle4);
        std::thread thread5(func,angle5);

        thread1.join();
        thread2.join();
        thread3.join();
        thread4.join();
        thread5.join();

        std::cout << "*******************************" << std::endl << std::endl;
    }

    std::string edge_single_all_file=dir+"/single_edge.las";
    PCM::WriteLas(edge_single_all,edge_single_all_file);

    std::string edge_double_all_file=dir+"/double_edge.las";
    PCM::WriteLas(edge_double_all,edge_double_all_file);

    std::string filename = dir + "/single_edge_pt.txt";
    WriteEdgePt(edge_single_pts_all, filename);

    std::string filename2=dir+"/double_edge_pt.txt";
    WriteEdgePt(edge_double_pts_all,filename2);

    return 1;
}