//
// Created by whubooks on 18-4-20.
//

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include <pcl/io/pcd_io.h>

#include <KittiData.h>
#include <PointCloudProject.h>
#include <LineSupportRegion.h>
#include <CloudUtil.h>
#include <Util.h>

/// calculate rotate matrix by rotate azix and rotate angle
Eigen::Matrix3f CalRotation(const Eigen::Vector3f &azix,const float angle)
{
    const float a = azix(0), b = azix(1), c = azix(2);
    const float eps = 0.00000000001f;

    float sqrt_bb_cc=std::sqrt(b*b+c*c);
    if(sqrt_bb_cc==0.0f)
        sqrt_bb_cc+=eps;

    float sqrt_aa_bb_cc=std::sqrt(a*a+b*b+c*c);
    if(sqrt_aa_bb_cc==0.0f)
        sqrt_aa_bb_cc+=eps;

    float cos_alpha = c / sqrt_bb_cc;
    float sin_alpha = b / sqrt_bb_cc;
    float cos_beta = sqrt_bb_cc / sqrt_aa_bb_cc;
    float sin_beta = a / sqrt_aa_bb_cc;
    float cos_theta = std::cos(angle);
    float sin_theta = std::sin(angle);

    Eigen::Matrix3f rx_neg_alpha = Eigen::Matrix3f::Identity();
    rx_neg_alpha(1, 1) = cos_alpha;
    rx_neg_alpha(1, 2) = sin_alpha;
    rx_neg_alpha(2, 1) = -sin_alpha;
    rx_neg_alpha(2, 2) = cos_alpha;
//    std::cout<<"rx_neg_alpha ~ \n"<<rx_neg_alpha<<std::endl;

    Eigen::Matrix3f ry_beta = Eigen::Matrix3f::Identity();
    ry_beta(0, 0) = cos_beta;
    ry_beta(0, 2) = sin_beta;
    ry_beta(2, 0) = -sin_beta;
    ry_beta(2, 2) = cos_beta;
//    std::cout<<"ry_beta ~ \n"<<ry_beta<<std::endl;

    Eigen::Matrix3f rz_theta = Eigen::Matrix3f::Identity();
    rz_theta(0, 0) = cos_theta;
    rz_theta(0, 1) = sin_theta;
    rz_theta(1, 0) = -sin_theta;
    rz_theta(1, 1) = cos_theta;
//    std::cout<<"rz_theta ~ \n"<<rz_theta<<std::endl;

    Eigen::Matrix3f ry_neg_beta = Eigen::Matrix3f::Identity();
    ry_neg_beta(0, 0) = cos_beta;
    ry_neg_beta(0, 2) = -sin_beta;
    ry_neg_beta(2, 0) = sin_beta;
    ry_neg_beta(2, 2) = cos_beta;
//    std::cout<<"ry_neg_beta ~ \n"<<ry_neg_beta<<std::endl;

    Eigen::Matrix3f rx_alpha = Eigen::Matrix3f::Identity();
    rx_alpha(1, 1) = cos_alpha;
    rx_alpha(1, 2) = -sin_alpha;
    rx_alpha(2, 1) = sin_alpha;
    rx_alpha(2, 2) = cos_alpha;
//    std::cout<<"rx_alpha ~ \n"<<rx_alpha<<std::endl;

    Eigen::Matrix3f rotate = rx_neg_alpha * ry_beta * rz_theta * ry_neg_beta * rx_alpha;
    return rotate;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr TransformCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,const Sophus::SE3f &se)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZI>);
    for(const pcl::PointXYZI &pt : cloud->points)
    {
        Eigen::Vector3f pos(pt.x,pt.y,pt.z);
        Eigen::Vector3f pos_trans=se*pos;

        pcl::PointXYZI p;
        p.x=pos_trans(0);
        p.y=pos_trans(1);
        p.z=pos_trans(2);
        p.intensity=pt.intensity;
        cloud_trans->push_back(p);
    }
    return cloud_trans;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr TransformCloud(const std::vector<std::vector<pcl::PointXYZI>> &vec,const Sophus::SE3f &se)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZI>);
    for(const std::vector<pcl::PointXYZI> &edge : vec)
    {
        for (const pcl::PointXYZI &pt : edge)
        {
            Eigen::Vector3f pos(pt.x, pt.y, pt.z);
            Eigen::Vector3f pos_trans = se * pos;

            pcl::PointXYZI p;
            p.x = pos_trans(0);
            p.y = pos_trans(1);
            p.z = pos_trans(2);
            p.intensity = pt.intensity;
            cloud_trans->push_back(p);
        }
    }
    return cloud_trans;
}

std::vector<pcl::PointXYZI> TransformCloud(const std::vector<pcl::PointXYZI> &vec,const Sophus::SE3f &se)
{
    std::vector<pcl::PointXYZI> result;
    for(const pcl::PointXYZI &pt : vec)
    {
        Eigen::Vector3f pos(pt.x,pt.y,pt.z);
        Eigen::Vector3f pos_trans=se*pos;

        pcl::PointXYZI p;
        p.x=pos_trans(0);
        p.y=pos_trans(1);
        p.z=pos_trans(2);
        p.intensity=pt.intensity;
        result.push_back(p);
    }
    return result;
}

float StdDev(const std::vector<float> &vector)
{
    int size=vector.size();

    float ave=std::accumulate(vector.begin(),vector.end(),0.0f)/size;
    float square_sum=0.0f;
    for(const float &tmp : vector)
        square_sum+=(tmp-ave)*(tmp-ave);

    float std_dev=std::sqrt(square_sum/size);
    return std_dev;
}

std::vector<float> MedianFilter(const std::vector<float> &vec,int N)
{
    std::vector<float> median_vec;
    for(int i=0;i<vec.size();i++)
    {
        std::vector<float> tmp_vec;
        for(int j=std::max(0,i-N);j<=std::min((int)vec.size()-1,i+N);j++)
            tmp_vec.push_back(vec[j]);

        std::sort(tmp_vec.begin(),tmp_vec.end());
        float median_depth=0.0f;
        if(tmp_vec.size()%2==1)
            median_depth=tmp_vec[(tmp_vec.size()-1)/2];
        else
            median_depth=(tmp_vec[tmp_vec.size()/2]+tmp_vec[tmp_vec.size()/2-1])/2.0f;
        median_vec.push_back(median_depth);
    }
    return median_vec;
}

typedef std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> PosVec;

std::vector<pcl::PointXYZI> Search(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,const PosVec &vec)
{
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    flann->setInputCloud(cloud);

    const float radius=0.05f;
    std::vector<int> indices;
    for(const Eigen::Vector3f &pos : vec)
    {
        pcl::PointXYZI pt;
        pt.x=pos(0);
        pt.y=pos(1);
        pt.z=pos(2);

        std::vector<int> vindices;
        std::vector<float> vdistance;
        flann->radiusSearch(pt,radius,vindices,vdistance);

        indices.insert(indices.end(),vindices.begin(),vindices.end());
    }

    std::sort(indices.begin(),indices.end());
    std::vector<int> tmp_indices;
    int last_index=-10000000;
    for(const int &tmp_index : indices)
    {
        if(tmp_index!=last_index)
            tmp_indices.push_back(tmp_index);
        last_index=tmp_index;
    }
    indices.swap(tmp_indices);

    std::vector<pcl::PointXYZI> result;
    for(const int &index : indices)
    {
        pcl::PointXYZI pt=cloud->points[index];
        result.push_back(pt);
    }
    return result;
}


int main(int argc,char **argv)
{
    std::string pcd_file = (argc > 1) ? std::string(argv[1]) : "50_stat.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcd_file, *cloud);
    std::cout << "Load Cloud Size ~ " << cloud->size() << std::endl;

    std::string calib_file = "../data/calib.txt";
    Eigen::Matrix3f K = PCM::KittiData::LoadKFromFile(calib_file);

    std::string img_file = "../data/000050.png";
    cv::Mat img = cv::imread(img_file, cv::IMREAD_GRAYSCALE);
    int img_rows = img.rows;
    int img_cols = img.cols;

    std::string dir = "LSR_" + Util::GetNameFromTime();
    Util::DirBuild(dir);

    std::vector<std::vector<pcl::PointXYZI>> edge_vecs;

    /// rotate the cloud around y azix
    const Eigen::Vector3f azix(0.0f, 1.0f, 0.0f);
    for (int i = 0; i < 360; i = i + 20)
    {
        float angle = i * M_PI / 180.0f;

        Eigen::Matrix3f rotation = CalRotation(azix, angle);
        Eigen::Vector3f translation(0.0f, 0.0f, 5.0f);
        Sophus::SE3f se(rotation, translation);

        std::cout << "Rotate Angle ~ " << i << std::endl;

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_trans = TransformCloud(cloud, se);
        PCM::PointCloudProject project;
        project.SetInput(cloud_trans);
        project.SetK(K);
        project.SetImageSize(img_rows, img_cols);
        project.Project2();
        project.EDLScale();
        std::vector<uchar> gray_shadow_vec = project.GetGrayShadowVec(true);

        PCM::LineSupportRegion lsr(img_rows, img_cols, gray_shadow_vec);
        lsr.Detect2();
        std::vector<PCM::Line2D> line_vec = lsr.GetLine();
        std::vector<std::vector<cv::Point2i>> line_region_vec = lsr.GetLineRegion();
//        std::vector<std::vector<pcl::PointXYZI>> edge_vec;
        std::vector<pcl::PointXYZI> edge_vec_i;

        /// traverse each line segment
        for (int j = 0; j < line_vec.size(); j++)
        {
            PCM::Line2D line = line_vec[j];
            std::vector<cv::Point2i> line_region = line_region_vec[j];

            PosVec line_3d_region;
            std::vector<float> tmp_depth_vec;
            for (const cv::Point2i &pt : line_region)
            {
                Eigen::Vector3f pos = project.ReProject(pt.y, pt.x);
                if (pos(2) == -1.0f)
                    continue;
                line_3d_region.push_back(pos);
                tmp_depth_vec.push_back(pos(2));
            }

            if (line_3d_region.size()<10)
            {
//                std::cout << "No Enough Positive Depth!" << std::endl << std::endl;
                continue;
            }

            float max_depth = *std::max_element(tmp_depth_vec.begin(), tmp_depth_vec.end());
            float min_depth = *std::min_element(tmp_depth_vec.begin(), tmp_depth_vec.end());
            float std_depth = StdDev(tmp_depth_vec);

            /// median filter
            std::vector<float> tmp_median_depth_vec = MedianFilter(tmp_depth_vec, 3);
            std::vector<int> scale_median_depth_vec;
            for (const float &tmp_depth : tmp_median_depth_vec)
            {
                int scale_median_depth = std::floor(10.0f + 100.0f * (tmp_depth - min_depth) / (max_depth - min_depth));
                scale_median_depth_vec.push_back(scale_median_depth);
            }

            /// convert the origin 3d line segment's depth to median depth
            PosVec median_line_3d_region;
            for (int k = 0; k < line_3d_region.size(); k++)
            {
                Eigen::Vector3f pos = line_3d_region[k];
                pos(2) = tmp_median_depth_vec[k];
                median_line_3d_region.push_back(pos);
            }
            std::vector<pcl::PointXYZI> tmp_edge_3d_vec = Search(cloud_trans, median_line_3d_region);
            edge_vec_i.insert(edge_vec_i.end(), tmp_edge_3d_vec.begin(), tmp_edge_3d_vec.end());
//            edge_vec.push_back(tmp_edge_3d_vec);

            /// show the scale median depth
            cv::Mat img_median_depth(120, tmp_median_depth_vec.size() + 2, CV_8UC1);
            img_median_depth.setTo(0);
            for (int k = 0; k < scale_median_depth_vec.size(); k++)
            {
                int tmp_row = scale_median_depth_vec[k];
                int tmp_col = k + 1;
                img_median_depth.at<uchar>(tmp_row, tmp_col) = 255;
            }
            cv::imshow("img_median_depth", img_median_depth);

            /// show the line segment
            cv::Mat img_lsr(img_rows, img_cols, CV_8UC1, gray_shadow_vec.data());
            cv::cvtColor(img_lsr, img_lsr, CV_GRAY2BGR);
            cv::line(img_lsr, line.s, line.e, cv::Scalar(0, 0, 255));
            for (int k = 0; k < line_region.size(); k++)
                cv::circle(img_lsr, line_region[k], 1, cv::Scalar(255, 0, 0));
            cv::imshow("img_lsr", img_lsr);
            cv::waitKey(10);
        }

        /// recover the pos
//        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_edge_i = TransformCloud(edge_vec, se.inverse());
        std::vector<pcl::PointXYZI> edge_trans_vec=TransformCloud(edge_vec_i,se.inverse());
        edge_vecs.push_back(edge_trans_vec);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_edge_i(new pcl::PointCloud<pcl::PointXYZI>);
        for(const pcl::PointXYZI &pt : edge_trans_vec)
            cloud_edge_i->push_back(pt);
        std::string pcd_file_i = dir + "/edge_angle" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileBinary(pcd_file_i, *cloud_edge_i);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_edge(new pcl::PointCloud<pcl::PointXYZI>);
    for(int i=0;i<edge_vecs.size();i++)
    {
        for(const pcl::PointXYZI &pt : edge_vecs[i])
        {
            pcl::PointXYZI p;
            p.x=pt.x;
            p.y=pt.y;
            p.z=pt.z;
            p.intensity=i;
            cloud_edge->push_back(p);
        }
    }

    std::string edge_file=dir+"/edge.pcd";
    pcl::io::savePCDFileBinary(edge_file,*cloud_edge);

}