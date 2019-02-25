//
// Created by whubooks on 18-4-18.
// Paper : Line segment extraction for large scale unorganized point clouds
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

typedef std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> PosVec;

struct Pt
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3f pos;
    Eigen::Vector3f nor;
};

std::vector<Pt> Search(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,const PosVec &vec)
{
    pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZINormal>);
    flann->setInputCloud(cloud);

    const float radius=0.05f;
    std::vector<int> indices;
    for(const Eigen::Vector3f &pos : vec)
    {
        pcl::PointXYZINormal pt;
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

    std::vector<Pt> result;
    for(const int &index : indices)
    {
        pcl::PointXYZINormal pt=cloud->points[index];

        if(std::isnan(pt.normal_x) || std::isnan(pt.normal_y) || std::isnan(pt.normal_z))
            continue;

        Pt tmp;
        tmp.pos=Eigen::Vector3f(pt.x,pt.y,pt.z);
        tmp.nor=Eigen::Vector3f(pt.normal_x,pt.normal_y,pt.normal_z);
        tmp.nor.normalize();
        result.push_back(tmp);
    }
    return result;
}

Eigen::Vector3f CalNormal(const std::vector<Pt> &pts)
{
    Eigen::Vector3f centroid(0.0f,0.0f,0.0f);
    for(const Pt &pt : pts)
    {
        centroid += pt.pos;
    }
    centroid/=pts.size();

    float sum_weight=0.0f;
    Eigen::Vector3f normal(0.0f,0.0f,0.0f);
    for(const Pt &pt : pts)
    {
        float norm=(pt.pos-centroid).norm();
        float weight=(norm<0.1f)?0.1f:0.1f/norm;

        normal+=(weight*pt.nor);
        sum_weight+=weight;

    }
    normal/=sum_weight;
    normal.normalize();

    return normal;
}

float IncludeAngle(const Eigen::Vector3f &vec1,const Eigen::Vector3f &vec2)
{
    float product=vec1.transpose()*vec2;
    float angle=std::acos(product/(vec1.norm()*vec2.norm()));
}

/// calculate the rotate matrix from vec1 to vec2
Eigen::Matrix3f CalRotate(const Eigen::Vector3f &vec1,const Eigen::Vector3f &vec2)
{
    float product = vec1.transpose() * vec2;
    float angle = std::acos(product / (vec1.norm() * vec2.norm()));

    float a1 = vec1(0), a2 = vec1(1), a3 = vec1(2);
    float b1 = vec2(0), b2 = vec2(1), b3 = vec2(2);
    Eigen::Vector3f azix(a2 * b3 - a3 * b2, a3 * b1 - a1 * b3, a1 * b2 - a2 * b1);
    azix.normalize();
    float w1 = azix(0), w2 = azix(1), w3 = azix(2);

    Eigen::Matrix3f skew_symmetry;
    skew_symmetry << 0.0f, -w3, w2, w3, 0.0f, -w1, -w2, w1, 0.0f;

    Eigen::Matrix3f rotate=Eigen::Matrix3f::Identity()+skew_symmetry*std::sin(angle)+skew_symmetry*skew_symmetry*(1.0f-std::cos(angle));
    return rotate;
}

int main(int argc,char **argv)
{
    std::string pcd_file = (argc > 1) ? std::string(argv[1]) : "50_mls.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcd_file, *cloud);
    std::cout << "Load Cloud Size ~ " << cloud->size() << std::endl;

    /// estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI,pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.setRadiusSearch(0.1);
    normalEstimation.compute(*normals);

    /// concatenate xyzi and normal
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::concatenateFields(*cloud,*normals,*cloud_normal);
    std::cout<<"Compute Cloud Normal Done."<<std::endl;

    std::string calib_file = "../data/calib.txt";
    Eigen::Matrix3f K = PCM::KittiData::LoadKFromFile(calib_file);

    std::string img_file = "../data/000050.png";
    cv::Mat img = cv::imread(img_file, cv::IMREAD_GRAYSCALE);
    int img_rows = img.rows;
    int img_cols = img.cols;

    PCM::PointCloudProject project;
    project.SetInput(cloud);
    project.SetK(K);
    project.SetImageSize(img_rows, img_cols);
    project.Project2();
    project.EDLScale();
    std::vector<uchar> gray_shadow_vec=project.GetGrayShadowVec(true);

    PCM::LineSupportRegion lsr(img_rows,img_cols,gray_shadow_vec);
    lsr.Detect();
    std::cout<<"Detect Line Support Done."<<std::endl;

    std::vector<PCM::Envelop2D> env_vec=lsr.GetEnvelop();
    std::vector<PCM::Envelop2D> left_env_vec=lsr.GetLeftEnvelop();
    std::vector<PCM::Envelop2D> right_env_vec=lsr.GetRightEnvelop();
    std::vector<std::vector<cv::Point2i>> left_region_vec=lsr.GetLeftRegion();
    std::vector<std::vector<cv::Point2i>> right_region_vec=lsr.GetRightRegion();

    /// reproject camera point to cloud point, and search in the origin cloud
    for(int i=0;i<left_region_vec.size();i++)
    {
        std::vector<cv::Point2i> left_region=left_region_vec[i];
        std::vector<cv::Point2i> right_region=right_region_vec[i];

        PosVec left_pos_vec,right_pos_vec;
        for(const cv::Point2i &left_tmp : left_region)
        {
            Eigen::Vector3f pos=project.ReProject(left_tmp.y,left_tmp.x);
            if(pos(2)!=-1.0f)
                left_pos_vec.push_back(pos);
        }
        for(const cv::Point2i &right_tmp : right_region)
        {
            Eigen::Vector3f pos=project.ReProject(right_tmp.y,right_tmp.x);
            if(pos(2)!=-1.0f)
                right_pos_vec.push_back(pos);
        }

        /// convert the 2d line support region to 3d line support region
        std::vector<Pt> left_pts=Search(cloud_normal,left_pos_vec);
        std::vector<Pt> right_pts=Search(cloud_normal,right_pos_vec);
        if (left_pts.empty() || right_pts.empty())
            continue;

        /// calculate the two half plananrs' normal
        Eigen::Vector3f left_normal = CalNormal(left_pts);
        Eigen::Vector3f right_normal = CalNormal(right_pts);

        /// ignore the planars which are nearly parallel
        float angle = IncludeAngle(left_normal, right_normal);
        if (angle > 0.9 * M_PI || angle < 0.1 * M_PI)
            continue;

        /// compute the project direction
        Eigen::Vector3f direction = left_normal.cross(right_normal);
        direction.normalize();

        float val1 = left_normal.transpose() * direction;
        float val2 = right_normal.transpose() * direction;

        /// calculate the rotation matrix from project direction to y azix
        Eigen::Vector3f direction_y(0.0f, 1.0f, 0.0f);
        Eigen::Matrix3f rotate = CalRotate(direction, direction_y);
        Sophus::SE3f se3(rotate, Eigen::Vector3f(0.0f, 0.0f, 0.0f));

        Eigen::Vector3f rotate_val1=rotate*direction;
        Eigen::Vector3f rotate_val2=se3*direction;

        /// rotate the planar so the projection be the y azix
        std::vector<cv::Point2f> rotate_left_vec, rotate_right_vec;
        float min_x = 1000000000000.0f, min_z = 10000000000.0f;
        float max_x = -1000000000000.0f, max_z = -10000000000.0f;
        for (const Pt &pt : left_pts)
        {
            Eigen::Vector3f rotate_pos = se3 * pt.pos;
            float rotate_x = rotate_pos(0);
            float rotate_z = rotate_pos(2);

            min_x = (min_x < rotate_x) ? min_x : rotate_x;
            min_z = (min_z < rotate_z) ? min_z : rotate_z;
            max_x = (max_x > rotate_x) ? max_x : rotate_x;
            max_z = (max_z > rotate_z) ? max_z : rotate_z;
            rotate_left_vec.push_back(cv::Point2f(rotate_x, rotate_z));
        }
        for (const Pt &pt : right_pts)
        {
            Eigen::Vector3f rotate_pos = se3 * pt.pos;
            float rotate_x = rotate_pos(0);
            float rotate_z = rotate_pos(2);

            min_x = (min_x < rotate_x) ? min_x : rotate_x;
            min_z = (min_z < rotate_z) ? min_z : rotate_z;
            max_x = (max_x > rotate_x) ? max_x : rotate_x;
            max_z = (max_z > rotate_z) ? max_z : rotate_z;
            rotate_right_vec.push_back(cv::Point2f(rotate_x, rotate_z));
        }

        /// optimize the 2d pos to 10~110
        std::vector<cv::Point2f> trans_left_vec, trans_right_vec;
        for (const cv::Point2f &pt : rotate_left_vec)
        {
            cv::Point2f tmp(100 * (pt.x - min_x) / (max_x - min_x) + 10.0f,
                            100 * (pt.y - min_z) / (max_z - min_z) + 10.0f);
            trans_left_vec.push_back(tmp);
        }

        for (const cv::Point2f &pt : rotate_right_vec)
        {
            cv::Point2f tmp(100 * (pt.x - min_x) / (max_x - min_x) + 10.0f,
                            100 * (pt.y - min_z) / (max_z - min_z) + 10.0f);
            trans_right_vec.push_back(tmp);
        }

        /// show
        int tmp_rows = 120;
        int tmp_cols = 120;
        cv::Mat img_tmp(tmp_rows, tmp_cols, CV_8UC3);
        img_tmp.setTo(cv::Scalar(0,0,0));
        for (const cv::Point2f &pt : trans_left_vec)
            cv::circle(img_tmp, pt, 1, cv::Scalar(255,0,0));
        for (const cv::Point2f &pt : trans_right_vec)
            cv::circle(img_tmp, pt, 1, cv::Scalar(0,0,255));
        cv::imshow("project",img_tmp);

        cv::Mat img_lsr(img_rows,img_cols,CV_8UC1,gray_shadow_vec.data());
        cv::cvtColor(img_lsr,img_lsr,CV_GRAY2BGR);
        PCM::Envelop2D env=env_vec[i];
        PCM::Envelop2D left_env=left_env_vec[i];
        cv::line(img_lsr,env.lt,env.rt,cv::Scalar(0,0,255));
        cv::line(img_lsr,env.rt,env.rb,cv::Scalar(0,0,255));
        cv::line(img_lsr,env.rb,env.lb,cv::Scalar(0,0,255));
        cv::line(img_lsr,env.lb,env.lt,cv::Scalar(0,0,255));
        cv::line(img_lsr,left_env.rt,left_env.rb,cv::Scalar(255,0,0));
        cv::imshow("img_lsr",img_lsr);

        std::cout<<"***************************************"<<std::endl;
        std::cout << "Left Planar Point Size ~ " << left_pts.size() << std::endl;
        std::cout << "Right Planar Point Size ~ " << right_pts.size() << std::endl;
        std::cout << "Left Planar Normal ~ " << left_normal.transpose() << std::endl;
        std::cout << "Right Planar Normal ~ " << right_normal.transpose() << std::endl;
        std::cout << "Project Direction ~ " << direction.transpose() << std::endl<<std::endl;

        cv::waitKey(0);
    }

//    std::string dir="LSR"+Util::GetNameFromTime();
//    Util::DirBuild(dir);
//
//    std::string lsr_file=dir+"/"+"LSR.pcd";
//    WriteLsrResult(left_pts_vec,right_pts_vec,lsr_file);

    return 1;
}