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
    std::string pcd_file = (argc > 1) ? std::string(argv[1]) : "50_mls.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcd_file, *cloud);
    std::cout << "Load Cloud Size ~ " << cloud->size() << std::endl;

//    /// estimate normals
//    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimation;
//    normalEstimation.setInputCloud(cloud);
//    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
//    normalEstimation.setSearchMethod(kdtree);
//    normalEstimation.setKSearch(16);
//    normalEstimation.compute(*normals);
//
//    /// concatenate xyzi and normal
//    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointXYZINormal>);
//    pcl::concatenateFields(*cloud, *normals, *cloud_normal);
//    std::cout << "Compute Cloud Normal Done." << std::endl;

    std::string calib_file = "../data/calib.txt";
    Eigen::Matrix3f K = PCM::KittiData::LoadKFromFile(calib_file);

    std::string img_file = "../data/000050.png";
    cv::Mat img = cv::imread(img_file, cv::IMREAD_GRAYSCALE);
    int img_rows = img.rows;
    int img_cols = img.cols;

    /// project cloud to depth image
    PCM::PointCloudProject project;
    project.SetInput(cloud);
    project.SetK(K);
    project.SetImageSize(img_rows, img_cols);
    project.Project2();
    project.EDLScale();
    std::vector<uchar> gray_shadow_vec = project.GetGrayShadowVec(true);

    /// detect 2d line segment
    PCM::LineSupportRegion lsr(img_rows, img_cols, gray_shadow_vec);
    lsr.Detect2();
    std::cout << "Detect Line Support Done." << std::endl;

    /// draw the 2d line
    std::vector<pcl::PointXYZI> edge_3d_vec;
    std::vector<PCM::Line2D> line_vec=lsr.GetLine();
    std::vector<std::vector<cv::Point2i>> line_region_vec=lsr.GetLineRegion();
    for(int i=0;i<line_vec.size();i++)
    {
        PCM::Line2D line=line_vec[i];
        std::vector<cv::Point2i> line_region=line_region_vec[i];

        PosVec line_3d_region;
        std::vector<float> tmp_depth_vec;
        for(const cv::Point2i &pt : line_region)
        {
            Eigen::Vector3f pos=project.ReProject(pt.y,pt.x);
            if(pos(2)==-1.0f)
                continue;
            line_3d_region.push_back(pos);
            tmp_depth_vec.push_back(pos(2));
        }

        if(line_3d_region.empty())
        {
            std::cout<<"No Positive Depth!"<<std::endl<<std::endl;
            continue;
        }

        float max_depth=*std::max_element(tmp_depth_vec.begin(),tmp_depth_vec.end());
        float min_depth=*std::min_element(tmp_depth_vec.begin(),tmp_depth_vec.end());
        float std_depth=StdDev(tmp_depth_vec);

//        std::cout<<"iter ~ "<<i<<std::endl;
//        std::cout<<"line size ~ "<<line_region.size()<<std::endl;
//        std::cout<<"depth size ~ "<<tmp_depth_vec.size()<<std::endl;
//        std::cout<<"min_depth ~ "<<min_depth<<std::endl;
//        std::cout<<"max_depth ~ "<<max_depth<<std::endl;
//        std::cout<<"std_depth ~ "<<std_depth<<std::endl<<std::endl;
//
//        std::vector<int> scale_depth_vec;
//        for(const float &tmp_depth : tmp_depth_vec)
//        {
//            int scale_depth=std::floor(10.0f+100.0f*(tmp_depth-min_depth)/(max_depth-min_depth));
//            scale_depth_vec.push_back(scale_depth);
//        }
//        cv::Mat img_depth(120,scale_depth_vec.size()+2,CV_8UC1);
//        img_depth.setTo(0);
//        for(int k=0;k<scale_depth_vec.size();k++)
//        {
//            int tmp_row=scale_depth_vec[k];
//            int tmp_col=k+1;
//            img_depth.at<uchar>(tmp_row,tmp_col)=255;
//        }
//        cv::imshow("img_depth",img_depth);

        /// median filter
        std::vector<float> tmp_median_depth_vec=MedianFilter(tmp_depth_vec,3);
        std::vector<int> scale_median_depth_vec;
        for(const float &tmp_depth : tmp_median_depth_vec)
        {
            int scale_median_depth=std::floor(10.0f+100.0f*(tmp_depth-min_depth)/(max_depth-min_depth));
            scale_median_depth_vec.push_back(scale_median_depth);
        }

        cv::Mat img_median_depth(120,tmp_median_depth_vec.size()+2,CV_8UC1);
        img_median_depth.setTo(0);
        for(int k=0;k<scale_median_depth_vec.size();k++)
        {
            int tmp_row=scale_median_depth_vec[k];
            int tmp_col=k+1;
            img_median_depth.at<uchar>(tmp_row,tmp_col)=255;
        }
        cv::imshow("img_median_depth",img_median_depth);

        cv::Mat img_lsr(img_rows,img_cols,CV_8UC1,gray_shadow_vec.data());
        cv::cvtColor(img_lsr,img_lsr,CV_GRAY2BGR);
        cv::line(img_lsr,line.s,line.e,cv::Scalar(0,0,255));
        for(int j=0;j<line_region.size();j++)
            cv::circle(img_lsr,line_region[j],1,cv::Scalar(255,0,0));
        cv::imshow("img_lsr",img_lsr);

        PosVec median_line_3d_region;
        for(int k=0;k<line_3d_region.size();k++)
        {
            Eigen::Vector3f pos=line_3d_region[k];
            pos(2)=tmp_median_depth_vec[k];
            median_line_3d_region.push_back(pos);
        }

        std::vector<pcl::PointXYZI> tmp_edge_3d_vec=Search(cloud,median_line_3d_region);
        edge_3d_vec.insert(edge_3d_vec.end(),tmp_edge_3d_vec.begin(),tmp_edge_3d_vec.end());

        cv::waitKey(1000);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_edge(new pcl::PointCloud<pcl::PointXYZI>);
    for(const pcl::PointXYZI &pt : edge_3d_vec)
        cloud_edge->push_back(pt);
    pcl::io::savePCDFileBinary("edge.pcd",*cloud_edge);

    return 1;

}