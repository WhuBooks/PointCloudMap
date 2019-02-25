//
// Created by whubooks on 18-4-12.
// Paper : Line segment extraction for large scale unorganized point clouds
//

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include <pcl/io/pcd_io.h>

#include <KittiData.h>
#include <PointCloudProject.h>
#include <Util.h>

std::vector<cv::Point2d> LineRectangle(const cv::Point2d &pt_s,const cv::Point2d &pt_e,double width)
{
    double x1=pt_s.x,y1=pt_s.y;
    double x2=pt_e.x,y2=pt_e.y;

    std::vector<cv::Point2d> result;
    if(x1==x2)
    {
        result.push_back(cv::Point2d(x1-width/2.0,y1));
        result.push_back(cv::Point2d(x1+width/2.0,y1));
        result.push_back(cv::Point2d(x2+width/2.0,y2));
        result.push_back(cv::Point2d(x2-width/2.0,y2));
    }
    else
    {
        double val=std::sqrt(width*width*(x1-x2)*(x1-x2)/(2*(y1-y2)*(y1-y2)+2*(x1-x2)*(x1-x2)));

        double tmp_y1=y1-val;
        double tmp_x1=(tmp_y1-y1)*(y2-y1)/(x2-x1)+x1;

        double tmp_y2=y1+val;
        double tmp_x2=(tmp_y2-y1)*(y2-y1)/(x2-x1)+x1;

        double tmp_y3=y2+val;
        double tmp_x3=(tmp_y3-y2)*(y1-y2)/(x1-x2)+x2;

        double tmp_y4=y2-val;
        double tmp_x4=(tmp_y4-y2)*(y1-y2)/(x1-x2)+x2;

        result.push_back(cv::Point2d(tmp_x1,tmp_y1));
        result.push_back(cv::Point2d(tmp_x2,tmp_y2));
        result.push_back(cv::Point2d(tmp_x3,tmp_y3));
        result.push_back(cv::Point2d(tmp_x4,tmp_y4));

    }
    return result;
}

bool Contain(const std::vector<cv::Point2d> &rect,const cv::Point2i &pos)
{
    cv::Point2d lt_rt(rect[0].x-rect[1].x,rect[0].y-rect[1].y);
    cv::Point2d lt_pos(rect[0].x-pos.x,rect[0].y-pos.y);
    cv::Point2d rb_lb(rect[2].x-rect[3].x,rect[2].y-rect[3].y);
    cv::Point2d rb_pos(rect[2].x-pos.x,rect[2].y-pos.y);

    cv::Point2d rt_rb(rect[1].x-rect[2].x,rect[1].y-rect[2].y);
    cv::Point2d rt_pos(rect[1].x-pos.x,rect[1].y-pos.y);
    cv::Point2d lb_lt(rect[3].x-rect[0].x,rect[3].y-rect[0].y);
    cv::Point2d lb_pos(rect[3].x-pos.x,rect[3].y-pos.y);

    return (lt_rt.cross(lt_pos)*rb_lb.cross(rb_pos))>=0 && (rt_rb.cross(rt_pos)*lb_lt.cross(lb_pos))>=0;
}

int main(int argc,char **argv)
{
    std::string pcd_file=(argc>1)?std::string(argv[1]):"50_mls.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcd_file,*cloud);
    std::cout<<"Load Cloud Size ~ "<<cloud->size()<<std::endl;

    std::string calib_file="../data/calib.txt";
    Eigen::Matrix3f K=PCM::KittiData::LoadKFromFile(calib_file);

    std::string img_file="../data/000050.png";
    cv::Mat img=cv::imread(img_file,cv::IMREAD_GRAYSCALE);
    int img_rows=img.rows;
    int img_cols=img.cols;

    PCM::PointCloudProject project;
    project.SetInput(cloud);
    project.SetK(K);
    project.SetImageSize(img_rows,img_cols);
    project.Project2();
    project.EDLScale();

    std::vector<float> depth_vec=project.GetDepthVec();
    std::vector<uchar> gray_shadow_vec=project.GetGrayShadowVec(true);
    cv::Mat img_gray_shadow(img_rows,img_cols,CV_8UC1,gray_shadow_vec.data());
    cv::imshow("img_gray_shadow",img_gray_shadow);

    cv::Ptr<cv::LineSegmentDetector> lsd=cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
    std::vector<cv::Vec4d> lines;
    std::vector<double> widths;
    lsd->detect(img_gray_shadow,lines,widths);

    cv::Mat img_lsd_lines(img_gray_shadow);
    cv::cvtColor(img_lsd_lines,img_lsd_lines,CV_GRAY2BGR);
    std::vector<std::vector<cv::Point2d>> rects;
    int num_negative=0;
    for(int i=0;i<lines.size();i++)
    {
        cv::Point2d pt_s(lines[i][0],lines[i][1]);
        cv::Point2d pt_e(lines[i][2],lines[i][3]);
        double width=widths[i];

        double length=std::sqrt((pt_s.x-pt_e.x)*(pt_s.x-pt_e.x)+(pt_s.y-pt_e.y)*(pt_s.y-pt_e.y));
        if(length*width<80)
        {
            num_negative++;
            continue;
        }

        cv::line(img_lsd_lines, pt_s, pt_e, cv::Scalar(255, 0, 0));

        std::vector<cv::Point2d> vec=LineRectangle(pt_s,pt_e,width);
        rects.push_back(vec);

        cv::line(img_lsd_lines,vec[0],vec[1],cv::Scalar(0,0,255));
        cv::line(img_lsd_lines,vec[1],vec[2],cv::Scalar(0,0,255));
        cv::line(img_lsd_lines,vec[2],vec[3],cv::Scalar(0,0,255));
        cv::line(img_lsd_lines,vec[3],vec[0],cv::Scalar(0,0,255));
    }
    cv::imshow("img_lsd_lines",img_lsd_lines);
    std::cout<<"2D Line Support Region Size ~ "<<rects.size()<<std::endl;
    std::cout<<"Num Negative ~ "<<num_negative<<std::endl;

    /// 3d line support region
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    flann->setInputCloud(cloud);
    const float radius=0.05f;
    std::vector<int> lsr_region_index;
    for(int i=0;i<img_rows;i++)
    {
        for(int j=0;j<img_cols;j++)
        {
            cv::Point2i pos_2d(j,i);
            for(const std::vector<cv::Point2d> &rect : rects)
            {
                if(Contain(rect,pos_2d))
                {
                    Eigen::Vector3f pos_3d=project.ReProject(i,j);
                    pcl::PointXYZI pt;
                    pt.x=pos_3d(0);
                    pt.y=pos_3d(1);
                    pt.z=pos_3d(2);
                    pt.intensity=0.5;

                    std::vector<int> vindices;
                    std::vector<float> vdistance;
                    flann->radiusSearch(pt,radius,vindices,vdistance);

                    lsr_region_index.insert(lsr_region_index.end(),vindices.begin(),vindices.end());

                    break;
                }
            }
        }
    }

    std::sort(lsr_region_index.begin(),lsr_region_index.end());
    int last_index=-100000;
    std::vector<int> tmp_lsr_region_index;
    for(const int &index : lsr_region_index)
    {
        if(index!=last_index)
            tmp_lsr_region_index.push_back(index);
        last_index=index;
    }
    lsr_region_index.swap(tmp_lsr_region_index);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_lsr(new pcl::PointCloud<pcl::PointXYZI>);
    for(const int &index : lsr_region_index)
        cloud_lsr->push_back(cloud->points[index]);

    std::string lsr_file="LSR.pcd";
    pcl::io::savePCDFileBinary(lsr_file,*cloud_lsr);



    cv::waitKey(0);
    return 1;
}