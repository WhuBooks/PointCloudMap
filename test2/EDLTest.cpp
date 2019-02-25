//
// Created by books on 2018/4/10.
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


std::vector<float> ScaleDepth(const std::vector<float> &m_depth_vec,int m_rows,int m_cols,int scale)
{
    int scale_rows=std::floor(m_rows/scale);
    int scale_cols=std::floor(m_cols/scale);

    std::vector<float> scale_depth_vec(scale_rows*scale_cols,0.0f);
    for(int i=0;i<scale_rows;i++)
    {
        for(int j=0;j<scale_cols;j++)
        {
            float sum_depth=0.0f;
            for(int ii=0;ii<scale;ii++)
            {
                for(int jj=0;jj<scale;jj++)
                {
                    int index_iijj=(i*scale+ii)*m_cols+scale*j+jj;
                    sum_depth+=m_depth_vec[index_iijj];
                }
            }
            int index_ij=i*scale_cols+j;
            scale_depth_vec[index_ij]=sum_depth/(scale*scale);
        }
    }
    return scale_depth_vec;
}

std::vector<float> ScaleEDL(const std::vector<float> &scale_depth_vec,int scale_rows,int scale_cols)
{

    std::vector<float> scale_shadow_vec(scale_depth_vec.size(),0.0f);
    const float shadow_factor=100.0f;
    for(int i=0;i<scale_rows;i++)
    {
        for (int j = 0; j < scale_cols; j++)
        {
            int index_ij=i*scale_cols+j;
            float val_ij=scale_depth_vec[index_ij];
            float sum_dis=0.0f;
            for(int ii=std::max(0,i-2);ii<=std::min(scale_rows-1,i+2);ii++)
            {
                for(int jj=std::max(0,j-2);jj<=std::min(scale_cols-1,j+2);jj++)
                {
                    if(ii==i && jj==j)
                        continue;

                    int index_iijj=ii*scale_cols+jj;
                    float val_iijj=scale_depth_vec[index_iijj];

                    float tmp_euc_dis=(float)std::sqrt((i-ii)*(i-ii)+(j-jj)*(j-jj));
                    float tmp_dis=(val_ij-val_iijj)/tmp_euc_dis;

                    sum_dis+=std::max(tmp_dis,0.0f);
                }
            }
            float shadow_val=std::exp(-shadow_factor*sum_dis);
            scale_shadow_vec[index_ij]=shadow_val;
        }
    }

    float max_shadow_val=*std::max_element(scale_shadow_vec.begin(),scale_shadow_vec.end());
    float min_shadow_val=*std::min_element(scale_shadow_vec.begin(),scale_shadow_vec.end());

    for(float &val : scale_shadow_vec)
        val=(val-min_shadow_val)/(max_shadow_val-min_shadow_val);

    return scale_shadow_vec;
}

float CalMedian(std::vector<float> vec)
{
    std::sort(vec.begin(),vec.end());
    int num=vec.size();

    float val=0.0f;
    if(num%2==0)
        val= 0.5f*(vec[num/2-1]+vec[num/2]);
    else
        val=vec[(num-1)/2];
    return val;

}

std::vector<float> MedianFilter(const std::vector<float> &vec,int rows,int cols,int N)
{
    std::vector<float> result(vec.size(),0.0f);
    int half_n=std::floor(N/2);
    for(int i=0;i<rows;i++)
    {
        for(int j=0;j<cols;j++)
        {
            int index_ij=i*cols+j;
            std::vector<float> tmp;
            for(int ii=std::max(0,i-half_n);ii<=std::min(rows-1,i+half_n);ii++)
            {
                for(int jj=std::max(0,j-half_n);jj<=std::min(cols-1,j+half_n);jj++)
                {
                    int index_iijj=ii*cols+jj;
                    tmp.push_back(vec[index_iijj]);
                }
            }
            result[index_ij]=CalMedian(tmp);
        }
    }
    return result;
}


int main()
{
    std::string pcd_file="50_stat.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcd_file,*cloud);

    std::string calib_file="../data/calib.txt";
    Eigen::Matrix3f K=PCM::KittiData::LoadKFromFile(calib_file);

    std::string img_file="000050.png";
    cv::Mat img=cv::imread(img_file,cv::IMREAD_GRAYSCALE);
    int img_rows=img.rows;
    int img_cols=img.cols;

    PCM::PointCloudProject project;
    project.SetInput(cloud);
    project.SetK(K);
    project.SetImageSize(img_rows,img_cols);
    project.Project2();
    project.EDLScale();

//    std::vector<float> depth_vec=project.GetDepthVec();
//    cv::Mat img_depth(img_rows,img_cols,CV_32FC1,depth_vec.data());
//    cv::imshow("img_depth",img_depth);

    std::vector<float> shadow_vec=project.GetShadowVec();
    cv::Mat img_shadow(img_rows,img_cols,CV_32FC1,shadow_vec.data());
    cv::imshow("img_shadow",img_shadow);

    std::vector<float> process_shadow_vec=MedianFilter(shadow_vec,img_rows,img_cols,3);
    std::vector<uchar> gray_shadow_vec=PCM::PointCloudProject::Float2Gray(process_shadow_vec);
    cv::Mat img_gray_shadow(img_rows,img_cols,CV_8UC1,gray_shadow_vec.data());
    cv::imshow("img_gray_shadow",img_gray_shadow);


//    cv::Mat img_shadow_process(img_shadow);

    /// mean blur
//    const int mean_blur_size=5;
//    cv::blur(img_shadow_process,img_shadow_process,cv::Size(mean_blur_size,mean_blur_size));

    /// median blur
//    const int median_blur_size=3;
//    cv::medianBlur(img_shadow_process,img_shadow_process,median_blur_size);

//    /// gaussian blur
//    const int gaussian_blur_size=5;
//    cv::GaussianBlur(img_shadow_process,img_shadow_process,cv::Size(gaussian_blur_size,gaussian_blur_size),0,0);

    /// bilateral
    //cv::bilateralFilter(img_shadow,img_shadow_process,3,15,15);

    /// sobel
//    cv::Mat img_sobel_x(img_rows,img_cols,CV_32FC1);
//    cv::Mat img_sobel_y(img_rows,img_cols,CV_32FC1);
//
//    cv::Sobel(img_shadow_process,img_sobel_x,1,0,3);
//    cv::Sobel(img_shadow_process,img_sobel_y,0,1,3);
//    img_shadow_process.setTo(0);
//    cv::addWeighted(img_sobel_x,0.5,img_sobel_y,0.5,0,img_shadow_process);

    /// laplican
//    cv::Laplacian(img_shadow_process,img_shadow_process,CV_32FC1,3);

//    cv::imshow("img_shadow_process",img_shadow_process);

//    cv::Mat img_gray_shadow(img_rows,img_cols,CV_8UC1);
//    for(int i=0;i<img_rows;i++)
//    {
//        float * row_shadow_process=img_shadow_process.ptr<float>(i);
//        uchar * row_gray_shadow=img_gray_shadow.ptr<uchar>(i);
//        for(int j=0;j<img_cols;j++)
//        {
//            row_gray_shadow[j]=cv::saturate_cast<uchar>(row_shadow_process[j]*255);
//        }
//    }
//    cv::imshow("img_gray_shadow",img_gray_shadow);

//    cv::Mat img_gray_bilateral(img_rows,img_cols,CV_8UC1);
//    img_gray_bilateral.setTo(0);
//    cv::bilateralFilter(img_gray_shadow,img_gray_bilateral,-1,15,15);
//    cv::imshow("img_gray_bilateral",img_gray_bilateral);

//    std::vector<float> shadow_vec=ScaleEDL(depth_vec,img_rows,img_cols);
//    cv::Mat img_shadow(img_rows,img_cols,CV_32FC1,shadow_vec.data());
//    cv::imshow("img_shadow",img_shadow);
//
//    /// half
//    int half_rows=std::floor(img_rows/2);
//    int half_cols=std::floor(img_cols/2);
//
//    std::vector<float> half_depth=ScaleDepth(depth_vec,img_rows,img_cols,2);
//    cv::Mat img_half_depth(half_rows,half_cols,CV_32FC1,half_depth.data());
//    cv::imshow("img_half_depth",img_half_depth);
//
//    std::vector<float> half_shadow=ScaleEDL(half_depth,half_rows,half_cols);
//    cv::Mat img_half_shadow(half_rows,half_cols,CV_32FC1,half_shadow.data());
//    cv::imshow("img_half_shadow",img_half_shadow);
//
//    /// quarter
//    int quarter_rows=std::floor(img_rows/4);
//    int quarter_cols=std::floor(img_cols/4);
//
//    std::vector<float> quarter_depth=ScaleDepth(depth_vec,img_rows,img_cols,4);
//    cv::Mat img_quarter_depth(quarter_rows,quarter_cols,CV_32FC1,quarter_depth.data());
//    cv::imshow("img_quarter_depth",img_quarter_depth);
//
//    std::vector<float> quarter_shadow=ScaleEDL(quarter_depth,quarter_rows,quarter_cols);
//    cv::Mat img_quarter_shadow(quarter_rows,quarter_cols,CV_32FC1,quarter_shadow.data());
//    cv::imshow("img_quarter_shadow",img_quarter_shadow);
//
//    std::vector<float> merge_depth(img_rows*img_cols,0.0f);
//    std::vector<float> merge_shadow(img_rows*img_cols,0.0f);
//    for(int i=0;i<img_rows;i++)
//    {
//        for(int j=0;j<img_cols;j++)
//        {
//            int index=i*img_cols+j;
//            int index_half=std::floor(i/2)*half_cols+std::floor(j/2);
//            int index_quarter=std::floor(i/4)*quarter_cols+std::floor(j/4);
//
//            float tmp_depth=(4.0f*depth_vec[index]+2.0f*half_depth[index_half]+quarter_depth[index_quarter])/7.0f;
//            merge_depth[index]=tmp_depth;
//
//            float val=(4.0f*shadow_vec[index]+2.0f*half_shadow[index_half]+quarter_shadow[index_quarter])/7.0f;
//            merge_shadow[index]=val;
//        }
//    }
//
//    cv::Mat img_merge_depth(img_rows,img_cols,CV_32FC1,merge_depth.data());
//    cv::imshow("img_merge_depth",img_merge_depth);
//
//    cv::Mat img_merge_shadow(img_rows,img_cols,CV_32FC1,merge_shadow.data());
//    cv::imshow("img_merge_shadow",img_merge_shadow);

    cv::waitKey(0);
    return 1;
}