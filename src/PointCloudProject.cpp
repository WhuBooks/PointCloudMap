//
// Created by books on 2018/3/25.
//

#include "PointCloudProject.h"
#include <pcl/io/pcd_io.h>

namespace PCM{

    const float Shadow_Factor=75.0f;

    float CalMean(const std::vector<float> &vec)
    {
        return std::accumulate(vec.begin(),vec.end(),0.0f)/vec.size();
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

    void MedianFilter(std::vector<float> &vec,int rows,int cols,int N)
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
        vec.swap(result);
    }
    void MeanFilter(std::vector<float> &vec,int rows,int cols,int N)
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
                result[index_ij]=CalMean(tmp);
            }
        }
        vec.swap(result);
    }

    std::vector<float> ScaleDepth(const std::vector<float> &depth_vec,int rows,int cols,int scale)
    {
        int scale_rows=std::floor(rows/scale);
        int scale_cols=std::floor(cols/scale);

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
                        int index_iijj=(i*scale+ii)*cols+scale*j+jj;
                        sum_depth+=depth_vec[index_iijj];
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
                float shadow_val=std::exp(-Shadow_Factor*sum_dis);
                scale_shadow_vec[index_ij]=shadow_val;
            }
        }

        float max_shadow_val=*std::max_element(scale_shadow_vec.begin(),scale_shadow_vec.end());
        float min_shadow_val=*std::min_element(scale_shadow_vec.begin(),scale_shadow_vec.end());

        for(float &val : scale_shadow_vec)
            val=(val-min_shadow_val)/(max_shadow_val-min_shadow_val);

        return scale_shadow_vec;
    }

    PointCloudProject::PointCloudProject():m_cloud(new pcl::PointCloud<pcl::PointXYZINormal>)
    {

    }

    PointCloudProject::~PointCloudProject()
    {

    }

    /// set input cloud
    void PointCloudProject::SetInput(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        m_cloud->clear();

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZI>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZI>);
        pcl::NormalEstimation<pcl::PointXYZI,pcl::Normal> normalEstimation;
        normalEstimation.setInputCloud(cloud);
        normalEstimation.setSearchMethod(kdTree);
        normalEstimation.setKSearch(16);
        normalEstimation.compute(*normals);

        pcl::concatenateFields(*cloud,*normals,*m_cloud);
    }

    /// set input cloud
    void PointCloudProject::SetInput(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
    {
        m_cloud->clear();
        pcl::copyPointCloud(*cloud,*m_cloud);
    }

    /// set camera instrinsic
    void PointCloudProject::SetK(const Eigen::Matrix3f &K)
    {
        m_K=Eigen::Matrix3f(K);
//        std::cout<<"K ~ "<<m_K<<std::endl;
    }

    /// set image width and height
    void PointCloudProject::SetImageSize(const int &rows, const int &cols)
    {
        m_rows=rows;
        m_cols=cols;
    }

    std::vector<float> PointCloudProject::GetDepthVec() const
    {
        return m_depth_vec;
    }

    std::vector<uchar> PointCloudProject::GetGrayDepthVec() const
    {
        std::vector<uchar> depth_gray_vec;
        for(const float &val : m_depth_vec)
            depth_gray_vec.push_back(cv::saturate_cast<uchar>(255*val));
        return depth_gray_vec;
    }

    std::vector<cv::Vec3f> PointCloudProject::GetNormalVec() const
    {
        return m_normal_vec;
    }

    std::vector<cv::Vec3b> PointCloudProject::GetRGBNormalVec() const
    {
        std::vector<cv::Vec3b> normal_gray_vec;
        for(const cv::Vec3f &normal : m_normal_vec)
        {
            uchar r=cv::saturate_cast<uchar>(normal[0]*255);
            uchar g=cv::saturate_cast<uchar>(normal[1]*255);
            uchar b=cv::saturate_cast<uchar>(normal[2]*255);
            normal_gray_vec.push_back(cv::Vec3b(r,g,b));
        }

        return normal_gray_vec;
    }

    std::vector<Feature> PointCloudProject::GetFeature() const
    {
        std::vector<Feature> result;
        for(int i=0;i<m_rows;i++)
        {
            for(int j=0;j<m_cols;j++)
            {
                Feature feature;
                feature.u=j;
                feature.v=i;
                int index=i*m_cols+j;
                feature.u_d=m_depth_vec[index]*j;
                feature.v_d=m_depth_vec[index]*i;
                feature.normal_x=m_normal_vec[index][0];
                feature.normal_y=m_normal_vec[index][1];
                feature.normal_z=m_normal_vec[index][2];
                result.push_back(feature);
            }
        }
        return result;
    }

    std::vector<float> PointCloudProject::GetShadowVec() const
    {
        return m_shadows_vec;
    }

    std::vector<uchar> PointCloudProject::GetGrayShadowVec(bool median_filter) const
    {
        std::vector<float> tmp_shadow_vec;
        tmp_shadow_vec.assign(m_shadows_vec.begin(),m_shadows_vec.end());

        if(median_filter)
        {

            MedianFilter(tmp_shadow_vec,m_rows,m_cols,3);
            MeanFilter(tmp_shadow_vec,m_rows,m_cols,3);
        }

        std::vector<uchar> gray_shadow_vec;
        for(const float &val : tmp_shadow_vec)
            gray_shadow_vec.push_back(cv::saturate_cast<uchar>(255*val));
        return gray_shadow_vec;
    }

    std::vector<float> PointCloudProject::GetOriginDeph() const
    {
        return m_origin_depth_vec;
    }

    std::vector<cv::Vec3b> PointCloudProject::GetRGBShadowVec() const
    {
        std::vector<float> tmp_shadow_vec;
        tmp_shadow_vec.assign(m_shadows_vec.begin(),m_shadows_vec.end());

//        if(median_filter)
//        {
        MedianFilter(tmp_shadow_vec,m_rows,m_cols,3);
        MeanFilter(tmp_shadow_vec,m_rows,m_cols,3);
//        }

        std::vector<uchar> gray_shadow_vec;
        for(const float &val : tmp_shadow_vec)
            gray_shadow_vec.push_back(cv::saturate_cast<uchar>(255*val));

        cv::Mat img_gray(m_rows,m_cols,CV_8UC1,gray_shadow_vec.data());
        cv::cvtColor(img_gray,img_gray,CV_GRAY2BGR);

        std::vector<cv::Vec3b> rgb_shadow_vec;
        for(int i=0;i<m_rows;i++)
        {
            for(int j=0;j<m_cols;j++)
            {
                rgb_shadow_vec.push_back(img_gray.at<cv::Vec3b>(i,j));
            }
        }
        return rgb_shadow_vec;
    }

    std::vector<float> PointCloudProject::Normalize(const std::vector<float> &vec)
    {
        float min_val=*std::min_element(vec.begin(),vec.end());
        float max_val=*std::max_element(vec.begin(),vec.end());

        std::vector<float> result;
        for(const float &tmp : vec)
            result.push_back((tmp-min_val)/(max_val-min_val));
        return result;
    }

    std::vector<uchar> PointCloudProject::Float2Gray(const std::vector<float> &vec)
    {
        std::vector<uchar> gray;
        for(const float &val : vec)
            gray.push_back(cv::saturate_cast<uchar>(val*255));
        return gray;
    }

    void PointCloudProject::CameraModelFilter()
    {
        /// filter origin cloud using camera model
        float K_fx = m_K(0, 0), K_dx = m_K(0, 2), K_fy = m_K(1, 1), K_dy = m_K(1, 2);
        float u_limit_min = -K_dx / K_fx;
        float u_limit_max = (m_cols - K_dx) / K_fx;
        float v_limit_min = -K_dy / K_fy;
        float v_limit_max = (m_rows - K_dy) / K_fy;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_camera(new pcl::PointCloud<pcl::PointXYZINormal>);
        for (const pcl::PointXYZINormal &pt : m_cloud->points)
        {
            if (pt.z <= 0)
                continue;

            float xz_ratio = pt.x / pt.z;
            if (xz_ratio <= u_limit_min || xz_ratio >= u_limit_max)
                continue;

            float yz_ratio = pt.y / pt.z;
            if (yz_ratio <= v_limit_min || yz_ratio >= v_limit_max)
                continue;

            cloud_camera->points.push_back(pt);
        }
//        std::cout << "Cloud Camera Size ~ " << cloud_camera->points.size() << std::endl;
        std::swap(*m_cloud,*cloud_camera);
    }
    
    void PointCloudProject::FillHole()
    {
        std::vector<float> tmp_depth_vec(m_rows * m_cols, -1.0f);
        std::vector<cv::Vec3f> tmp_normal_vec(m_rows * m_cols, cv::Vec3f(0.0f, 0.0f, 0.0f));

        int num_fill = 0;

        /// ignore the outside
        for (int i = 1; i < m_rows-1; i++)
        {
            for (int j = 1; j < m_cols-1; j++)
            {
                int index = i * m_cols + j;
                /// if here exist an depth, then don't need interpolation
                if (m_depth_vec[index] != -1.0f)
                {
                    tmp_depth_vec[index] = m_depth_vec[index];
                    tmp_normal_vec[index] = m_normal_vec[index];
                    continue;
                }

                /// calculate the average depth and normals of 8 neighbours
                float tmp_depth_ave = 0.0f, tmp_x_ave = 0.0f, tmp_y_ave = 0.0f, tmp_z_ave = 0.0f;
                int num_neighbour = 0;
                for (int ii = i - 1; ii <= i + 1; ii++)
                {
                    for (int jj = j - 1; jj <= j + 1; jj++)
                    {
                        if (ii == i && jj == j)
                            continue;

                        int index_neighbour = ii * m_cols + jj;
                        if (m_depth_vec[index_neighbour] == -1.0f)
                            continue;

                        tmp_depth_ave += m_depth_vec[index_neighbour];
                        tmp_x_ave += m_normal_vec[index_neighbour][0];
                        tmp_y_ave += m_normal_vec[index_neighbour][1];
                        tmp_z_ave += m_normal_vec[index_neighbour][2];

                        num_neighbour++;
                    }
                }

                /// ignore if the positive neighbour num less than 3
                if (num_neighbour < 3)
                {
                    tmp_depth_vec[index] = m_depth_vec[index];
                    tmp_normal_vec[index] = m_normal_vec[index];
                    continue;
                }

                tmp_depth_vec[index] = tmp_depth_ave / num_neighbour;
                cv::Vec3f normal_neighbour(tmp_x_ave / num_neighbour, tmp_y_ave / num_neighbour,
                                           tmp_z_ave / num_neighbour);
                tmp_normal_vec[index] = normal_neighbour;

                num_fill++;
            }
        }
//        std::cout << "Number Fill ~ " << num_fill << std::endl;
        m_depth_vec.swap(tmp_depth_vec);
        m_normal_vec.swap(tmp_normal_vec);
    }

    /// project the point cloud into image planar
    void PointCloudProject::Project()
    {
        /// filter the input cloud using camera model
        float K_fx = m_K(0, 0), K_dx = m_K(0, 2), K_fy = m_K(1, 1), K_dy = m_K(1, 2);
        float u_limit_min = -K_dx / K_fx;
        float u_limit_max = (m_cols - K_dx) / K_fx;
        float v_limit_min = -K_dy / K_fy;
        float v_limit_max = (m_rows - K_dy) / K_fy;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_camera(new pcl::PointCloud<pcl::PointXYZINormal>);
        for (const pcl::PointXYZINormal &pt : m_cloud->points)
        {
            if (pt.z <= 0)
                continue;

            float xz_ratio = pt.x / pt.z;
            if (xz_ratio <= u_limit_min || xz_ratio >= u_limit_max)
                continue;

            double yz_ratio = pt.y / pt.z;
            if (yz_ratio <= v_limit_min || yz_ratio >= v_limit_max)
                continue;

            cloud_camera->points.push_back(pt);
        }
        std::cout << "Cloud Camera Size ~ " << cloud_camera->points.size() << std::endl;

        /// project by pinhole camera model
        std::vector<ProjectBase> tmp_vec;
        for (const pcl::PointXYZINormal &pt : cloud_camera->points)
        {
            /// camera coordinate to image coordinate
            Eigen::Vector3f pos_3d(pt.x, pt.y, pt.z);
            Eigen::Vector2f pos_2d = (m_K * pos_3d).head<2>() / pt.z;

            Eigen::Vector3f normal_3d(pt.normal_x, pt.normal_y, pt.normal_z);
            normal_3d.normalize();

            ProjectBase projectBase;
            projectBase.u_p = pos_2d(0);
            projectBase.v_p = pos_2d(1);
            projectBase.depth = pt.z;
            projectBase.normal_x = normal_3d(0);
            projectBase.normal_y = normal_3d(1);
            projectBase.normal_z = normal_3d(2);
            tmp_vec.push_back(projectBase);
        }
        std::cout << "Project Done! ~ " << tmp_vec.size() << std::endl;

        /// remove the different point on same optical azix
        std::sort(tmp_vec.begin(), tmp_vec.end());
        float last_u = 0.0f, last_v = 0.0f;
        for (const ProjectBase &tmp : tmp_vec)
        {
            if (m_2d_points.empty())
            {
                m_2d_points.push_back(tmp);
                last_u = tmp.u_p;
                last_v = tmp.v_p;
                continue;
            }

            if (last_u == tmp.u_p && last_v == tmp.v_p)
                continue;

            m_2d_points.push_back(tmp);
            last_u = tmp.u_p;
            last_v = tmp.v_p;
        }
        std::cout << "Extract Unique Done! ~ " << m_2d_points.size() << std::endl;

//        /// normalize depth to [0,1]
//        float depth_min=10000000;
//        float depth_max=-10000000;
//        for(const ProjectBase &pt_2d:m_2d_points)
//        {
//            depth_min=(depth_min<pt_2d.depth)?depth_min:pt_2d.depth;
//            depth_max=(depth_max>pt_2d.depth)?depth_max:pt_2d.depth;
//        }
//
//        for(ProjectBase &pt_2d : m_2d_points)
//            pt_2d.depth=(pt_2d.depth-depth_min)/(depth_max-depth_min);
//        std::cout<<"Normalize Depth Done!"<<std::endl;

        std::vector<std::vector<float>> tmp_depth_vec(m_rows * m_cols, std::vector<float>());
        std::vector<std::vector<float>> tmp_x_vec(m_rows * m_cols, std::vector<float>());
        std::vector<std::vector<float>> tmp_y_vec(m_rows * m_cols, std::vector<float>());
        std::vector<std::vector<float>> tmp_z_vec(m_rows * m_cols, std::vector<float>());
        for (const ProjectBase &pt_2d : m_2d_points)
        {
            int row = std::floor(pt_2d.v_p);
            int col = std::floor(pt_2d.u_p);

            int index = row * m_cols + col;
            tmp_depth_vec[index].push_back(pt_2d.depth);
            tmp_x_vec[index].push_back(pt_2d.normal_x);
            tmp_y_vec[index].push_back(pt_2d.normal_y);
            tmp_z_vec[index].push_back(pt_2d.normal_z);
        }

        /// z-buffer algorithm and using the average depth in each grid
        std::vector<float> depth_vec(m_rows * m_cols, 1.0f);
        std::vector<cv::Vec3f> normal_vec(m_rows * m_cols, cv::Vec3f(0.0f, 0.0f, 0.0f));
        int none_num = 0;
        for (int i = 0; i < m_rows; i++)
        {
            for (int j = 0; j < m_cols; j++)
            {
                int index = i * m_cols + j;
                if (tmp_depth_vec[index].empty())
                {
                    none_num++;
                    continue;
                }

//                float depth_ave = CalMean(tmp_depth_vec[index]);
//                float normal_x_ave = CalMean(tmp_x_vec[index]);
//                float normal_y_ave = CalMean(tmp_y_vec[index]);
//                float normal_z_ave = CalMean(tmp_z_vec[index]);
                float depth_ave = CalMedian(tmp_depth_vec[index]);
                float normal_x_ave = CalMedian(tmp_x_vec[index]);
                float normal_y_ave = CalMedian(tmp_y_vec[index]);
                float normal_z_ave = CalMedian(tmp_z_vec[index]);

                depth_vec[index] = depth_ave;
                normal_vec[index] = cv::Vec3f(normal_x_ave, normal_y_ave, normal_z_ave);
            }
        }
        m_depth_vec.swap(depth_vec);
        m_normal_vec.swap(normal_vec);
        std::cout << "Grid Not Exist Points ~ " << none_num << std::endl;

        /// normalize depth
//        float min_depth = *(std::min_element(m_depth_vec.begin(), m_depth_vec.end()));
//        float max_depth = *(std::max_element(m_depth_vec.begin(), m_depth_vec.end()));

        float min_depth = 100000000.0f, max_depth = -10000000.0f;
        for (const float &tmp_val : m_depth_vec)
        {
            if (tmp_val == -1.0f)
                continue;

            min_depth = (min_depth < tmp_val) ? min_depth : tmp_val;
            max_depth = (max_depth > tmp_val) ? max_depth : tmp_val;
        }

        for (float &depth : m_depth_vec)
        {
//            depth=(depth-min_depth)/(max_depth-min_depth);

            if (depth == -1.0f)
            {
                depth = 0.0f;
                continue;
            }

            depth = 0.1f + 0.9f * (depth - min_depth) / (max_depth - min_depth);
        }
        std::cout << "Normalize Depth Done!" << std::endl;

    }

    void PointCloudProject::Project2()
    {
        CameraModelFilter();

        /// project and z-buffer
        std::vector<float> tmp_depth_vec(m_rows * m_cols, -1.0f);
        m_normal_vec = std::vector<cv::Vec3f>(m_rows * m_cols,cv::Vec3f(0.0f,0.0f,0.0f));
        for (const pcl::PointXYZINormal &pt : m_cloud->points)
        {
            Eigen::Vector3f pos_3d(pt.x, pt.y, pt.z);
            Eigen::Vector3f pos_2d = m_K * pos_3d;

            Eigen::Vector3f normal_3d(pt.normal_x, pt.normal_y, pt.normal_z);
            normal_3d.normalize();

            int pt_row = std::floor(pos_2d(1) / pt.z);
            int pt_col = std::floor(pos_2d(0) / pt.z);

            int index = pt_row * m_cols + pt_col;
            if (tmp_depth_vec[index] == -1.0f || tmp_depth_vec[index] > pt.z)
            {
                tmp_depth_vec[index] = pt.z;
                m_normal_vec[index] = cv::Vec3f(normal_3d(0), normal_3d(1), normal_3d(2));
            }
        }
//        std::cout << "Project Done! ~ " << m_normal_vec.size() << std::endl;

        m_depth_vec.swap(tmp_depth_vec);
        m_origin_depth_vec.assign(m_depth_vec.begin(),m_depth_vec.end());

        FillHole();

//        float min_depth = *(std::min_element(m_depth_vec.begin(), m_depth_vec.end()));
//        float max_depth = *(std::max_element(m_depth_vec.begin(), m_depth_vec.end()));

        /// normalize depth
        float min_depth=100000000.0f,max_depth=-10000000.0f;
        for(const float &tmp_val : m_depth_vec)
        {
            if(tmp_val==-1.0f)
                continue;

            min_depth=(min_depth<tmp_val)?min_depth:tmp_val;
            max_depth=(max_depth>tmp_val)?max_depth:tmp_val;
        }

        for (float &depth : m_depth_vec)
        {
//            depth=(depth-min_depth)/(max_depth-min_depth);

            if(depth==-1.0f)
            {
                depth=0.0f;
                continue;
            }

            depth =0.1f+0.9f* (depth - min_depth) / (max_depth - min_depth);
        }
//        std::cout << "Normalize Depth Done!" << std::endl;
    }

    void PointCloudProject::EDL()
    {
        m_shadows_vec=ScaleEDL(m_depth_vec,m_rows,m_cols);
    }

    void PointCloudProject::EDLScale()
    {
        std::vector<float> shadow=ScaleEDL(m_depth_vec,m_rows,m_cols);

        std::vector<float> half_depth=ScaleDepth(m_depth_vec,m_rows,m_cols,2);
        int half_rows=std::floor(m_rows/2),half_cols=std::floor(m_cols/2);
        std::vector<float> half_shadow=ScaleEDL(half_depth,half_rows,half_cols);

        std::vector<float> quarter_depth=ScaleDepth(m_depth_vec,m_rows,m_cols,4);
        int quarter_rows=std::floor(m_rows/4),quarter_cols=std::floor(m_cols/4);
        std::vector<float> quarter_shadow=ScaleEDL(quarter_depth,quarter_rows,quarter_cols);

        std::vector<float> merge_shadow(m_rows*m_cols,0.0f);
        for(int i=0;i<m_rows;i++)
        {
            for(int j=0;j<m_cols;j++)
            {
                int index=i*m_cols+j;
                int index_half=std::floor(i/2)*half_cols+std::floor(j/2);
                int index_quarter=std::floor(i/4)*quarter_cols+std::floor(j/4);

                float val=(4.0f*shadow[index]+2.0f*half_shadow[index_half]+quarter_shadow[index_quarter])/7.0f;
                merge_shadow[index]=val;
            }
        }
        m_shadows_vec.swap(merge_shadow);
    }

    Eigen::Vector3f PointCloudProject::ReProject(int row, int col) const
    {
        float depth=m_origin_depth_vec[row*m_cols+col];
        /// if current pos is negative, then use 3*3 window to interplote the depth value
        if(depth==-1.0f)
        {
            std::vector<float> depth_vec;
            for(int i=std::max(0,row-1);i<=std::min(m_rows-1,row+1);i++)
            {
                for(int j=std::max(0,col-1);j<=std::min(m_cols-1,col);j++)
                {
                    float tmp_depth=m_origin_depth_vec[i*m_cols+j];
                    if(tmp_depth==-1.0f)
                        continue;
                    depth_vec.push_back(tmp_depth);
                }
            }
            if(depth_vec.empty())
                return Eigen::Vector3f(-1.0f,-1.0f,-1.0f);

            depth=std::accumulate(depth_vec.begin(),depth_vec.end(),0.0f)/depth_vec.size();
        }
        Eigen::Vector3f pixel_depth(col*depth,row*depth,depth);
        Eigen::Vector3f pos_3d=m_K.inverse()*pixel_depth;
        return pos_3d;
    }



}