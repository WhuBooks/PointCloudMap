//
// Created by whubooks on 18-5-2.
//

#include "EdgeFilter.h"

namespace PCM {

    /// back project threshold
    const int line_back_project_thres = 20;
    const int env_back_project_thres=70;

    /// inliers ratio threshold
    const double line_inlier_ratio_thres=0.3;
    const double env_inlier_ratio_thres=0.5;

    /// left and right planar density ratio threshold
    const double left_right_density_ratio_thres=0.3;

    ///line project search ratio threshold
    const double line_project_env_ratio=0.5;

    /// planar angle threshold
    const double left_right_angle_min_thres=0.2*M_PI;
    const double left_right_angle_max_thres=0.8*M_PI;

    EdgeFilter::EdgeFilter(int _rows, int _cols, const Eigen::Matrix3f &_K, const std::vector<float> &_origin_depth)
            :m_cloud(new pcl::PointCloud<pcl::PointXYZ>)
    {
        m_rows = _rows;
        m_cols = _cols;
        m_K = _K;
        m_origin_depth_vec.assign(_origin_depth.begin(), _origin_depth.end());
    }

    EdgeFilter::~EdgeFilter()
    {
    }

    void EdgeFilter::SetCloud(CloudXYZIPtr cloud)
    {
        m_cloud=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud,*m_cloud);
    }


    void EdgeFilter::SetRegion2D(const std::vector<Region2D> &line, const std::vector<Region2D> &left,
                                 const std::vector<Region2D> &right)
    {
        assert(line.size()==left.size() && left.size()==right.size());
        m_line_region_vec.assign(line.begin(), line.end());
        m_left_env_region_vec.assign(left.begin(), left.end());
        m_right_env_region_vec.assign(right.begin(), right.end());

//        for(int i=0;i<m_line_region_vec.size();i++)
//        {
//            Region2D line_region=m_line_region_vec[i];
//            Region2D left_env_region=m_left_env_region_vec[i];
//            Region2D right_env_region=m_right_env_region_vec[i];
//
//            CloudXYZPtr line_back=BackProjectLine(line_region);
//            CloudXYZPtr left_env_back=BackProjectEnv(left_env_region);
//            CloudXYZPtr right_env_back=BackProjectEnv(right_env_region);
//
//            m_line_back_vec.push_back(line_back);
//            m_left_env_back_vec.push_back(left_env_back);
//            m_right_env_back_vec.push_back(right_env_back);
//
//        }
    }

    int EdgeFilter::Filter()
    {
        m_line_back_vec.clear();
        m_line_search_vec.clear();
        m_line_sac_vec.clear();

        for (int i = 0; i < m_line_region_vec.size(); i++)
        {
//            std::cout<<std::endl<<"Iteration ~ "<<i<<std::endl;

            /// region 2d
            Region2D line_region = m_line_region_vec[i];
            Region2D left_env_region = m_left_env_region_vec[i];
            Region2D right_env_region = m_right_env_region_vec[i];

            /// back project by origin depth
            CloudXYZPtr line_back = BackProjectLine(line_region);
            CloudXYZPtr left_env_back = BackProjectEnv(left_env_region);
            CloudXYZPtr right_env_back = BackProjectEnv(right_env_region);

            /// search from origin cloud
            CloudXYZPtr line_search = SearchClosestWithinRadius(m_cloud, line_back);
            CloudXYZPtr left_env_search = SearchWithinRadius(m_cloud, left_env_back);
            CloudXYZPtr right_env_search = SearchWithinRadius(m_cloud, right_env_back);

            std::size_t d_c = line_region.size();
            std::size_t d_l = left_env_region.size();
            std::size_t d_r = right_env_region.size();

            std::size_t b_c = line_back->size();
            std::size_t b_l = left_env_back->size();
            std::size_t b_r = right_env_back->size();

            std::size_t n_c = line_search->size();
            std::size_t n_l = left_env_search->size();
            std::size_t n_r = right_env_search->size();

            if(n_c<line_back_project_thres)
            {
                continue;
            }
            if(std::max(n_l,n_r)<env_back_project_thres)
            {
                continue;
            }

            /// ransac
            std::vector<float> line_sac_coeff, left_env_sac_coeff, right_env_sac_coeff;
            CloudXYZPtr line_sac = SacLine(line_search, line_sac_coeff);
            CloudXYZPtr left_env_sac = SacPlanar(left_env_search, left_env_sac_coeff);
            CloudXYZPtr right_env_sac = SacPlanar(right_env_search, right_env_sac_coeff);

            std::size_t i_c = line_sac->size();
            std::size_t i_l = left_env_sac->size();
            std::size_t i_r = right_env_sac->size();

//            std::cout << "[ d_c d_l d_r ] ~ [ " << d_c << " " << d_l << " " << d_r << " ]" << std::endl;
//            std::cout << "[ b_c b_l b_r ] ~ [ " << b_c << " " << b_l << " " << b_r << " ]" << std::endl;
//            std::cout << "[ n_c n_l n_r ] ~ [ " << n_c << " " << n_l << " " << n_r << " ]" << std::endl;
//            std::cout << "[ i_c i_l i_r ] ~ [ " << i_c << " " << i_l << " " << i_r << " ]" << std::endl;

            float ir_c = (float) i_c / (n_c + 0.000000001f);
            float ir_l = (float) i_l / (n_l + 0.000000001f);
            float ir_r = (float) i_r / (n_r + 0.000000001f);

            bool flag_c = n_c >= line_back_project_thres && ir_c >= line_inlier_ratio_thres;
            bool flag_l = n_l >= env_back_project_thres && ir_l >= env_inlier_ratio_thres;
            bool flag_r = n_r >= env_back_project_thres && ir_r >= env_inlier_ratio_thres;

            if (!flag_c)
            {
//                std::cout<<"Line Failed [ n_c , ir_c ] ~ [ "<<n_c<<" , "<<std::setprecision(5)<<ir_c<<" ]"<<std::endl;
                continue;
            }
            if (!flag_l && !flag_r)
            {
//                std::cout<<"Envelop Failed [ n_l , ir_l ] ~ [ "<<n_l<<" , "<<std::setprecision(5)<<ir_l<<" ]\t";
//                std::cout<<" [ n_r , ir_r ] ~ [ "<<n_r<<" , "<<std::setprecision(5)<<ir_r<<" ]"<<std::endl;
                continue;
            }

            CloudXYZPtr line_project_search(new pcl::PointCloud<pcl::PointXYZ>);
            float left_env_density = PlanarDensity(left_env_sac);
            float right_env_density = PlanarDensity(right_env_sac);
//            std::cout << " Planar Density [ left , right ] ~ [" << std::setprecision(5) << left_env_density << " , " << right_env_density << " ]" << std::endl;

            if ((!flag_l && flag_r) || (left_env_density < left_right_density_ratio_thres * right_env_density))
            {
//                std::cout<<"Use Right Planar."<<std::endl;

//                PCM::CloudXYZPtr line_project_right_env=Project2Planar(line_search,right_env_sac_coeff);
//                line_env_project=SearchKnn(right_env_sac,line_project_right_env);

                line_project_search = LineEnv(line_search, right_env_sac, right_env_sac_coeff);
//              line_env_project=Project2Planar(line_search,right_env_sac_coeff);
            }
            else if ((flag_l && !flag_r) || (right_env_density < left_right_density_ratio_thres * left_env_density))
            {
//                std::cout<<"Use Left Planar."<<std::endl;

//                PCM::CloudXYZPtr line_project_left_env=Project2Planar(line_search,left_env_sac_coeff);
//                line_env_project=SearchKnn(left_env_sac,line_project_left_env);

                line_project_search = LineEnv(line_search, left_env_sac, left_env_sac_coeff);
//              line_env_project=Project2Planar(line_search,left_env_sac_coeff);
            }
            else
            {
//                std::cout<<"Failed Find Suitable Half-Planar [ left , right ] ~ ["<<std::setprecision(5)<<left_env_density<<" , "<<right_env_density<<" ]"<<std::endl;

                continue;
            }

            std::size_t n_lps = line_project_search->size();
            float r_lps = n_lps / (n_c + 0.0000001f);
            if (r_lps < line_project_env_ratio)
            {
//                std::cout<<"Project Search Failed [ n_lps , r_lps ] ~ [ "<<n_lps<<" , "<<std::setprecision(5)<<r_lps<<" ]"<<std::endl;
                continue;
            }

            std::vector<float> line_project_search_coeff;
            CloudXYZPtr line_env_sac = SacLine(line_project_search, line_project_search_coeff);
            if (line_env_sac->empty())
            {
//                std::cout<<"Failed Ransac Project Search Line."<<std::endl;
                continue;
            }

            for (const pcl::PointXYZ &pt : line_env_sac->points)
            {
                PCM::EdgePt edge_pt;
                edge_pt.pos = Eigen::Vector3f(pt.x, pt.y, pt.z);
                edge_pt.dir = Eigen::Vector3f(line_project_search_coeff[3], line_project_search_coeff[4],
                                              line_project_search_coeff[5]);
                m_edge_pts.push_back(edge_pt);
            }

//            std::cout<<" Ransac Line-Env Size ~ "<<line_env_sac->size()<<std::endl;


            m_index_vec.push_back(i);

            m_filter_line_region_vec.push_back(line_region);
            m_filter_left_env_region_vec.push_back(left_env_region);
            m_filter_right_env_region_vec.push_back(right_env_region);

            m_line_back_vec.push_back(line_back);
            m_line_search_vec.push_back(line_search);
            m_line_sac_vec.push_back(line_sac);

            m_left_env_back_vec.push_back(left_env_back);
            m_left_env_search_vec.push_back(left_env_search);
            m_left_env_sac_vec.push_back(left_env_sac);

            m_right_env_back_vec.push_back(right_env_back);
            m_right_env_search_vec.push_back(right_env_search);
            m_right_env_sac_vec.push_back(right_env_sac);

            m_line_env_project_vec.push_back(line_project_search);
            m_line_env_vec.push_back(line_env_sac);
        }
        return m_line_env_vec.size();
    }

    int EdgeFilter::Filter2()
    {
        for (int i = 0; i < m_line_region_vec.size(); i++)
        {
//            std::cout<<std::endl<<"Iteration ~ "<<i<<std::endl;

            /// region 2d
            Region2D line_region = m_line_region_vec[i];
            Region2D left_env_region = m_left_env_region_vec[i];
            Region2D right_env_region = m_right_env_region_vec[i];

            /// back project by origin depth
            CloudXYZPtr line_back = BackProjectLine(line_region);
            CloudXYZPtr left_env_back = BackProjectEnv(left_env_region);
            CloudXYZPtr right_env_back = BackProjectEnv(right_env_region);

            /// search from origin cloud
            CloudXYZPtr line_search = SearchClosestWithinRadius(m_cloud, line_back);
            CloudXYZPtr left_env_search = SearchWithinRadius(m_cloud, left_env_back);
            CloudXYZPtr right_env_search = SearchWithinRadius(m_cloud, right_env_back);

            std::size_t n_c = line_search->size();
            std::size_t n_l = left_env_search->size();
            std::size_t n_r = right_env_search->size();

            if (n_c < line_back_project_thres)
            {
                continue;
            }
            if (std::max(n_l, n_r) < env_back_project_thres)
            {
                continue;
            }
            /// ransac
            std::vector<float> line_sac_coeff, left_env_sac_coeff, right_env_sac_coeff;
            CloudXYZPtr line_sac = SacLine(line_search, line_sac_coeff);
            CloudXYZPtr left_env_sac = SacPlanar(left_env_search, left_env_sac_coeff);
            CloudXYZPtr right_env_sac = SacPlanar(right_env_search, right_env_sac_coeff);

            std::size_t i_c = line_sac->size();
            std::size_t i_l = left_env_sac->size();
            std::size_t i_r = right_env_sac->size();

            float ir_c = (float) i_c / (n_c + 0.000000001f);
            float ir_l = (float) i_l / (n_l + 0.000000001f);
            float ir_r = (float) i_r / (n_r + 0.000000001f);

            bool flag_l = n_l >= env_back_project_thres && ir_l >= env_inlier_ratio_thres;
            bool flag_r = n_r >= env_back_project_thres && ir_r >= env_inlier_ratio_thres;

            if (ir_c < line_inlier_ratio_thres)
            {
                continue;
            }
            if (!flag_l && !flag_r)
            {
                continue;
            }

            CloudXYZPtr line_project_search(new pcl::PointCloud<pcl::PointXYZ>);
            float left_env_density = PlanarDensity(left_env_sac);
            float right_env_density = PlanarDensity(right_env_sac);
            int type=0;

            if ((!flag_l && flag_r) || (left_env_density < left_right_density_ratio_thres * right_env_density))
            {
                line_project_search = LineEnv(line_search, right_env_sac, right_env_sac_coeff);
            }
            else if ((flag_l && !flag_r) || (right_env_density < left_right_density_ratio_thres * left_env_density))
            {
                line_project_search = LineEnv(line_search, left_env_sac, left_env_sac_coeff);
            }
            else
            {
                Eigen::Vector3f left_env_dir(left_env_sac_coeff[0],left_env_sac_coeff[1],left_env_sac_coeff[2]);
                Eigen::Vector3f right_env_dir(right_env_sac_coeff[0],right_env_sac_coeff[1],right_env_sac_coeff[2]);
                float angle=std::acos(left_env_dir.dot(right_env_dir)/(left_env_dir.norm()*right_env_dir.norm()));
                if(angle<left_right_angle_min_thres || angle>left_right_angle_max_thres)
                    continue;

                type=1;
                Eigen::Vector3f line_dir(line_sac_coeff[3],line_sac_coeff[4],line_sac_coeff[5]);
                float line_left_angle=std::acos(line_dir.dot(left_env_dir)/(line_dir.norm()*left_env_dir.norm()));
                float line_right_angle=std::acos(line_dir.dot(right_env_dir)/(line_dir.norm()*right_env_dir.norm()));

                float line_left_env_dis=std::exp(-1.0f/left_env_density)+std::exp(-(std::abs(line_left_angle-0.5*M_PI)));
                float line_right_env_dis=std::exp(-1.0f/right_env_density)+std::exp(-std::abs(line_right_angle-0.5*M_PI));

                if(line_left_env_dis<line_right_env_dis)
                {
                    line_project_search = LineEnv(line_search, right_env_sac, right_env_sac_coeff);
                }
                else
                {
                    line_project_search = LineEnv(line_search, left_env_sac, left_env_sac_coeff);
                }

            }

            std::size_t n_lps = line_project_search->size();
            float r_lps = n_lps / (n_c + 0.0000001f);
            if (r_lps < line_project_env_ratio)
            {
                continue;
            }

            std::vector<float> line_project_search_coeff;
            CloudXYZPtr line_env_sac = SacLine(line_project_search, line_project_search_coeff);
            if (line_env_sac->empty())
            {
                continue;
            }

            int edge_id=edge_pt_id++;
            for (const pcl::PointXYZ &pt : line_env_sac->points)
            {
                PCM::EdgePt edge_pt;
                edge_pt.id=edge_id;
                edge_pt.pos = Eigen::Vector3f(pt.x, pt.y, pt.z);
                edge_pt.dir = Eigen::Vector3f(line_project_search_coeff[3], line_project_search_coeff[4],
                                              line_project_search_coeff[5]);
                m_edge_pts.push_back(edge_pt);
            }

            m_index_vec.push_back(i);

            m_filter_line_region_vec.push_back(line_region);
            m_filter_left_env_region_vec.push_back(left_env_region);
            m_filter_right_env_region_vec.push_back(right_env_region);

            m_line_back_vec.push_back(line_back);
            m_line_search_vec.push_back(line_search);
            m_line_sac_vec.push_back(line_sac);

            m_left_env_back_vec.push_back(left_env_back);
            m_left_env_search_vec.push_back(left_env_search);
            m_left_env_sac_vec.push_back(left_env_sac);

            m_right_env_back_vec.push_back(right_env_back);
            m_right_env_search_vec.push_back(right_env_search);
            m_right_env_sac_vec.push_back(right_env_sac);

            m_edge_pts_type.push_back(type);
            m_line_env_project_vec.push_back(line_project_search);
            m_line_env_vec.push_back(line_env_sac);
        }
        return m_line_env_vec.size();

    }

    CloudXYZPtr EdgeFilter::BackProjectEnv(const Region2D &region_2d)
    {
        PosVec pos_vec;
        for (const cv::Point2i &pos_2d : region_2d)
        {
            int col = pos_2d.x;
            int row = pos_2d.y;
            float depth = m_origin_depth_vec[row * m_cols + col];

            if (depth == -1)
                continue;

            pos_vec.push_back(Eigen::Vector3f(col, row, depth));
        }

        /// apply median filter on depth to reduce the error
        bool use_median = true;
        if (use_median)
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
                        m_K.inverse() * Eigen::Vector3f(col_ii * median_depth, row_ii * median_depth, median_depth);
                median_pos_vec.push_back(pt_3d);
            }
            pos_vec.swap(median_pos_vec);
        }
        else
        {
            PosVec tmp_pos_vec;
            for (const Eigen::Vector3f &pos : pos_vec)
            {
                Eigen::Vector3f tmp_pos = m_K.inverse() * Eigen::Vector3f(pos(0) * pos(2), pos(1) * pos(2), pos(2));
                tmp_pos_vec.push_back(tmp_pos);
            }
            pos_vec.swap(tmp_pos_vec);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const Eigen::Vector3f &pos : pos_vec)
        {
            pcl::PointXYZ pt(pos(0), pos(1), pos(2));
            cloud->push_back(pt);
        }
        return cloud;
    }

    CloudXYZPtr EdgeFilter::BackProjectLine(const Region2D &region_2d)
    {
        PosVec pos_vec;
        for (const cv::Point2i &pos_2d : region_2d)
        {
            int col = pos_2d.x;
            int row = pos_2d.y;
            float depth = m_origin_depth_vec[row * m_cols + col];

            if (depth == -1)
                continue;

            pos_vec.push_back(Eigen::Vector3f(col, row, depth));
        }

        /// apply median filter on depth to reduce the error
        bool use_median = true;
        if (use_median)
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
                        m_K.inverse() * Eigen::Vector3f(col_ii * median_depth, row_ii * median_depth, median_depth);
                median_pos_vec.push_back(pt_3d);
            }
            pos_vec.swap(median_pos_vec);
        }
        else
        {
            PosVec tmp_pos_vec;
            for (const Eigen::Vector3f &pos : pos_vec)
            {
                Eigen::Vector3f tmp_pos = m_K.inverse() * Eigen::Vector3f(pos(0) * pos(2), pos(1) * pos(2), pos(2));
                tmp_pos_vec.push_back(tmp_pos);
            }
            pos_vec.swap(tmp_pos_vec);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const Eigen::Vector3f &pos : pos_vec)
        {
            pcl::PointXYZ pt(pos(0), pos(1), pos(2));
            cloud->push_back(pt);
        }
        return cloud;
    }

    CloudXYZPtr EdgeFilter::LineEnv(CloudXYZPtr line, CloudXYZPtr env, const std::vector<float> &env_coeff)
    {
        /// firstly project line to planar
        PCM::CloudXYZPtr line_project_env=Project2Planar(line,env_coeff);

        /// ransac the projected points to line
        std::vector<float> line_coeff;
        PCM::CloudXYZPtr line_sac=SacLine(line_project_env,line_coeff);

        PCM::CloudXYZPtr line_project_search(new pcl::PointCloud<pcl::PointXYZ>);

        /// search along the vertical direction
        if(!line_sac->empty())
        {
            /// compute the search direction
            Eigen::Vector3f nor1(line_coeff[3],line_coeff[4],line_coeff[5]);
            Eigen::Vector3f nor2(env_coeff[0],env_coeff[1],env_coeff[2]);
            Eigen::Vector3f dir=nor1.cross(nor2);

            /// search
            for(const pcl::PointXYZ &pt_proj : line_project_env->points)
            {
                std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>> pos_dis;
                for(const pcl::PointXYZ &pt_env : env->points)
                {
                    Eigen::Vector3f dir_tmp(pt_env.x-pt_proj.x,pt_env.y-pt_proj.y,pt_env.z-pt_proj.z);
                    float angle=std::acos(dir_tmp.dot(dir)/(dir.norm()*dir_tmp.norm()))*180.0f/M_PI;
                    float dis=dir_tmp.dot(dir)/dir.norm();
                    if(angle<20.0f || angle>160.0f)
                        pos_dis.push_back(Eigen::Vector4f(pt_env.x,pt_env.y,pt_env.z,dis));
                }

                if(pos_dis.empty())
                    continue;

                /// median filter by distance
                std::sort(pos_dis.begin(),pos_dis.end(),[](const Eigen::Vector4f &p1,const Eigen::Vector4f &p2){
                    return p1(3)<p2(3);
                });
                if(pos_dis.size()%2==0)
                {
                    Eigen::Vector4f pos_median=(pos_dis[pos_dis.size()/2-1]+pos_dis[pos_dis.size()/2])/2.0f;
                    line_project_search->push_back(pcl::PointXYZ(pos_median(0),pos_median(1),pos_median(2)));
                }
                else
                {
                    Eigen::Vector4f pos_median=pos_dis[(pos_dis.size()-1)/2];
                    line_project_search->push_back(pcl::PointXYZ(pos_median(0),pos_median(1),pos_median(2)));
                }
            }
//        std::cout<<"Project Search In Planar ~ "<<line_project_search->size()<<std::endl;
        }
        /// search closest within radius
        else
        {
            line_project_search=SearchClosestWithinRadius(env,line_project_env);
        }


//        if(line_project_search->size()<5)
//        {
////            std::cout<<"Failed Search Along Vertical Direction, Use Knn Means Search."<<std::endl;
//            line_project_search->clear();
//            line_project_search=SearchKnnMeans(env,line_project_env);
//        }

        return line_project_search;
    }

    void EdgeFilter::Write(std::string dir,std::vector<uchar> img) const
    {
        cv::Mat img_lsr(m_rows,m_cols,CV_8UC1,img.data());
        cv::cvtColor(img_lsr,img_lsr,CV_GRAY2BGR);

        cv::Mat img_all;
        img_lsr.copyTo(img_all);
        for(int i=0;i<m_line_env_vec.size();i++)
        {
            int index=m_index_vec[i];
            std::string img_file=dir+"/"+std::to_string(index)+".png";
            std::string search_file=dir+"/"+std::to_string(index)+"_search.las";
            std::string sac_file=dir+"/"+std::to_string(index)+"_sac.las";
            std::string proj_file=dir+"/"+std::to_string(index)+"_proj.las";
            std::string edge_file=dir+"/"+std::to_string(index)+"_filter.las";

            cv::Mat img_tmp;
            img_lsr.copyTo(img_tmp);
            Region2D line_region=m_filter_line_region_vec[i];
            std::sort(line_region.begin(),line_region.end(),[](const cv::Point2i &pt1,const cv::Point2i &pt2){
                return pt1.x<pt2.x ||(pt1.x==pt2.x && pt1.y<pt2.y);
            });
            cv::line(img_tmp,line_region.front(),line_region.back(),cv::Scalar(0,0,255));
            cv::line(img_all,line_region.front(),line_region.back(),cv::Scalar(0,0,255));
            cv::imwrite(img_file,img_tmp);

            CloudXYZPtr line_env=m_line_env_vec[i];
            CloudXYZPtr line_proj=m_line_env_project_vec[i];

            CloudXYZPtr line_search=m_line_search_vec[i];
            CloudXYZPtr left_env_search=m_left_env_search_vec[i];
            CloudXYZPtr right_env_search=m_right_env_search_vec[i];

            CloudXYZPtr line_sac=m_line_sac_vec[i];
            CloudXYZPtr left_env_sac=m_left_env_sac_vec[i];
            CloudXYZPtr right_env_sac=m_right_env_sac_vec[i];

            WriteLas(line_env,edge_file);
            WriteLas(line_proj,proj_file);
            WriteLas(line_search,left_env_search,right_env_search,search_file);
            WriteLas(line_sac,left_env_sac,right_env_sac,sac_file);
        }
        std::string img_all_file=dir+"/all.png";
        cv::imwrite(img_all_file,img_all);
    }

    void EdgeFilter::Write(std::string dir, std::vector<uchar> img, const Eigen::Affine3f &affine) const
    {
        cv::Mat img_lsr(m_rows,m_cols,CV_8UC1,img.data());
        cv::cvtColor(img_lsr,img_lsr,CV_GRAY2BGR);

        cv::Mat img_all;
        img_lsr.copyTo(img_all);
        for(int i=0;i<m_line_env_vec.size();i++)
        {
            int index=m_index_vec[i];
            std::string img_file=dir+"/"+std::to_string(index)+".png";
            std::string search_file=dir+"/"+std::to_string(index)+"_search.las";
            std::string sac_file=dir+"/"+std::to_string(index)+"_sac.las";
            std::string proj_file=dir+"/"+std::to_string(index)+"_proj.las";
            std::string edge_file=dir+"/"+std::to_string(index)+"_filter.las";

            cv::Mat img_tmp;
            img_lsr.copyTo(img_tmp);
            Region2D line_region=m_filter_line_region_vec[i];
            std::sort(line_region.begin(),line_region.end(),[](const cv::Point2i &pt1,const cv::Point2i &pt2){
                return pt1.x<pt2.x ||(pt1.x==pt2.x && pt1.y<pt2.y);
            });
            cv::line(img_tmp,line_region.front(),line_region.back(),cv::Scalar(0,0,255));
            cv::line(img_all,line_region.front(),line_region.back(),cv::Scalar(0,0,255));
            cv::imwrite(img_file,img_tmp);

            CloudXYZPtr line_env=m_line_env_vec[i];
            CloudXYZPtr line_proj=m_line_env_project_vec[i];

            CloudXYZPtr line_search=m_line_search_vec[i];
            CloudXYZPtr left_env_search=m_left_env_search_vec[i];
            CloudXYZPtr right_env_search=m_right_env_search_vec[i];

            CloudXYZPtr line_sac=m_line_sac_vec[i];
            CloudXYZPtr left_env_sac=m_left_env_sac_vec[i];
            CloudXYZPtr right_env_sac=m_right_env_sac_vec[i];

            CloudXYZPtr line_env_trans(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*line_env,*line_env_trans,affine);

            CloudXYZPtr line_proj_trans(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*line_proj,*line_proj_trans,affine);

            CloudXYZPtr line_search_trans(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*line_search,*line_search_trans,affine);

            CloudXYZPtr left_env_search_trans(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*left_env_search,*left_env_search_trans,affine);

            CloudXYZPtr right_env_search_trans(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*right_env_search,*right_env_search_trans,affine);

            CloudXYZPtr line_sac_trans(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*line_sac,*line_sac_trans,affine);

            CloudXYZPtr left_env_sac_trans(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*left_env_sac,*left_env_sac_trans,affine);

            CloudXYZPtr right_env_sac_trans(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*right_env_sac,*right_env_sac_trans,affine);

            WriteLas(line_env_trans,edge_file);
            WriteLas(line_proj_trans,proj_file);
            WriteLas(line_search_trans,left_env_search_trans,right_env_search_trans,search_file);
            WriteLas(line_sac_trans,left_env_sac_trans,right_env_sac_trans,sac_file);
        }
        std::string img_all_file=dir+"/all.png";
        cv::imwrite(img_all_file,img_all);
    }




}