//
// Created by whubooks on 18-4-18.
//

#include "LineSupportRegion.h"

namespace PCM {


    LineSupportRegion::LineSupportRegion(int rows, int cols, const std::vector<uchar> &data)
    {
        length_thres=25;
        width_thres=10;
        
        m_rows = rows;
        m_cols = cols;
        m_img.assign(data.begin(), data.end());
    }

    LineSupportRegion::~LineSupportRegion()
    {

    }

    void LineSupportRegion::Detect()
    {
        /// line segment detector
        cv::Mat img(m_rows, m_cols, CV_8UC1, m_img.data());
        cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector(cv::LSD_REFINE_NONE);
        std::vector<cv::Vec4d> lines;
        std::vector<double> widths;
        lsd->detect(img, lines, widths);

        std::vector<Line2D> line_vec;
        std::vector<Envelop2D> env_vec;
        std::vector<Envelop2D> left_env_vec, right_env_vec;
        int num_negative = 0;
        for (int i = 0; i < lines.size(); i++)
        {
            cv::Point2d pt_s(lines[i][0], lines[i][1]);
            cv::Point2d pt_e(lines[i][2], lines[i][3]);
            double width = widths[i];

            double length = std::sqrt((pt_s.x - pt_e.x) * (pt_s.x - pt_e.x) + (pt_s.y - pt_e.y) * (pt_s.y - pt_e.y));
            if(length<30)
            {
                num_negative++;
                continue;
            }

            Line2D line;
            line.s=pt_s;
            line.e=pt_e;
            line_vec.push_back(line);

            Envelop2D env = LineRectangle(pt_s, pt_e, width);
            Envelop2D env_left, env_right;
            env_left.lt = env.lt;
            env_left.rt = pt_s;
            env_left.rb = pt_e;
            env_left.lb = env.lb;
            env_right.lt = pt_s;
            env_right.rt = env.rt;
            env_right.rb = env.rb;
            env_right.lb = pt_e;

            env_vec.push_back(env);
            left_env_vec.push_back(env_left);
            right_env_vec.push_back(env_right);
        }

        m_line_vec.swap(line_vec);
        m_env_vec.swap(env_vec);
        m_left_env_vec.swap(left_env_vec);
        m_right_env_vec.swap(right_env_vec);


        /// find the 2d line support region
        std::vector<std::vector<cv::Point2i>> line_region_vec(m_line_vec.size(),std::vector<cv::Point2i>());
        std::vector<std::vector<cv::Point2i>> region_vec(m_env_vec.size(), std::vector<cv::Point2i>());
        std::vector<std::vector<cv::Point2i>> left_region_vec(m_env_vec.size(), std::vector<cv::Point2i>());
        std::vector<std::vector<cv::Point2i>> right_region_vec(m_env_vec.size(), std::vector<cv::Point2i>());
        for (int i = 0; i < m_rows; i++)
        {
            for (int j = 0; j < m_cols; j++)
            {
                cv::Point2i pt(j, i);
                for (int index = 0; index < m_env_vec.size(); index++)
                {
                    if(m_line_vec[index].Contain(pt))
                        line_region_vec[index].push_back(pt);

                    if (m_env_vec[index].Contain(pt))
                        region_vec[index].push_back(pt);
                    if (m_left_env_vec[index].Contain(pt))
                        left_region_vec[index].push_back(pt);
                    if (m_right_env_vec[index].Contain(pt))
                        right_region_vec[index].push_back(pt);
                }
            }
        }
        m_line_region_vec.swap(line_region_vec);
        m_env_region_vec.swap(region_vec);
        m_left_env_region_vec.swap(left_region_vec);
        m_right_env_region_vec.swap(right_region_vec);

        /// merge the overlap region

    }

    void LineSupportRegion::Detect2()
    {
        /// line segment detector
        cv::Mat img(m_rows, m_cols, CV_8UC1, m_img.data());
        cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector(cv::LSD_REFINE_NONE);
        std::vector<cv::Vec4d> lines;
        std::vector<double> widths;
        lsd->detect(img, lines, widths);

        std::vector<Line2D> line_vec;
        int num_negative = 0;
        for (int i = 0; i < lines.size(); i++)
        {
            cv::Point2d pt_s(lines[i][0], lines[i][1]);
            cv::Point2d pt_e(lines[i][2], lines[i][3]);
            double width = widths[i];

            double length = std::sqrt((pt_s.x - pt_e.x) * (pt_s.x - pt_e.x) + (pt_s.y - pt_e.y) * (pt_s.y - pt_e.y));
            if(length<30)
            {
                num_negative++;
                continue;
            }

            Line2D line;
            line.s=pt_s;
            line.e=pt_e;
            line_vec.push_back(line);
        }

        std::sort(line_vec.begin(),line_vec.end(),[](const Line2D &line1,const Line2D &line2){
           return line1.Length()>line2.Length();
        });
        m_line_vec.swap(line_vec);

        /// find the 2d line support region
        std::vector<std::vector<cv::Point2i>> line_region_vec(m_line_vec.size(),std::vector<cv::Point2i>());
        for (int i = 0; i < m_rows; i++)
        {
            for (int j = 0; j < m_cols; j++)
            {
                cv::Point2i pt(j, i);
                for (int index = 0; index < m_line_vec.size(); index++)
                {
                    if(m_line_vec[index].Contain(pt))
                        line_region_vec[index].push_back(pt);
                }
            }
        }
        m_line_region_vec.swap(line_region_vec);
        /// merge the overlap region

    }

    void LineSupportRegion::Detect3()
    {
        /// line segment detector
        cv::Mat img(m_rows, m_cols, CV_8UC1, m_img.data());
        cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector(cv::LSD_REFINE_NONE);
        std::vector<cv::Vec4d> lines;
        std::vector<double> widths;
        lsd->detect(img, lines, widths);

        std::vector<Line2D> line_vec;
        std::vector<Line2D> left_line_vec,right_line_vec;

        std::vector<Envelop2D> env_vec;
        std::vector<Envelop2D> left_env_vec,right_env_vec;
        int num_negative = 0;
        for (int i = 0; i < lines.size(); i++)
        {
            cv::Point2d pt_s(lines[i][0], lines[i][1]);
            cv::Point2d pt_e(lines[i][2], lines[i][3]);
            double width = widths[i];

            double length = std::sqrt((pt_s.x - pt_e.x) * (pt_s.x - pt_e.x) + (pt_s.y - pt_e.y) * (pt_s.y - pt_e.y));
            if(length<length_thres || width<width_thres)
            {
                num_negative++;
                continue;
            }

            Line2D line(pt_s,pt_e);
            line_vec.push_back(line);

            Envelop2D env(line,width);
            Envelop2D left_env(env.lt,pt_s,pt_e,env.lb);
            Envelop2D right_env(pt_s,env.rt,env.rb,pt_e);
            env_vec.push_back(env);
            left_env_vec.push_back(left_env);
            right_env_vec.push_back(right_env);

            Line2D left_line(env.lt,env.lb);
            Line2D right_line(env.rt,env.rb);
            left_line_vec.push_back(left_line);
            right_line_vec.push_back(right_line);
        }

        m_line_vec.swap(line_vec);
        m_left_line_vec.swap(left_line_vec);
        m_right_line_vec.swap(right_line_vec);

        m_env_vec.swap(env_vec);
        m_left_env_vec.swap(left_env_vec);
        m_right_env_vec.swap(right_env_vec);

        /// find the 2d line support region
        std::vector<Region2D> line_region_vec(m_line_vec.size(),Region2D());
        std::vector<Region2D> left_line_region_vec(m_line_vec.size(), Region2D());
        std::vector<Region2D> right_line_region_vec(m_line_vec.size(), Region2D());

        std::vector<Region2D> env_region_vec(m_line_vec.size(), Region2D());
        std::vector<Region2D> left_env_region_vec(m_line_vec.size(), Region2D());
        std::vector<Region2D> right_env_region_vec(m_line_vec.size(), Region2D());

        for (int i = 0; i < m_rows; i++)
        {
            for (int j = 0; j < m_cols; j++)
            {
                cv::Point2i pt(j, i);
                for (int index = 0; index < m_line_vec.size(); index++)
                {
                    bool line_flag = m_line_vec[index].Contain(pt);
                    bool left_line_flag = m_left_line_vec[index].Contain(pt);
                    bool right_line_flag = m_right_line_vec[index].Contain(pt);

                    bool env_flag = m_env_vec[index].Contain(pt);
                    bool left_env_flag = m_left_env_vec[index].Contain(pt);
                    bool right_env_flag = m_right_env_vec[index].Contain(pt);

                    if (line_flag)
                        line_region_vec[index].push_back(pt);
                    if (left_line_flag)
                        left_line_region_vec[index].push_back(pt);
                    if (right_line_flag)
                        right_line_region_vec[index].push_back(pt);

                    if (env_flag)// && !line_flag && !left_line_flag && !right_line_flag)
                        env_region_vec[index].push_back(pt);
                    if (left_env_flag && !line_flag)// && !left_line_flag)
                        left_env_region_vec[index].push_back(pt);
                    if (right_env_flag && !line_flag)// && !right_line_flag)
                        right_env_region_vec[index].push_back(pt);

                }
            }
        }
        m_line_region_vec.swap(line_region_vec);
        m_left_line_region_vec.swap(left_line_region_vec);
        m_right_line_region_vec.swap(right_line_region_vec);

        m_env_region_vec.swap(env_region_vec);
        m_left_env_region_vec.swap(left_env_region_vec);
        m_right_env_region_vec.swap(right_env_region_vec);

    }


//    void LineSupportRegion::BackProject(const std::vector<float> &depth_vec,const Eigen::Matrix3f &K)
//    {
//        std::vector<Line2D> filter_line_vec;
//        std::vector<std::vector<cv::Point2i>> filter_line_region_vec;
//        std::vector<Edge> edge_vec;
//        std::vector<Edge> median_edge_vec;
//        for (int i = 0; i < m_line_vec.size(); i++)
//        {
//            Line2D line_i = m_line_vec[i];
//            std::vector<cv::Point2i> line_region_i = m_line_region_vec[i];
//
//            /// find the positive pixel
//            Edge pixel_edge;
//            Edge origin_edge;
//            for (const cv::Point2i &pt_2d : line_region_i)
//            {
//                /// back-project 2d point
//                int row = pt_2d.y, col = pt_2d.x;
//                float depth = depth_vec[row * m_cols + col];
//
//                /// if current pos is negative, then use 3*3 window to interplote the depth value
//                if (depth == -1.0f)
//                {
//                    float sum_weight=0.0f;
//                    float sum_depth=0.0f;
//                    int sum_num=0;
//
//                    for (int ii = std::max(0, row - 1); ii <= std::min(m_rows - 1, row + 1); ii++)
//                    {
//                        for (int jj = std::max(0, col - 1); jj <= std::min(m_cols - 1, col + 1); jj++)
//                        {
//                            float tmp_depth = depth_vec[ii * m_cols + jj];
//                            if (tmp_depth == -1.0f)
//                                continue;
//                            float tmp_weight=std::sqrt((ii-row)*(ii-row)+(jj-col)*(jj-col));
//                            sum_weight+=tmp_weight;
//                            sum_depth+=tmp_depth*tmp_weight;
//                            sum_num++;
//                        }
//                    }
//                    if(sum_num<4)
//                        continue;
//
//                    depth=sum_depth/sum_weight;
//                }
//                Eigen::Vector3f pixel_depth(col, row, depth);
//                pixel_edge.push_back(pixel_depth);
//
//                Eigen::Vector3f origin_pos=K.inverse()*Eigen::Vector3f(col*depth,row*depth,depth);
//                origin_edge.push_back(origin_pos);
//            }
//
//            /// ignore the edge has no enough point
//            if (pixel_edge.size() < 30)
//                continue;
//
//            /// apply median filter on depth to reduce the error
//            Edge median_edge;
//            const int N = 2;
//            for (int ii = 0; ii < pixel_edge.size(); ii++)
//            {
//                std::vector<float> tmp_depth,tmp_row,tmp_col;
//                for (int jj = std::max(0, ii - N); jj <= std::min(ii + N, (int) pixel_edge.size()-1); jj++)
//                {
//                    float depth_jj=pixel_edge[jj](2);
//                    tmp_depth.push_back(depth_jj);
//                }
//
//                std::sort(tmp_depth.begin(), tmp_depth.end());
//                float median_depth = 0.0f;
//                if (tmp_depth.size() % 2 == 1)
//                    median_depth = tmp_depth[(tmp_depth.size() - 1) / 2];
//                else
//                    median_depth = (tmp_depth[tmp_depth.size() / 2] + tmp_depth[tmp_depth.size() / 2 - 1]) / 2.0f;
//
//                float col_ii=pixel_edge[ii](0);
//                float row_ii=pixel_edge[ii](1);
//                Eigen::Vector3f pt_3d = K.inverse()*Eigen::Vector3f(col_ii*median_depth,row_ii*median_depth,median_depth);
//                median_edge.push_back(pt_3d);
//            }
//
//            filter_line_vec.push_back(line_i);
//            filter_line_region_vec.push_back(line_region_i);
//            median_edge_vec.push_back(median_edge);
//            edge_vec.push_back(origin_edge);
//        }
//        m_filter_line_vec.swap(filter_line_vec);
//        m_filter_line_region_vec.swap(filter_line_region_vec);
//        m_median_edge_vec.swap(median_edge_vec);
//        m_edge_vec.swap(edge_vec);
//    }

    Envelop2D LineSupportRegion::LineRectangle(const cv::Point2d &pt_s, const cv::Point2d &pt_e, double width)
    {
        double x1 = pt_s.x, y1 = pt_s.y;
        double x2 = pt_e.x, y2 = pt_e.y;
        double length = std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

        double sin = (y2 - y1) / length;
        double cos = (x2 - x1) / length;

        Eigen::Matrix3d trans = Eigen::Matrix3d::Identity();
        trans << sin, cos, x1, -cos, sin, y1, 0.0, 0.0, 1.0;
        std::cout << "Trans ~ " << trans << std::endl;

        Eigen::Vector3d lt = trans * Eigen::Vector3d(-width / 2.0, length / 2.0, 1.0);
        Eigen::Vector3d rt = trans * Eigen::Vector3d(width / 2.0, length / 2.0, 1.0);
        Eigen::Vector3d rb = trans * Eigen::Vector3d(width / 2.0, -length / 2.0, 1.0);
        Eigen::Vector3d lb = trans * Eigen::Vector3d(-width / 2.0, -length / 2.0, 1.0);

        Envelop2D env;
        env.lt = cv::Point2d(lt(0), lt(1));
        env.rt = cv::Point2d(rt(0), rt(1));
        env.rb = cv::Point2d(rb(0), rb(1));
        env.lb = cv::Point2d(lb(0), lb(1));
        return env;
    }

    std::vector<PCM::Envelop2D> LineSupportRegion::GetEnvelop() const
    {
        return m_env_vec;
    }

    std::vector<PCM::Envelop2D> LineSupportRegion::GetLeftEnvelop() const
    {
        return m_left_env_vec;
    }

    std::vector<PCM::Envelop2D> LineSupportRegion::GetRightEnvelop() const
    {
        return m_right_env_vec;
    }

    std::vector<Region2D> LineSupportRegion::GetEnvRegion() const
    {
        return m_env_region_vec;
    }

    std::vector<Region2D> LineSupportRegion::GetLeftEnvRegion() const
    {
        return m_left_env_region_vec;
    }

    std::vector<Region2D> LineSupportRegion::GetRightEnvRegion() const
    {
        return m_right_env_region_vec;
    }

    std::vector<PCM::Line2D> LineSupportRegion::GetLine() const
    {
        return m_line_vec;
    }

    std::vector<Region2D> LineSupportRegion::GetLineRegion() const
    {
        return m_line_region_vec;
    }

//    std::vector<Edge> LineSupportRegion::GetEdgeVec() const
//    {
//        return m_edge_vec;
//    }
//
//    std::vector<PCM::Line2D> LineSupportRegion::GetFilterLine() const
//    {
//        return m_filter_line_vec;
//    }
//
//    std::vector<Region2D> LineSupportRegion::GetFilterLineRegion() const
//    {
//        return m_filter_line_region_vec;
//    }
//
//    std::vector<Edge> LineSupportRegion::GetMedianEdgeVec() const
//    {
//        return m_median_edge_vec;
//    }

    std::vector<PCM::Line2D> LineSupportRegion::GetLeftLine() const
    {
        return m_left_line_vec;
    }

    std::vector<Region2D> LineSupportRegion::GetLeftLineRegion() const
    {
        return m_left_line_region_vec;
    }

    std::vector<PCM::Line2D> LineSupportRegion::GetRightLine() const
    {
        return m_right_line_vec;
    }

    std::vector<Region2D> LineSupportRegion::GetRightLineRegion() const
    {
        return m_right_line_region_vec;
    }


}