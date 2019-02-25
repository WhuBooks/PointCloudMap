//
// Created by whubooks on 18-4-18.
//

#ifndef POINTCLOUDMAP_LINESUPPORTREGION_H
#define POINTCLOUDMAP_LINESUPPORTREGION_H

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

#include <CloudUtil.h>

namespace PCM {


    struct Line2D
    {
        cv::Point2d s;
        cv::Point2d e;

        Line2D()
        {
            s=cv::Point2d(0.0,0.0);
            e=cv::Point2d(0.0,0.0);
        };

        Line2D(const cv::Point2d &_s,const cv::Point2d &_e)
        {
            s=_s;
            e=_e;
        };

        double Length() const
        {
            return std::sqrt((s.x-e.x)*(s.x-e.x)+(s.y-e.y)*(s.y-e.y));
        }

        bool Contain(const cv::Point2i &pt) const
        {
            Eigen::Vector2d ab(e.x-s.x,e.y-s.y);
            Eigen::Vector2d ap(pt.x-s.x,pt.y-s.y);

            double ap_ab_product=ap.dot(ab);
            double r=ap_ab_product/(ab.norm()*ab.norm());
            if(r<0 || r>1)
                return false;

            Eigen::Vector2d ac=r*ab;
            double dis=std::sqrt(ap.norm()*ap.norm()-ac.norm()*ac.norm());

            return dis<=1;

        }
    };

    struct Envelop2D
    {
        cv::Point2d lt;
        cv::Point2d rt;
        cv::Point2d rb;
        cv::Point2d lb;

        Envelop2D()
        {
            lt = cv::Point2d(0.0, 0.0);
            rt = cv::Point2d(0.0, 0.0);
            lb = cv::Point2d(0.0, 0.0);
            rb = cv::Point2d(0.0, 0.0);
        };

        Envelop2D(const cv::Point2d &_lt, const cv::Point2d &_rt, const cv::Point2d &_rb, const cv::Point2d &_lb)
        {
            lt = _lt;
            rt = _rt;
            rb = _rb;
            lb = _lb;
        };

        Envelop2D(const Line2D &line,double width)
        {
            double x1 = line.s.x, y1 = line.s.y;
            double x2 = line.e.x, y2 = line.e.y;
            double length = std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

            double sin = (y1 - y2) / length;
            double cos = (x1 - x2) / length;

            Eigen::Matrix3d trans = Eigen::Matrix3d::Identity();
            trans << sin, cos, x2, -cos, sin, y2, 0.0, 0.0, 1.0;

            Eigen::Vector3d lt_pos = trans * Eigen::Vector3d(-width / 2.0, length, 1.0);
            Eigen::Vector3d rt_pos = trans * Eigen::Vector3d(width / 2.0, length, 1.0);
            Eigen::Vector3d rb_pos = trans * Eigen::Vector3d(width / 2.0, 0.0, 1.0);
            Eigen::Vector3d lb_pos = trans * Eigen::Vector3d(-width / 2.0, 0.0, 1.0);

//            Eigen::Matrix2d rot=Eigen::Matrix2d::Identity();
//            rot<<sin,cos,-cos,sin;
//
//            Eigen::Vector2d lt_pos = rot * Eigen::Vector2d(-width / 2.0+x1, length / 2.0+y1);
//            Eigen::Vector2d rt_pos = rot * Eigen::Vector2d(width / 2.0+x1, length / 2.0+y1);
//            Eigen::Vector2d rb_pos = rot * Eigen::Vector2d(width / 2.0+x1, -length / 2.0+y1);
//            Eigen::Vector2d lb_pos = rot * Eigen::Vector2d(-width / 2.0+x1, -length / 2.0+y1);

            lt = cv::Point2d(lt_pos(0), lt_pos(1));
            rt = cv::Point2d(rt_pos(0), rt_pos(1));
            rb = cv::Point2d(rb_pos(0), rb_pos(1));
            lb = cv::Point2d(lb_pos(0), lb_pos(1));

//            Eigen::Vector2d lt_rt(lt.x - rt.x, lt.y - rt.y);
//            Eigen::Vector2d lt_s(lt.x - line.s.x, lt.y - line.s.y);
//            Eigen::Vector2d rb_lb(rb.x - lb.x, rb.y - lb.y);
//            Eigen::Vector2d rb_e(rb.x - line.e.x, rb.y - line.e.y);
//
//            Eigen::Vector2d rt_rb(rt.x - rb.x, rt.y - rb.y);
//            Eigen::Vector2d rt_s(rt.x - line.s.x, rt.y - line.s.y);
//            Eigen::Vector2d lb_lt(lb.x - lt.x, lb.y - lt.y);
//            Eigen::Vector2d lb_e(lb.x - line.e.x, lb.y - line.e.y);

//            double w1=std::abs(lt_s.norm());
//            double w2=std::abs(rt_s.norm());
//            double w3=std::abs(rb_e.norm());
//            double w4=std::abs(lb_e.norm());
//
//            double w5=std::abs(lt_rt.norm());
//            double w6=std::abs(rb_lb.norm());
//            double w7=std::abs(rt_rb.norm());
//            double w8=std::abs(lb_lt.norm());


        };

        bool Contain(const cv::Point2i &pt) const
        {
            double length = std::sqrt((lt.x - lb.x) * (lt.x - lb.x) + (lt.y - lb.y) * (lt.y - lb.y));
            double width = std::sqrt((lt.x - rt.x) * (lt.x - rt.x) + (lt.y - rt.y) * (lt.y - rt.y));
            double sin = (lt.y - lb.y) / length;
            double cos = (lt.x - lb.x) / length;
            Eigen::Matrix2d rot = Eigen::Matrix2d::Identity();
            rot << sin, -cos, cos, sin;

            Eigen::Vector2d pos(pt.x - lb.x, pt.y - lb.y);
            Eigen::Vector2d pos_trans = rot * pos;

            if (pos_trans(0) < 0 || pos_trans(0) > width || pos_trans(1) < 0 || pos_trans(1) > length)
                return false;
            return true;


//            cv::Point2d lt_rt(lt.x - rt.x, lt.y - rt.y);
//            cv::Point2d lt_pos(lt.x - pt.x, lt.y - pt.y);
//            cv::Point2d rb_lb(rb.x - lb.x, rb.y - lb.y);
//            cv::Point2d rb_pos(rb.x - pt.x, rb.y - pt.y);
//
//            cv::Point2d rt_rb(rt.x - rb.x, rt.y - rb.y);
//            cv::Point2d rt_pos(rt.x - pt.x, rt.y - pt.y);
//            cv::Point2d lb_lt(lb.x - lt.x, lb.y - lt.y);
//            cv::Point2d lb_pos(lb.x - pt.x, lb.y - pt.y);
//
//            return (lt_rt.cross(lt_pos) * rb_lb.cross(rb_pos)) >= 0 &&
//                   (rt_rb.cross(rt_pos) * lb_lt.cross(lb_pos)) >= 0;
        };

        void Check()
        {
            Eigen::Vector2d lt_rt(lt.x - rt.x, lt.y - rt.y);
            Eigen::Vector2d rb_lb(rb.x - lb.x, rb.y - lb.y);
            Eigen::Vector2d rt_rb(rt.x - rb.x, rt.y - rb.y);
            Eigen::Vector2d lb_lt(lb.x - lt.x, lb.y - lt.y);

            double val1=std::abs(lt_rt.dot(rt_rb));
            double val2=std::abs(lt_rt.dot(lb_lt));
            double val3=std::abs(rt_rb.dot(rb_lb));
            double val4=std::abs(rb_lb.dot(lb_lt));

            assert(val1<0.00001 && val2<0.00001 && val3<0.000001 && val4<0.00001);
        }

        void Check(double width,double length)
        {
            Eigen::Vector2d lt_rt(lt.x - rt.x, lt.y - rt.y);
            Eigen::Vector2d rb_lb(rb.x - lb.x, rb.y - lb.y);
            Eigen::Vector2d rt_rb(rt.x - rb.x, rt.y - rb.y);
            Eigen::Vector2d lb_lt(lb.x - lt.x, lb.y - lt.y);

            double val1 = lt_rt.dot(rt_rb);
            double val2 = lt_rt.dot(lb_lt);
            double val3 = rt_rb.dot(rb_lb);
            double val4 = rb_lb.dot(lb_lt);

            assert(val1 < 0.00001 && val2 < 0.00001 && val3 < 0.000001 && val4 < 0.00001);

            double w1 = lt_rt.norm();
            double l1 = rt_rb.norm();
            double w2 = rb_lb.norm();
            double l2 = lb_lt.norm();

            assert(std::abs(w1 - width) < 1 && std::abs(w2 - width) < 1 && std::abs(l1 - length) < 1 &&
                   std::abs(l2 - length) < 1);


        }

    };

    class LineSupportRegion
    {
    public:
        LineSupportRegion(int rows,int cols,const std::vector<uchar> &data);
        ~LineSupportRegion();

        void Detect();
        void Detect2();
        void Detect3();
//        void BackProject(const std::vector<float> &depth_vec,const Eigen::Matrix3f &K);

        void SetThres(double len_th,double wid_th)
        {
            length_thres=len_th;
            width_thres=wid_th;
        };
        
        std::vector<PCM::Line2D> GetLine() const;
        std::vector<Region2D> GetLineRegion() const;

        std::vector<PCM::Line2D> GetLeftLine() const;
        std::vector<Region2D> GetLeftLineRegion() const;

        std::vector<PCM::Line2D> GetRightLine() const;
        std::vector<Region2D> GetRightLineRegion() const;

//        std::vector<Edge> GetEdgeVec() const;
//        std::vector<Edge> GetMedianEdgeVec() const;
//        std::vector<PCM::Line2D> GetFilterLine() const;
//        std::vector<Region2D> GetFilterLineRegion() const;

        std::vector<PCM::Envelop2D> GetEnvelop() const;
        std::vector<PCM::Envelop2D> GetLeftEnvelop() const;
        std::vector<PCM::Envelop2D> GetRightEnvelop() const;

        std::vector<Region2D> GetEnvRegion() const;
        std::vector<Region2D> GetLeftEnvRegion() const;
        std::vector<Region2D> GetRightEnvRegion() const;

    private:
        Envelop2D LineRectangle(const cv::Point2d &pt_s,const cv::Point2d &pt_e,double width);

        double length_thres;
        double width_thres;
        
        int m_rows;
        int m_cols;
        std::vector<uchar> m_img;

        /// middle line
        std::vector<Line2D> m_line_vec;
        std::vector<Region2D> m_line_region_vec;

        /// left line
        std::vector<Line2D> m_left_line_vec;
        std::vector<Region2D> m_left_line_region_vec;

        /// right line
        std::vector<Line2D> m_right_line_vec;
        std::vector<Region2D> m_right_line_region_vec;

        /// line -> edge
//        std::vector<Line2D> m_filter_line_vec;
//        std::vector<Region2D> m_filter_line_region_vec;
//        std::vector<Edge> m_edge_vec;
//        std::vector<Edge> m_median_edge_vec;

        /// envelop
        std::vector<Envelop2D> m_env_vec;
        std::vector<Region2D> m_env_region_vec;

        /// left envelop
        std::vector<Envelop2D> m_left_env_vec;
        std::vector<Region2D> m_left_env_region_vec;

        /// right envelop
        std::vector<Envelop2D> m_right_env_vec;
        std::vector<Region2D> m_right_env_region_vec;

    };
}


#endif //POINTCLOUDMAP_LINESUPPORTREGION_H
