//
// Created by whubooks on 18-3-25.
//

#include "KittiData.h"

namespace PCM
{

    KittiData::KittiData(const std::string &dir)
    {
        base_dir=dir;
    }

    KittiData::~KittiData()
    {

    }

    void KittiData::SetSubNum(int K)
    {
        std::string str_K;
        if(K<10)
            str_K="0"+std::to_string(K);
        else
            str_K=std::to_string(K);
        pose_dir=base_dir+"/data_odometry_poses/poses/";
        calib_dir=base_dir+"/data_odometry_calib/sequences/"+str_K+"/";
        velo_dir=base_dir+"/data_odometry_velodyne/sequences/"+str_K+"/velodyne/";
        img_dir=base_dir+"/data_odometry_gray/sequences/"+str_K+"/image_0/";


        std::string pose_file=pose_dir+str_K+".txt";
        std::string calib_file=calib_dir+"calib.txt";

        /// load calib file
        Eigen::Matrix<float,3,4> transform;
        std::ifstream ifs_calib(calib_file,std::ios::in);
        while(ifs_calib.good()&&!ifs_calib.eof())
        {
            std::string str;
            double x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12;
            ifs_calib >> str >> x1 >> x2 >> x3 >> x4 >> x5 >> x6 >> x7 >> x8 >> x9 >> x10 >> x11 >> x12;

            if(str=="Tr:")
            {
                transform << x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12;
            }

            if(str=="P0:")
            {
                m_K<<x1,x2,x3,x5,x6,x7,x9,x10,x11;
            }
        }
        ifs_calib.close();

        Eigen::Matrix3f rotation = transform.block(0, 0, 3, 3);
        Eigen::Vector3f translation = transform.block(0, 3, 3, 1);
        m_calib=Sophus::SE3f(rotation,translation);

        /// load pose file
        std::ifstream ifs_pose(pose_file,std::ios::in);
        while(ifs_pose.good()&&!ifs_pose.eof())
        {
            double x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12;
            ifs_pose >> x1 >> x2 >> x3 >> x4 >> x5 >> x6 >> x7 >> x8 >> x9 >> x10 >> x11 >> x12;

            Eigen::Matrix<float,3,4> pose;
            pose<<x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12;

            Eigen::Matrix3f r = pose.block(0, 0, 3, 3);
            Eigen::Vector3f t = pose.block(0, 3, 3, 1);
            m_pose_vec.push_back(Sophus::SE3f(r,t));
        }

        if(!m_pose_vec.empty())
            m_pose_vec.erase(m_pose_vec.end()-1);

        m_size=(int)m_pose_vec.size();
    }

    int KittiData::GetSize() const
    {
        return m_size;
    }

    std::string GetNumStr(int num)
    {
        std::string num_str=std::to_string(num);
        if(num<10)
            return "00000"+num_str;
        if(num<100)
            return "0000"+num_str;
        if(num<1000)
            return "000"+num_str;
        if(num<10000)
            return "00"+num_str;
        if(num<100000)
            return "0"+num_str;
        return num_str;
    }

    cv::Mat KittiData::GetImage(int num)
    {
        std::string img_name=img_dir+GetNumStr(num)+".png";
        cv::Mat img=cv::imread(img_name,cv::IMREAD_UNCHANGED);
        return img;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr KittiData::GetCloud(int num)
    {
        std::string pcd_name=velo_dir+GetNumStr(num)+".bin";

        std::vector<float> vx,vy,vz,vr;
        std::ifstream ifs(pcd_name, std::ios::in | std::ios::binary);

        while (ifs.good() && !ifs.eof())
        {
            float x=0.0f,y=0.0f,z=0.0f,r=0.0f;
            ifs.read(reinterpret_cast<char *>(&x), sizeof(float));
            ifs.read(reinterpret_cast<char *>(&y), sizeof(float));
            ifs.read(reinterpret_cast<char *>(&z), sizeof(float));
            ifs.read(reinterpret_cast<char *>(&r), sizeof(float));

            vx.push_back(x);
            vy.push_back(y);
            vz.push_back(z);
            vr.push_back(r);
        }
        ifs.close();

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for(int i=0;i<vx.size()-1;i++)
        {
            Eigen::Vector3f pos(vx[i],vy[i],vz[i]);
            Eigen::Vector3f pos_camera=m_calib*pos;

            pcl::PointXYZI pt;
            pt.x=pos_camera(0);
            pt.y=pos_camera(1);
            pt.z=pos_camera(2);
            pt.intensity=vr[i];
            cloud->push_back(pt);
        }
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr KittiData::GetCloudWithDis(int num)
    {
        std::string pcd_name=velo_dir+GetNumStr(num)+".bin";

        std::vector<float> vx,vy,vz,vr;
        std::ifstream ifs(pcd_name, std::ios::in | std::ios::binary);

        while (ifs.good() && !ifs.eof())
        {
            float x=0.0f,y=0.0f,z=0.0f,r=0.0f;
            ifs.read(reinterpret_cast<char *>(&x), sizeof(float));
            ifs.read(reinterpret_cast<char *>(&y), sizeof(float));
            ifs.read(reinterpret_cast<char *>(&z), sizeof(float));
            ifs.read(reinterpret_cast<char *>(&r), sizeof(float));

            vx.push_back(x);
            vy.push_back(y);
            vz.push_back(z);
            vr.push_back(r);
        }
        ifs.close();

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        for(int i=0;i<vx.size()-1;i++)
        {
            Eigen::Vector3f pos(vx[i],vy[i],vz[i]);
            Eigen::Vector3f pos_camera=m_calib*pos;

            pcl::PointXYZINormal pt;
            pt.x=pos_camera(0);
            pt.y=pos_camera(1);
            pt.z=pos_camera(2);
            pt.intensity=vr[i];
            pt.curvature=pos.norm();
            cloud->push_back(pt);
        }
        return cloud;
    }

    Sophus::SE3f KittiData::GetCalib()
    {
        return m_calib;
    }

    Sophus::SE3f KittiData::GetPose(int num)
    {
        return m_pose_vec[num];
    }

    Eigen::Matrix3d KittiData::GetK()
    {
        return m_K;
    }

    Eigen::Matrix3f KittiData::LoadKFromFile(const std::string filename)
    {
        /// load calib file
        Eigen::Matrix3f K;
        K.setZero();
        std::ifstream ifs_calib(filename,std::ios::in);
        while(ifs_calib.good()&&!ifs_calib.eof())
        {
            std::string str;
            double x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12;
            ifs_calib >> str >> x1 >> x2 >> x3 >> x4 >> x5 >> x6 >> x7 >> x8 >> x9 >> x10 >> x11 >> x12;

            if(str=="P0:")
            {
                K<<x1,x2,x3,x5,x6,x7,x9,x10,x11;
            }
        }
        ifs_calib.close();
        return K;
    }


}