//
// Created by whubooks on 18-3-19.
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <random>
#include <deque>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

struct EarPt
{
    float x;
    float y;
    float z;

    float x_normal;
    float y_normal;
    float z_normal;

    float intensity;

    EarPt()
    {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        x_normal = 0.0f;
        y_normal = 0.0f;
        z_normal = 0.0f;
        intensity = 0.0f;
    }
    EarPt(float _x,float _y,float _z,float _x_normal,float _y_normal,float _z_normal,float _intensity)
    {
        x = _x;
        y = _y;
        z = _z;
        x_normal = _x_normal;
        y_normal = _y_normal;
        z_normal = _z_normal;
        intensity = _intensity;
    }
    EarPt(const pcl::PointXYZINormal &pt)
    {
        x = pt.x;
        y = pt.y;
        z = pt.z;
        x_normal = pt.normal_x;
        y_normal = pt.normal_y;
        z_normal = pt.normal_z;
        intensity = pt.intensity;
    }

    pcl::PointXYZINormal ToPclPt() const
    {
        pcl::PointXYZINormal pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        pt.intensity = intensity;
        pt.normal_x = x_normal;
        pt.normal_y = y_normal;
        pt.normal_z = z_normal;
        return pt;
    }

    Eigen::Vector3f Pos() const
    {
        return Eigen::Vector3f(x,y,z);
    }
    Eigen::Vector3f Normal() const
    {
        return Eigen::Vector3f(x_normal,y_normal,z_normal);
    }



};
typedef std::vector<EarPt> EarPtVec;

EarPtVec KnnSearch(const EarPtVec &vec,const EarPt &pt,int k)
{
    typedef std::pair<EarPt, double> pair_dis;
    std::vector<std::pair<EarPt, double>> map;
    for (const EarPt &tmp : vec)
    {
        if (tmp.x == pt.x && tmp.y == pt.y && tmp.z == pt.z && tmp.intensity == pt.intensity &&
            tmp.x_normal == pt.x_normal && tmp.y_normal == pt.y_normal && tmp.z_normal == pt.z_normal)
            continue;

        double dis =
                (pt.x - tmp.x) * (pt.x - tmp.x) + (pt.y - tmp.y) * (pt.y - tmp.y) + (pt.z - tmp.z) * (pt.z - tmp.z);
        map.push_back(std::make_pair(tmp, dis));
    }

    std::sort(map.begin(), map.end(), [](const pair_dis &x1, const pair_dis &x2) {
        return x1.second < x2.second;
    });

    EarPtVec result;
    for (int i = 0; i < k; i++)
        result.push_back(map[i].first);
    return result;
}

struct EarBase
{
    EarPt pt;
    EarPt pt_neighbour;
    float priority;
};
typedef std::vector<EarBase> EarBaseVec;

int main()
{
    std::string pcdfile = "";
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::io::loadPCDFile(pcdfile, *cloud);

    EarPtVec ear_pt_vec_j;
    for (const pcl::PointXYZINormal &pt : cloud->points)
        ear_pt_vec_j.push_back(EarPt(pt));
    std::cout << "Size ~ " << ear_pt_vec_j.size() << std::endl;

    const int k = 16;
    //base selection
    EarBaseVec ear_base_vec_j;
    for (const EarPt &pt : ear_pt_vec_j)
    {
        EarPtVec ear_pt_vec_knn = KnnSearch(ear_pt_vec_j, pt, k);

        float minD = 1000000, maxDNor = -1000000;
        EarPt neighbour;
        for (const EarPt &pt_knn : ear_pt_vec_knn)
        {
            Eigen::Vector3f b = (pt.Pos() + pt_knn.Pos()) / 2.0f;
            float D = (b - pt_knn.Pos() - pt_knn.Normal().transpose() * (b - pt_knn.Pos()) * pt_knn.Normal()).norm();
            float D_normal = (2.0f - pt.Normal().transpose() * pt_knn.Normal());

            maxDNor = (maxDNor > D_normal) ? maxDNor : D_normal;
            if (minD < D)
            {
                neighbour = pt_knn;
                minD = D;
            }
        }

        EarBase base_pt;
        base_pt.pt = pt;
        base_pt.pt_neighbour = neighbour;
        base_pt.priority = maxDNor;
        ear_base_vec_j.push_back(base_pt);
    }

    std::sort(ear_base_vec_j.begin(), ear_base_vec_j.end(),
              [](const EarBase &x, const EarBase &y) { return x.priority < y.priority; });
    EarBase best_ear_base=ear_base_vec_j.back();
}