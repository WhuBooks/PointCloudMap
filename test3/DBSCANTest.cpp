//
// Created by books on 2018/5/21.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>

#include <Util.h>
#include <EdgeFilter.h>
#include <CloudUtil.h>

static int pt_id=0;
struct Pt
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int id;
    int edge_id;
    Eigen::Vector3f pos;
    Eigen::Vector3f dir;

    float Distance(const Pt &other) const
    {
        return (other.pos-pos).norm();
    }

    float DirAngle(const Pt &other) const
    {
        return std::acos(dir.dot(other.dir)/(dir.norm()*other.dir.norm()))*180.0f/M_PI;
    }

};
typedef std::vector<Pt> PtVec;

struct Kernel
{
    int pt_id;
    std::vector<int> direct_density_ids;

    bool Contain(const int &other_id) const
    {
        for(const int &id : direct_density_ids)
        {
            if(id==other_id)
                return true;
        }
        return false;
    }
};

PtVec ReadPts(std::string filename)
{
    std::ifstream ifs(filename);
    if(!ifs.is_open())
        return PtVec();
    PtVec edge_pts;
    while(ifs.good()&&!ifs.eof())
    {
        int edge_id;
        float x,y,z;
        float dx,dy,dz;
        ifs>>edge_id>>x>>y>>z>>dx>>dy>>dz;

        Pt pt;
        pt.id=pt_id++;
        pt.edge_id=edge_id;
        pt.pos=Eigen::Vector3f(x,y,z);
        pt.dir=Eigen::Vector3f(dx,dy,dz);
        edge_pts.push_back(pt);
    }

    if(!edge_pts.empty())
        edge_pts.erase(edge_pts.end()-1);

    return edge_pts;
}

int main()
{
    std::string single_file = "single_edge_pt.txt";
    PtVec single_pts = ReadPts(single_file);
    std::string double_file = "double_edge_pt.txt";
    PtVec double_pts = ReadPts(double_file);

    PtVec all_pts;
    all_pts.insert(all_pts.end(), single_pts.begin(), single_pts.end());
    all_pts.insert(all_pts.end(), double_pts.begin(), double_pts.end());

    std::vector<Kernel> kernels;
    for (const Pt &pt : all_pts)
    {
        Eigen::Vector3f pt_dir = pt.dir;

        PtVec search_vec;
        std::vector<int> id_vec;
        for (const Pt &tmp : all_pts)
        {
            if (tmp.id == pt.id || tmp.edge_id == pt.edge_id)
                continue;
            if (tmp.Distance(pt) > 0.5f)
                continue;
            if (tmp.DirAngle(pt) > 30.0f)
                continue;

            search_vec.push_back(tmp);
            id_vec.push_back(tmp.id);
        }

        if (search_vec.size() < 3)
            continue;

        Kernel kernel;
        kernel.pt_id = pt.id;
        kernel.direct_density_ids = id_vec;
        kernels.push_back(kernel);
    }
    std::cout << "Kernel Size ~ " << kernels.size() << std::endl;

    std::vector<std::vector<Kernel>> clusters;
    while (!kernels.empty())
    {
        /// find density arrive kernels of first kernel
        std::vector<Kernel> density_arrive_kernels;
        density_arrive_kernels.push_back(kernels.front());

        std::vector<Kernel> update_kernels;
        bool find_kernel_flag = true;
        while (find_kernel_flag)
        {
            std::vector<Kernel> tmp_kernels;
            for (const Kernel &kernel : kernels)
            {
                for (const Kernel &tmp_kernel : density_arrive_kernels)
                {
                    if (kernel.pt_id == tmp_kernel.pt_id)
                        continue;
                    update_kernels.push_back(kernel);
                    if (kernel.Contain(tmp_kernel.pt_id))
                    {
                        tmp_kernels.push_back(kernel);
                        break;
                    }
                }
            }
            find_kernel_flag = tmp_kernels.size() != density_arrive_kernels.size();
            if (find_kernel_flag)
                update_kernels.clear();
            density_arrive_kernels.swap(tmp_kernels);
        }

        /// update kernels
        kernels.swap(update_kernels);
        clusters.push_back(density_arrive_kernels);
    }
    std::cout << "Cluster Size ~ " << clusters.size() << std::endl;

    std::string dir = Util::GetNameFromTime();
    Util::DirBuild(dir);
    for (int i = 0; i < clusters.size(); i++)
    {
        std::vector<Kernel> cluster = clusters[i];
        PCM::CloudXYZPtr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const Kernel &kernel : cluster)
        {
            for (const Pt &pt : all_pts)
            {
                if (pt.id == kernel.pt_id)
                {
                    cloud_cluster->push_back(pcl::PointXYZ(pt.pos[0], pt.pos[1], pt.pos[2]));
                    break;
                }
            }
        }

        std::string filename = dir + "/" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileBinary(filename, *cloud_cluster);
    }

    return 1;
}