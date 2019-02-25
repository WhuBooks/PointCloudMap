//
// Created by books on 2018/5/19.
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

void WriteLas(const PtVec &vec,std::string file)
{
    PCM::CloudXYZPtr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(const Pt &p : vec)
        cloud->push_back(pcl::PointXYZ(p.pos[0],p.pos[1],p.pos[2]));

    PCM::WriteLas(cloud,file);
}


int main()
{
    std::string single_file="single_edge_pt.txt";
    PtVec single_pts=ReadPts(single_file);
    std::string double_file="double_edge_pt.txt";
    PtVec double_pts=ReadPts(double_file);

    PtVec all_pts;
    all_pts.insert(all_pts.end(),single_pts.begin(),single_pts.end());
    all_pts.insert(all_pts.end(),double_pts.begin(),double_pts.end());

    PtVec filter_pts;
    for(const Pt &pt : all_pts)
    {
        Eigen::Vector3f pt_dir=pt.dir;

        PtVec search_vec;
        for(const Pt &tmp : all_pts)
        {
            if(tmp.id==pt.id || tmp.edge_id==pt.edge_id)
                continue;
            if(tmp.Distance(pt)>0.5)
                continue;
            if(tmp.DirAngle(pt)>30)
                continue;

            search_vec.push_back(tmp);
        }

        if(search_vec.size()>3)
            filter_pts.push_back(pt);

    }

    std::cout<<"Filter Size ~ "<<filter_pts.size()<<std::endl;

    std::string filter_file="filter_edge.las";
    WriteLas(filter_pts,filter_file);
}