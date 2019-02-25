//
// Created by whubooks on 18-3-20.
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

#include <liblas/liblas.hpp>

#include <Util.h>

struct PointM
{
    double x;
    double y;
    double z;
    float intensity;
};

int main(int argc,char **argv)
{
    if(argc==1)
    {
        std::cerr<<"Need File Input!"<<std::endl;
        return -1;
    }

    std::string pcdfile=std::string(argv[1]);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile,*cloud);

    std::vector<PointM> vector;
    float minI=10000000,maxI=-1000000000;
    for(const pcl::PointXYZI &pt : cloud->points)
    {
        PointM tmp;
        tmp.x=pt.x;
        tmp.y=pt.y;
        tmp.z=pt.z;
        tmp.intensity=pt.intensity;
        minI=minI<tmp.intensity?minI:tmp.intensity;
        maxI=maxI>tmp.intensity?maxI:tmp.intensity;
        vector.push_back(tmp);
    }
    std::cout<<"Point Size ~ "<<vector.size()<<std::endl;

    std::string lasfile=Util::SplitNameWithoutExt(pcdfile)+".las";
    std::ofstream ofs(lasfile);

    liblas::Header header;
    header.SetDataFormatId(liblas::ePointFormat1); // Time only

    header.SetScale(0.01,0.01,0.01);

    // Set coordinate system using GDAL support
    liblas::SpatialReference srs;
    srs.SetFromUserInput("EPSG:4326");
    header.SetSRS(srs);
    liblas::Writer writer(ofs,header);

    for(const PointM &pt : vector)
    {
        liblas::Point tmp(&header);

        tmp.SetCoordinates(pt.x,pt.y,pt.z);
        uint16_t intensity= static_cast<uint16_t >((pt.intensity-minI)/(maxI-minI)*65536);
        tmp.SetIntensity(intensity);

        writer.WritePoint(tmp);
    }

    return 1;

//    std::string filename="armadillo.txt";
//    std::ifstream ifs(filename,std::ios::in);
//
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
//    while(ifs.good()&&!ifs.eof())
//    {
//        float x=0,y=0,z=0;
//        ifs>>x>>y>>z;
//
//        pcl::PointXYZI pt;
//        pt.x=x;
//        pt.y=y;
//        pt.z=z;
//        pt.intensity=10;
//        cloud->push_back(pt);
//    }
//
//    pcl::io::savePCDFileASCII("conv.pcd",*cloud);

}