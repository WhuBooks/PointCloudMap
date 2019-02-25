//
// Created by books on 18-1-17.
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

struct Point
{
    int id;
    float x;
    float y;
    float z;
    float r;
};

std::vector<Point> Load_cpp(std::string filename)
{
    std::vector<Point> ptVec;
    std::ifstream ifs(filename, std::ios::in | std::ios::binary);
    if (!ifs.is_open())
        return ptVec;

    int id = 0;
    while (ifs.good() && !ifs.eof())
    {
        Point pt;
        pt.id = id++;
        ifs.read(reinterpret_cast<char *>(&pt.x), sizeof(float));
        ifs.read(reinterpret_cast<char *>(&pt.y), sizeof(float));
        ifs.read(reinterpret_cast<char *>(&pt.z), sizeof(float));
        ifs.read(reinterpret_cast<char *>(&pt.r), sizeof(float));

        ptVec.push_back(pt);
    }
    ifs.close();

    if(!ptVec.empty())
        ptVec.erase(ptVec.end()-1);

    return ptVec;
}

int main(int argc,char **argv)
{
    std::string filename = std::string(argv[1]);
    std::cout<<"Load File ~ "<<"filename"<<std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(filename,*cloud);

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    //showCloud函数是同步的，在此处等待直到渲染显示为止
    viewer.showCloud(cloud);
    int user=0;
    while (!viewer.wasStopped ())
    {
        user++;
    }
    return 0;


}