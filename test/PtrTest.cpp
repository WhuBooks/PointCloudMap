//
// Created by whubooks on 18-4-5.
//

#include <iostream>
#include <vector>
#include <algorithm>
#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void AddPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointXYZ pt;
    pt.x=100;
    pt.y=100;
    pt.z=100;

    cloud->push_back(pt);

}


int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for(int i=0;i<100;i++)
    {
        AddPoint(cloud);
        std::cout<<cloud->size()<<std::endl;
    }

    return 1;
}