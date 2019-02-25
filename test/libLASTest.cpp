//
// Created by books on 18-1-17.

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <random>
#include <chrono>

#include <liblas/liblas.hpp>

int main()
{
    std::ofstream ofs("test.las",std::ios::out|std::ios::binary);
    if(!ofs.is_open())
        return -1;

    liblas::Header header;
    header.SetDataFormatId(liblas::ePointFormat1); // Time only

    // Set coordinate system using GDAL support
    liblas::SpatialReference srs;
    srs.SetFromUserInput("EPSG:4326");
    header.SetSRS(srs);

    liblas::Writer writer(ofs,header);

    std::default_random_engine e;
    e.seed(std::time(nullptr));
    std::normal_distribution<double> normalDistribution(0.0f,50.0f);

    std::chrono::steady_clock::time_point tp1=std::chrono::steady_clock::now();

    for(int i=0;i<1000;i++)
    {
        double x=normalDistribution(e);
        double y=normalDistribution(e);
        double z=normalDistribution(e);

        liblas::Point pt(&header);

        pt.SetCoordinates(x,y,z);
        pt.SetIntensity(10);

        std::chrono::steady_clock::time_point tp2=std::chrono::steady_clock::now();
        std::chrono::milliseconds milli=std::chrono::duration_cast<std::chrono::milliseconds>(tp2-tp1);
        double span=milli.count();
        pt.SetTime(span);

        writer.WritePoint(pt);
    }



    return 1;
}