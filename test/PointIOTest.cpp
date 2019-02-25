//
// Created by books on 18-1-17.
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>
#include <list>
#include <algorithm>

struct Point
{
    int id;
    float x;
    float y;
    float z;
    float r;
};

std::vector<Point> Load_c(std::string filename)
{
    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    int32_t num = 1000000;
    float *data = (float *) malloc(num * sizeof(float));

    // pointers
    float *px = data + 0;
    float *py = data + 1;
    float *pz = data + 2;
    float *pr = data + 3;

    std::vector<Point> ptVec;
    int id = 0;

    // load point cloud
    FILE *stream;
    stream = fopen(filename.c_str(), "rb");
    if(!stream)
        return ptVec;

    num = fread(data, sizeof(float), num, stream) / 4;
    for (int32_t i = 0; i < num; i++)
    {
        Point pt;
        pt.id = id++;
        pt.x = *px;
        pt.y = *py;
        pt.z = *pz;
        pt.r = *pr;

        ptVec.push_back(pt);
        px += 4;
        py += 4;
        pz += 4;
        pr += 4;
    }
    fclose(stream);

    return ptVec;
}

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

void Save(std::string filename,const std::vector<Point> &ptVec)
{
    std::ofstream ofs(filename,std::ios::out);
    ofs << std::setiosflags(std::ios::fixed);
    for (const Point &pt : ptVec)
    {
        ofs << pt.id << "\t";
        ofs << std::setprecision(5) << pt.x << "\t";
        ofs << std::setprecision(5) << pt.y << "\t";
        ofs << std::setprecision(5) << pt.z << "\t";
        ofs << std::setprecision(5) << pt.r << std::endl;
    }
    ofs.flush();
    ofs.close();
}

bool Compare(const std::vector<Point> &vec1,const std::vector<Point> &vec2)
{
    for(std::size_t i=0;i<vec1.size();i++)
    {
        Point pt1=vec1[i];
        Point pt2=vec2[i];

        if(pt1.x!=pt2.x || pt1.y!=pt2.y || pt1.z!=pt2.z || pt1.r!=pt2.r)
            return false;
    }

    return true;
}

std::vector<float> UniqueReflectance(const std::vector<Point> &frame)
{
    std::vector<float> vec;
    for(const Point &pt : frame)
    {
        float r=pt.r;
        bool flag=false;
        for(const float &tmp: vec)
        {
            flag=flag||(tmp==r);
        }
        if(!flag)
            vec.push_back(r);
    }
    return vec;
}

int main()
{
    std::string filename = "../data/000000.bin";

    std::vector<Point> ptVec_cpp=Load_cpp(filename);
    std::string name_cpp="PointCloud_cpp.txt";
    Save(name_cpp,ptVec_cpp);

    std::vector<Point> ptVec_c=Load_c(filename);
    std::string name_c="PointCloud_c.txt";
    Save(name_c,ptVec_c);

    bool flag=Compare(ptVec_c,ptVec_cpp);
    std::cout<<"Compare C and Cpp result ~ "<<flag<<std::endl;

    std::vector<float> rVec=UniqueReflectance(ptVec_cpp);
    std::sort(rVec.begin(),rVec.end());
    std::cout<<std::setiosflags(std::ios::fixed);
    for(const float &r : rVec)
    {
        std::cout<<std::setprecision(3)<<r<<std::endl;
    }


    return 1;
}
