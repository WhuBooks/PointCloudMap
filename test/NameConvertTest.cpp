//
// Created by books on 18-1-17.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

#include <Util.h>

std::vector<double> ReadFile(std::string file)
{
    std::vector<double> vec;
    std::ifstream ifs(file);
    
    while(ifs.good() && !ifs.eof())
    {
        double val=-1;
        ifs>>val;
        vec.push_back(val);
    }
    return vec;
}

bool EqualVec(std::vector<double> vec1,std::vector<double> vec2)
{
    if(vec1.size()!=vec2.size())
        return false;
    
    for(int i=0;i<vec1.size();i++)
    {
        if(vec1[i]!=vec2[i])
            return false;
    }
    return true;
}

int main()
{
//    std::string dir="/home/books/00/velodyne";
//    std::vector<std::string> vfile=Util::GetFiles(dir);
//
//    for(const std::string &filename : vfile)
//    {
//        std::string tmp=Util::SplitNameWithoutExt(filename);
//
//        std::stringstream ss;
//        ss<<tmp;
//
//        int val=0;
//        ss>>val;
//
//        std::cout<<val<<std::endl;
//    }
    
    std::string file1="lidar_project4.txt";
    std::string file2="pcm_depth.txt";
    
    std::vector<double> vec1=ReadFile(file1);
    std::vector<double> vec2=ReadFile(file2);
    
    std::cout<<"Equal ~ "<<EqualVec(vec1,vec2)<<std::endl;

    return 1;
}