//
// Created by books on 18-1-17.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <Eigen/Core>

typedef Eigen::Matrix<float,3,4> Matrix34f;
typedef std::vector<Matrix34f,Eigen::aligned_allocator<Matrix34f>> Matrix34fVec;

Matrix34f ReadCalibFile(std::string filename)
{
    Matrix34f transform;
    //transform.setZero();

    std::ifstream ifs(filename,std::ios::in);
    if(!ifs.is_open())
        return transform;

    while(ifs.good()&&!ifs.eof())
    {
        std::string str;
        float x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12;
        ifs >> str >> x1 >> x2 >> x3 >> x4 >> x5 >> x6 >> x7 >> x8 >> x9 >> x10 >> x11 >> x12;

        if(str=="Tr:")
        {
            transform << x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12;
            break;
        }
    }
    ifs.close();

    return transform;
}

Matrix34fVec ReadPoseFile(std::string filename)
{
    Matrix34fVec vec;

    std::ifstream ifs(filename,std::ios::in);
    if(!ifs.is_open())
        return vec;

    while(ifs.good()&&!ifs.eof())
    {
        float x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12;
        ifs >> x1 >> x2 >> x3 >> x4 >> x5 >> x6 >> x7 >> x8 >> x9 >> x10 >> x11 >> x12;

        Matrix34f pose;
        pose<<x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12;

        vec.push_back(pose);
    }

    if(!vec.empty())
        vec.erase(vec.end()-1);

    return vec;

};

int main()
{
    std::string calibfile="../data/calib.txt";
    std::string posefile="../data/pose.txt";

    Matrix34f transform=ReadCalibFile(calibfile);
    std::cout<<"T ~ \n"<<transform<<std::endl;

    Eigen::Matrix3f rotation=transform.block(0,0,3,3);
    std::cout<<"R ~ \n"<<rotation<<std::endl;

    Eigen::Vector3f translation=transform.block(0,3,3,1);
    std::cout<<"t ~ \n"<<translation<<std::endl;

    Matrix34fVec poseVec=ReadPoseFile(posefile);
    for(const Matrix34f &pose : poseVec)
    {
        std::cout<<pose<<std::endl;
    }

    return 1;
}