//
// Created by whubooks on 18-4-24.
//

#include <iostream>
#include <vector>
#include <random>
#include <ctime>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/se3.hpp>

int main()
{
    std::default_random_engine e;
    e.seed(std::time(nullptr));
    std::uniform_real_distribution<> uniform(0.0, 360.0);

    Eigen::Vector3d pos(0.0, 0.0, 0.0);
    for (int i = 0; i < 3; i++)
        pos(i) = uniform(e);
    std::cout << "Pos ~ " << pos.transpose() << std::endl;

    double angle_z = uniform(e);
    Eigen::AngleAxisd angle_axis_z(angle_z * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    std::cout<<"Angle Z ~ "<<angle_z*M_PI/180.0<<std::endl;

    double angle_y = uniform(e);
    Eigen::AngleAxisd angle_axis_y(angle_y * M_PI / 180.0, Eigen::Vector3d::UnitY());
    std::cout<<"Angle Y ~ "<<angle_y*M_PI/180.0<<std::endl;

    double angle_x = uniform(e);
    Eigen::AngleAxisd angle_axis_x(angle_x * M_PI / 180.0, Eigen::Vector3d::UnitX());
    std::cout<<"Angle X ~ "<<angle_x*M_PI/180.0<<std::endl;

    Eigen::Matrix3d rotation = angle_axis_z.toRotationMatrix() * angle_axis_y.toRotationMatrix() * angle_axis_x.toRotationMatrix();
    std::cout << "Rotation ~ \n" << rotation << std::endl;

    Eigen::Vector3d translation(0.0, 0.0, 0.0);
    for (int i = 0; i < 3; i++)
        translation(i) = uniform(e);
    std::cout << "Translation ~ " << translation.transpose() << std::endl;

    Sophus::SE3d se(rotation,translation);
    std::cout<<"SE ~ \n"<<se.matrix3x4()<<std::endl;
    std::cout<<"SE Angle Z ~ "<<se.angleZ()<<std::endl;
    std::cout<<"SE Angle Y ~ "<<se.angleY()<<std::endl;
    std::cout<<"SE Angle X ~ "<<se.angleX()<<std::endl;

    Eigen::Vector3d pos_trans1 = rotation * pos + translation;
    Eigen::Vector3d pos_trans2 = rotation * (pos + translation);
    Eigen::Vector3d pos_trans3=se*pos;

    std::cout<<"****** Transform ******"<<std::endl;
    std::cout<<"Pos Rotation then Translation ~ "<<pos_trans1.transpose()<<std::endl;
    std::cout<<"Pos Translation then Rotation ~ "<<pos_trans2.transpose()<<std::endl;
    std::cout<<"Pos Sophus ~ "<<pos_trans3.transpose()<<std::endl<<std::endl;

    Eigen::Vector3d pos_inverse_trans1 = rotation.inverse() * pos - translation;
    Eigen::Vector3d pos_inverse_trans2 = rotation.inverse() * (pos - translation);
    Eigen::Vector3d pos_inverse_trans3=se.inverse()*pos;

    std::cout<<"****** Inverse Transform ******"<<std::endl;
    std::cout<<"Pos Rotation then Translation ~ "<<pos_inverse_trans1.transpose()<<std::endl;
    std::cout<<"Pos Translation then Rotation ~ "<<pos_inverse_trans2.transpose()<<std::endl;
    std::cout<<"Pos Sophus ~ "<<pos_inverse_trans3.transpose()<<std::endl<<std::endl;

    return 1;
}