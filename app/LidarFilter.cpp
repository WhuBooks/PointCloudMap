//
// Created by whubooks on 18-4-5.
//

#include <iostream>
#include <vector>
#include <string>

#include <CloudUtil.h>
#include <Util.h>

int main(int argc,char **argv)
{
    if(argc==1)
    {
        std::cerr<<"Need Input Pcd File!"<<std::endl;
        return -1;
    }

    std::string pcdfile=std::string(argv[1]);
    std::string filename=Util::SplitNameWithoutExt(pcdfile);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile,*cloud);
    std::cout<<"Origin Cloud Size ~ "<<cloud->size()<<std::endl;

    PCM::RansacRemoveGround(cloud);
    std::cout<<"Plane Cloud Size ~ "<<cloud->size()<<std::endl;
    std::string planefile=filename+"_plane.pcd";
    pcl::io::savePCDFileBinary(planefile,*cloud);
    
//    PCM::ConditionFilter(cloud);
//    std::cout<<"Condition Cloud Size ~ "<<cloud->size()<<std::endl;
//    std::string condfile=filename+"_cond.pcd";
//    pcl::io::savePCDFileASCII(condfile,*cloud);

    PCM::StatisticFilter(cloud);
    std::cout<<"Statistic Cloud Size ~ "<<cloud->size()<<std::endl;
    std::string statfile=filename+"_stat.pcd";
    pcl::io::savePCDFileBinary(statfile,*cloud);

//    PCM::VoxelFilter(cloud);
//    std::cout<<"Voxel Cloud Size ~ "<<cloud->size()<<std::endl;
//    std::string voxelfile=filename+"_voxel.pcd";
//    pcl::io::savePCDFileASCII(voxelfile,*cloud);
    
    PCM::MlsFilter(cloud);
    std::string mlsfile=filename+"_mls.pcd";
    pcl::io::savePCDFileBinary(mlsfile,*cloud);

    return 1;

}
