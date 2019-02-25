//
// Created by books on 2018/3/26.
//

#include <iostream>
#include <vector>

#include <CloudUtil.h>
#include <Util.h>

int main(int argc,char **argv)
{
    if (argc == 1)
    {
        std::cerr << "Need Pcd File Or Directory." << std::endl;
        return -1;
    }

    std::string input=std::string(argv[1]);
    
    //std::string input="50_stat_edge_index.pcd";
    std::string input_ext=Util::SplitExtention(input);

    if(input_ext=="")
    {
        std::cout << "Input Directory ~ " << input << std::endl;
        std::vector<std::string> file_vec = Util::GetFiles(input);

        std::string las_dir = Util::GetNameFromTime();
        if (input == ".")
        {
            if (!Util::DirBuild(las_dir))
            {
                std::cerr << "Failed Create Las Directory." << std::endl;
                return -1;
            }
            las_dir=las_dir+"/";
        }
        else
            las_dir = input;
        std::cout << "Las File Dir ~ " << las_dir << std::endl;
        for (const std::string &pcd_file: file_vec)
        {
            std::string pcd_file_ext = Util::SplitExtention(pcd_file);
            if (pcd_file_ext != "pcd")
            {
                std::cout << "File Is Not Pcd ~ " << pcd_file << std::endl;
                continue;
            }

            std::string las_file = las_dir + Util::SplitNameWithoutExt(pcd_file) + ".las";

            if (PCM::ConvertPcd2Las(pcd_file, las_file))
                std::cout << "Success Convert ~ " << las_file << std::endl;
            else
                std::cout << "Failed Convert ~ " << las_file << std::endl;
        }
    }
    else if(input_ext=="pcd")
    {
        std::cout<<"Input File ~ "<<input<<std::endl;
        std::string pcd_file=input;
        std::string las_file=Util::SplitNameWithoutExt(pcd_file)+".las";
        std::string pcd_dir=Util::SplitDirectory(pcd_file);
        if(!pcd_dir.empty())
            las_file=pcd_dir+"/"+las_file;
        if(PCM::ConvertPcd2Las(pcd_file,las_file))
            std::cout<<"Success Convert ~ "<<las_file<<std::endl;
        else
            std::cout<<"Failed Convert ~ "<<las_file<<std::endl;
    }
    else
    {
        std::cerr<<"Need Pcd File Or Directory."<<std::endl;
        return -1;
    }

    return 1;
}