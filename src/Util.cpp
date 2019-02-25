//
// Created by books on 18-1-17.
//

#include "Util.h"

namespace Util
{
    std::string GetNameFromTime()
    {
        time_t now = std::time(nullptr);
        std::tm *t = std::localtime(&now);
        std::string str;
        str += std::to_string(t->tm_year);
        str += "_";
        str += std::to_string(t->tm_mon);
        str += "_";
        str += std::to_string(t->tm_mday);
        str += "_";
        str += std::to_string(t->tm_hour);
        str += "_";
        str += std::to_string(t->tm_min);
        str += "_";
        str += std::to_string(t->tm_sec);
        return std::string(str);
    }

    //return directory exist or not
    bool DirExist(std::string path)
    {
#ifdef WIN32
        int accessflag=_access(path.c_str(),0);
		if(accessflag!=0)
			std::cout<<"[ FileIO - DirExist ] ~ "<<path<<std::endl;
		return accessflag==0;
#else
        int accessflag = access(path.c_str(), F_OK);
        if (accessflag != 0)
            std::cout << "[ FileIO - DirExist ] ~ " << path << std::endl;
        return accessflag == 0;
#endif
    }

    //create new directory
    bool DirBuild(std::string path)
    {
#ifdef WIN32
        int buildflag=_mkdir(path.c_str());
		if(buildflag!=0)
			std::cout<<"[ FileIO - DirBuild ] ~ "<<path<<std::endl;
		return buildflag==0;
#else

        int buildflag = mkdir(path.c_str(), S_IRWXU);
        if (buildflag != 0)
            std::cout << "[ FileIO - DirBuild ] ~ " << path << std::endl;
        return buildflag == 0;
#endif
    }

    std::vector<std::string> GetFiles(const std::string &path)
    {
        std::vector<std::string> files;//存放文件名

#ifdef WIN32

        long hFile=0;
		struct _finddata_t fileinfo;
		std::string p;
		 if((hFile=_findfirst(p.assign(path).append("\\*").c_str(),&fileinfo))!=-1)
		 {
			 _findnext(hFile,&fileinfo);
			 do{
				   if (strcmp(fileinfo.name, ".") == 0 || strcmp(fileinfo.name, "..") == 0)
					 continue;
				 files.push_back(path+"/"+p.assign((fileinfo.name)));
			 }while(_findnext(hFile,&fileinfo)==0);
			 _findclose(hFile);
		 }

#else

        DIR *dir;
        struct dirent *ptr;
        char base[1000];

        if ((dir = opendir(path.c_str())) == nullptr)
        {
            perror("Open dir error...");
            exit(1);
        }

        while ((ptr = readdir(dir)) != nullptr)
        {
            //current dir OR parrent dir
            if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
                continue;
            //file
            if (ptr->d_type == 8)
                files.push_back(path + "/" + std::string(ptr->d_name));
                //link file
            else if (ptr->d_type == 10)
                continue;
                //dir
            else if (ptr->d_type == 4)
            {
                continue;
                //files.push_back(std::string(ptr->d_name));
            }
        }
        closedir(dir);
#endif

        //排序，按从小到大排序
        std::sort(files.begin(), files.end());
        return files;
    }

    std::vector<std::string> GetSubDirectory(std::string root)
    {
        std::vector<std::string> files;//存放文件名

#ifdef WIN32

        long hFile=0;
		struct _finddata_t fileinfo;
		std::string p;
		 if((hFile=_findfirst(p.assign(root).append("\\*").c_str(),&fileinfo))!=-1)
		 {
			 _findnext(hFile,&fileinfo);
			 do{
				   if (strcmp(fileinfo.name, ".") == 0 || strcmp(fileinfo.name, "..") == 0)
					 continue;
				 files.push_back(root+"/"+p.assign((fileinfo.name)));
			 }while(_findnext(hFile,&fileinfo)==0);
			 _findclose(hFile);
		 }

#else

        DIR *dir;
        struct dirent *ptr;
        char base[1000];

        if ((dir = opendir(root.c_str())) == nullptr)
        {
            perror("Open dir error...");
            exit(1);
        }

        while ((ptr = readdir(dir)) != nullptr)
        {
            //current dir OR parrent dir
            if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
                continue;
            //file
            if (ptr->d_type == 8)
                continue;
                //files.push_back(root+"/"+std::string(ptr->d_name));
                //link file
            else if (ptr->d_type == 10)
                continue;
                //dir
            else if (ptr->d_type == 4)
                files.push_back(root + "/" + std::string(ptr->d_name));
            //continue;
        }
        closedir(dir);
#endif

        //排序，按从小到大排序
        std::sort(files.begin(), files.end());
        return files;
    }

    std::string SplitDirectory(const std::string &path)
    {
        std::size_t pos = path.find_last_of("/\\");
        if (pos >= path.size())
            return std::string();
        return path.substr(0, pos);
    }

    std::string SplitNameWithExt(const std::string &path)
    {
        std::size_t pos = path.find_last_of("/\\");
        if (pos >= path.size())
            return path;
        return path.substr(pos + 1);
    }

    std::string SplitNameWithoutExt(const std::string &path)
    {
        std::string file = SplitNameWithExt(path);
        std::size_t pos2 = file.find_last_of('.');
        if (pos2 > path.size())
            return file;
        return file.substr(0, pos2);
    }

    std::string SplitExtention(const std::string &path)
    {
        std::size_t pos = path.find_last_of('.');
        if (pos >= path.size())
            return std::string();
        return path.substr(pos + 1);
    }


}