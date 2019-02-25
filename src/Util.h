//
// Created by books on 18-1-17.
//

#ifndef BOOKS_UTIL_H
#define BOOKS_UTIL_H

#include <string.h>
#include <vector>
#include <fstream>
#include <ctime>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <iomanip>
#include <algorithm>

#ifdef WIN32
#include <direct.h>
#include <io.h>
#else
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#endif


namespace Util
{
    std::string GetNameFromTime();

    bool DirExist(std::string path);

    bool DirBuild(std::string path);

    std::vector<std::string> GetFiles(const std::string &path);
    std::vector<std::string> GetSubDirectory(std::string root);

    std::string SplitDirectory(const std::string &path);

    std::string SplitNameWithExt(const std::string &path);

    std::string SplitNameWithoutExt(const std::string &path);

    std::string SplitExtention(const std::string &path);

}


#endif //BOOKS_UTIL_H
