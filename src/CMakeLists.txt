project(PointCloudMap)

add_library(UtilLib Util.h Util.cpp)

add_library(PCMLib
        PointCloudProject.h PointCloudProject.cpp
        CloudUtil.h CloudUtil.cpp
        KittiData.h KittiData.cpp
        Register.h Register.cpp
        LineSupportRegion.h LineSupportRegion.cpp
        EdgeFilter.h EdgeFilter.cpp)
#target_link_libraries(PCMLib UtilLib)