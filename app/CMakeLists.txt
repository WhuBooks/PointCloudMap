project(PointCloudMap)

add_executable(RegisterByIMU RegisterByIMU.cpp)
target_link_libraries(RegisterByIMU UtilLib PCMLib)

add_executable(Pcd2Las Pcd2Las.cpp)
target_link_libraries(Pcd2Las UtilLib PCMLib)

add_executable(LidarFilter LidarFilter.cpp)
target_link_libraries(LidarFilter UtilLib PCMLib)

#add_executable(RemoveGround RemoveGround.cpp)
#target_link_libraries(RemoveGround UtilLib PCMLib)

add_executable(ToImage ToImage.cpp)
target_link_libraries(ToImage UtilLib PCMLib)

add_executable(EdgeExtract EdgeExtract.cpp)
target_link_libraries(EdgeExtract UtilLib PCMLib)
