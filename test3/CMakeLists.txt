project(PointCloudMap)

add_executable(DensityFilterTest DensityFilterTest.cpp)
target_link_libraries(DensityFilterTest PCMLib UtilLib)

add_executable(DBSCANTest DBSCANTest.cpp)
target_link_libraries(DBSCANTest PCMLib UtilLib)
