project(PointCloudMap)

#add_executable(libLASTest libLASTest.cpp)
#target_link_libraries(libLASTest ${libLAS_LIBRARIES})

#add_executable(PointIOTest PointIOTest.cpp)

add_executable(PointDisplayTest PointDisplayTest.cpp)
#target_link_libraries(PointDisplayTest ${PCL_LIBRARIES})

#add_executable(RegisterTest RegisterTest.cpp)

add_executable(RegisterTest2 RegisterTest2.cpp)
target_link_libraries(RegisterTest2 UtilLib)

#add_executable(PoseTest PoseTest.cpp)

add_executable(NameConvertTest NameConvertTest.cpp)

add_executable(PtrTest PtrTest.cpp)

add_executable(LSDTest LSDTest.cpp)

add_executable(LidarProjectTest LidarProjectTest.cpp)

add_executable(LidarProjectTest2 LidarProjectTest2.cpp)

add_executable(LidarProjectTest3 LidarProjectTest3.cpp)

add_executable(LidarProjectTest4 LidarProjectTest4.cpp)

add_executable(LidarProjectTest5 LidarProjectTest5.cpp)
target_link_libraries(LidarProjectTest5 PCMLib)

add_executable(LidarFilterTest LidarFilterTest.cpp)

add_executable(Pcd2LasTest Pcd2LasTest.cpp)
target_link_libraries(Pcd2LasTest UtilLib)

#add_executable(EARTest EARTest.cpp)

add_executable(UpSamplingTest UpSamplingTest.cpp)

add_executable(MLSSmoothTest MLSSmoothTest.cpp)
target_link_libraries(MLSSmoothTest UtilLib)

add_executable(SegmentationTest SegmentationTest.cpp)

add_executable(SegmentationTest2 SegmentationTest2.cpp)

add_executable(SegmentationTest3 SegmentationTest3.cpp)
target_link_libraries(SegmentationTest3 UtilLib)

add_executable(SegmentationTest4 SegmentationTest4.cpp)
target_link_libraries(SegmentationTest4 UtilLib)

add_executable(RangeImageTest RangeImageTest.cpp)

#add_executable(ExtractGroundTest ExtractGroundTest.cpp)

add_executable(FindEdgeTest FindEdgeTest.cpp)
target_link_libraries(FindEdgeTest PCMLib)