cmake_minimum_required(VERSION 2.8)
# 增加一个可执行的二进制
# ADD_EXECUTABLE( main main.cpp )

# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED )

# list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04
# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )


# set(PCL_INCLUDE_DIRS /usr/include/pcl-1.8)
# 添加头文件和库文件
file(GLOB_RECURSE SRC ../src/*.cpp)

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES(../include )

LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

ADD_EXECUTABLE(test_Img2PC test_Img2PC.cpp ${SRC})
TARGET_LINK_LIBRARIES( test_Img2PC ${OpenCV_LIBS}
    ${PCL_LIBRARIES} )

ADD_EXECUTABLE(test_ComputeFeatAndDesp test_ComputeFeatAndDesp.cpp ${SRC})
TARGET_LINK_LIBRARIES( test_ComputeFeatAndDesp ${OpenCV_LIBS}
    ${PCL_LIBRARIES} )

ADD_EXECUTABLE(test_MatchAndRansac test_MatchAndRansac.cpp ${SRC})
TARGET_LINK_LIBRARIES( test_MatchAndRansac ${OpenCV_LIBS}
    ${PCL_LIBRARIES} )

ADD_EXECUTABLE(test_RvecTvec2Mat test_RvecTvec2Mat.cpp ${SRC})
TARGET_LINK_LIBRARIES( test_RvecTvec2Mat ${OpenCV_LIBS}
    ${PCL_LIBRARIES} )

ADD_EXECUTABLE(test_UpdatePointCloud test_UpdatePointCloud.cpp ${SRC})
TARGET_LINK_LIBRARIES( test_UpdatePointCloud ${OpenCV_LIBS}
    ${PCL_LIBRARIES} )

