cmake_minimum_required(VERSION 2.8)
# 增加一个可执行的二进制
# ADD_EXECUTABLE( main main.cpp )

#add c++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED )

# add dependence of g2o
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O )
set(G2O_LIBRARIES g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension)
# CSparse_g2o
FIND_PACKAGE( CSparse )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )



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

ADD_EXECUTABLE(visualOdometry visualOdometry.cpp ${SRC})
TARGET_LINK_LIBRARIES( visualOdometry ${OpenCV_LIBS}
    ${PCL_LIBRARIES} )

ADD_EXECUTABLE(g2o g2o.cpp ${SRC})
TARGET_LINK_LIBRARIES( g2o ${OpenCV_LIBS}
    ${PCL_LIBRARIES} ${G2O_LIBRARIES} ${CSPARSE_LIBRARY})

ADD_EXECUTABLE(g2o_u g2o_update.cpp ${SRC})
TARGET_LINK_LIBRARIES(g2o_u ${OpenCV_LIBS}
    ${PCL_LIBRARIES} ${G2O_LIBRARIES} ${CSPARSE_LIBRARY})
