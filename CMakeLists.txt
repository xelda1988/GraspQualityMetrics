PROJECT (graspSimulation)
# Build system:

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#ADD_DEFINITIONS(-Wall)
#ADD_DEFINITIONS(-Wno-c++11-extensions -Wno-unused-variable -Wno-unneeded-internal-declaration)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

SET(qhullDir /home/alexander/external_libs/qhull-2012.1)

INCLUDE_DIRECTORIES(${qhullDir}/src/libqhullcpp)
INCLUDE_DIRECTORIES(${qhullDir}/src)
LINK_DIRECTORIES(${qhullDir}/build)	

INCLUDE_DIRECTORIES(${qhullDir}/src/libqhullcpp)

set(qhullLibs ${qhullDir}/build/libqhullcpp.a ${qhullDir}/build/libqhullstatic_p.a)             
                
#examples:
#ADD_EXECUTABLE(qhullExample example/qhullExample.cpp)
#TARGET_LINK_LIBRARIES(qhullExample ${qhulllibs})

ADD_EXECUTABLE(libExample example/libExample.cpp)
TARGET_LINK_LIBRARIES(libExample ${qhullLibs})
set_target_properties(libExample PROPERTIES
    COMPILE_DEFINITIONS "qh_QHpointer")
