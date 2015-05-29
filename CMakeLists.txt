PROJECT (graspSimulation)
# Build system:

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
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
INCLUDE_DIRECTORIES(include)

set(qhullLibs ${qhullDir}/build/libqhullcpp.a ${qhullDir}/build/libqhullstatic_p.a)             
                
#examples:
#ADD_EXECUTABLE(qhullExample example/qhullExample.cpp)
#TARGET_LINK_LIBRARIES(qhullExample ${qhulllibs})

ADD_EXECUTABLE(libExample example/libExample.cpp)
TARGET_LINK_LIBRARIES(libExample ${qhullLibs})
set_target_properties(libExample PROPERTIES
    COMPILE_DEFINITIONS "qh_QHpointer")

ADD_EXECUTABLE(testWS example/testWrenchSpaceComp.cpp src/grasp.cpp src/wrench_cone.cpp src/wrenchspace.cpp)
TARGET_LINK_LIBRARIES(testWS ${qhullLibs})
set_target_properties(testWS PROPERTIES
    COMPILE_DEFINITIONS "qh_QHpointer")
