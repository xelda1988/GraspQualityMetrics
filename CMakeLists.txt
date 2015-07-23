PROJECT (graspSimulation)
# Build system:

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -fPIC" )
#ADD_DEFINITIONS(-Wall)
#ADD_DEFINITIONS(-Wno-c++11-extensions -Wno-unused-variable -Wno-unneeded-internal-declaration)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

SET(qhullDir /home/c7031075/external_libraries/qhull-2012.1)

INCLUDE_DIRECTORIES(${qhullDir}/src/libqhullcpp)
INCLUDE_DIRECTORIES(${qhullDir}/src)
LINK_DIRECTORIES(${qhullDir}/build)	

INCLUDE_DIRECTORIES(${qhullDir}/src/libqhullcpp)
INCLUDE_DIRECTORIES(include)

#SET(qhullLibs ${qhullDir}/build/libqhullcpp.so ${qhullDir}/build/libqhull_p.so)
SET(qhullLibs qhullcpp qhull_p)
                
#examples:
#ADD_EXECUTABLE(qhullExample example/qhullExample.cpp)
#TARGET_LINK_LIBRARIES(qhullExample ${qhulllibs})

ADD_EXECUTABLE(libExample example/libExample.cpp)
TARGET_LINK_LIBRARIES(libExample ${qhullLibs})
SET_TARGET_PROPERTIES(libExample PROPERTIES
    COMPILE_DEFINITIONS "qh_QHpointer")

SET(cppsources
src/grasp.cpp
src/wrench_cone.cpp
src/wrenchspace.cpp
src/taskwrenchellipsoid.cpp
src/twsgraspquality.cpp
)

ADD_EXECUTABLE(testWS example/testWrenchSpaceComp.cpp ${cppsources})

TARGET_LINK_LIBRARIES(testWS ${qhullLibs})
SET_TARGET_PROPERTIES(testWS PROPERTIES
    COMPILE_DEFINITIONS "qh_QHpointer")

ADD_LIBRARY(GraspQualityMetrics SHARED ${cppsources})
TARGET_LINK_LIBRARIES(GraspQualityMetrics ${qhullLibs})
SET_TARGET_PROPERTIES(GraspQualityMetrics PROPERTIES
    COMPILE_DEFINITIONS "qh_QHpointer")

ADD_EXECUTABLE(testQM example/testQmLibrary.cpp)
TARGET_LINK_LIBRARIES(testQM GraspQualityMetrics)
#SET_TARGET_PROPERTIES(testQM PROPERTIES
#    COMPILE_DEFINITIONS "qh_QHpointer")

#installation of header files into /usr/local/include
FILE(GLOB PROJECT_HEADERS include/*.h)
INSTALL(FILES ${PROJECT_HEADERS} DESTINATION include/GraspQualityMetrics)

INSTALL(TARGETS GraspQualityMetrics
     RUNTIME DESTINATION bin
     LIBRARY DESTINATION lib
     ARCHIVE DESTINATION lib
     COMPONENT libraries)
     
     

SET_TARGET_PROPERTIES (
    GraspQualityMetrics
    PROPERTIES LINKER_LANGUAGE CXX
)
