# Grasp-Simulation-Vrep

Software capabilities:

Perform a grasp in simulation and read out contact normals and mesh for 
template generation - Template includes Pointcloud 

Dependencies:

    ROS
    qhull
    
Installation of qhull:

    wget http://www.qhull.org/download/qhull-2012.1-src.tgz
* Untar 
* In libqhull/user.h set qh_QHpointer 1
* Install with Cmake installation script

Installation of Project Grasp-Simulation-Vrep:
* In CMakeLists modify Line 14: SET(qhullDir "yourQhullBaseDirectory") 

Needs:

    Scene containing robotic hand and a floor (if gravity on)

Functions:

Module Simulation Control

    setGripperPose
    setGripperJoints
    setGripperTorques
    loadObjectFromFile
    setObjectPose
    setObjectMass
    setObjectFriction
    detectGraspEquilibrium
    getObjMesh(in new Gripper frame)
    getContactPoints
    getContactNormals
    getContactForces
    getObjCenter
    getTactileMatrices
    getGripperJoints

Module Grasp Quality Measures

    computeGraspWrenchSpace
    visualizeGraspWrenchSpace
    computeTaskWrenchSpace
    visualizeTaskWrenchSpace
    computeQualityMeasure
