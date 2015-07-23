# GraspQualityMetrics

Software capabilities:

Perform a grasp in simulation and read out contact points, contact normals and mesh/pointcloud for template generation.
Contact points can be clustered and used for Quality Measure Computation.

Dependencies:

    qhull
    vrep
    ROS
    
Installation of qhull:

    git clone https://github.com/xelda1988/qhull-2012.1.git
    mkdir build && cd build
    cmake ..
    make

Installation of the Project GraspQualityMetrics:

* In CMakeLists modify Line 14: SET(qhullDir "<yourQhullDirectory>") 

Needs:

    Scene containing robotic hand and a floor (if gravity on)

Functions:

Library GraspQualityMetrics

    computeGraspWrenchSpace
    visualizeGraspWrenchSpace
    computeTaskWrenchSpace
    visualizeTaskWrenchSpace
    computeQualityMeasure

Module ContactPointGeneration

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
