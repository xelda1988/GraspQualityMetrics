# Grasp-Simulation-Vrep

Software capabilities:

Perform a grasp in simulation and read out contact normals and mesh for 
template generation - Template includes Pointcloud 

Dependencies:

    ROS
    qhull

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
