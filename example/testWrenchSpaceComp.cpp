//Testing the grasp, conews and gws classes!

#include "grasp.h"
#include "wrench_cone.h"
#include "wrenchspace.h"
#include "taskwrenchellispoid.h"
#include "twsgraspquality.h"

#include <chrono>
#include <time.h>

using namespace std;

int main(int argc, char** argv)
{

    //Settings:

    //From regression algorithm
    double mass(1); //1kg
    double sigmaMass(0.2); //uncertainty: 0.2kg
    Eigen::Vector3d com(0,0,0); //center of mass
    Eigen::Vector3d sigmaCom(0.05,0.05,0.05); //uncertainty center of mass in meters
    Eigen::Matrix3d camera_to_pca_rot=Eigen::Matrix3d::Identity();

    //Gravity Direction (s) TODO: Later add set of Rotations of body
    Eigen::Vector3d gravity_normal(0,0,-1); //in the frame of the camera, but here cam to pca is identity for testing

    //settings hand and environment;

    double maxForceHand=50; //maximum contact force in each contact is 50 Newtons
    double noiseForce=1; //1 Newton noise in force
    double noiseTorque=noiseForce*0.05; // Noise in Torques, actually could be computed from the maximum bounding sphere of sigmaCom

    //Transforms:

    Transform camera_to_world, world_to_hand;

    camera_to_world.translation << 0,0,0;
    camera_to_world.rotation = Eigen::Matrix3d::Identity();

    //gripper frame is 2 cm below object with 3 fingers pointing upwards to
    //check if non force closure grasps works together with gravity, translation along z should not play any role for the grasp quality measure!

    world_to_hand.translation << 0,0,-0.02;
    world_to_hand.rotation = Eigen::Matrix3d::Identity();

    //defining all contacts for the grasp!

    Grasp grasp;

    Vector6d contact1, contact2, contact3, contact4;
    //3 Finger Grasp, hard finger contact with friction, in hand frame pointing upwards at positions building a triangle, it might make the hull degenerate?

    contact1 << 0, 0.03                ,0.01,   0,0,1;
    contact2 << -0.03*0.5,-0.03*0.87   ,0.01,   0,0,1;
    contact3 <<  0.03*0.5,-0.03*0.87   ,0.01,   0,0,1;

    contact4 << -1,0 ,0.01,  1,0,0;

    grasp.addContact(contact1);
    grasp.addContact(contact2);
    grasp.addContact(contact3);
    //grasp.addContact(contact4);

    grasp.setFriction(0.3);

    //starting clock, computing Task Wrench Ellipsoids

    clock_t begin, end;
    double time_spent;
    begin = clock();

    cout << "Starting Timer ..." << endl;

    taskwrenchEllipsoid tws_ell;

    tws_ell.setInput(mass,sigmaMass, com, sigmaCom, camera_to_pca_rot);
    tws_ell.setNoise(noiseForce,noiseTorque);
    tws_ell.sampleComEllipsoid();
    tws_ell.computeTorqueEllipsoid();

    end = clock();
    time_spent = (double)(end - begin) / CLOCKS_PER_SEC;

    cout<<"Computation Time for Ellipsoid: "<< time_spent << endl;

    tws_ell.writeSampledPts("/home/alexander/tmp/2dGraspWrenchSpace/sampledPts.txt");
    tws_ell.writeTaskEllipse("/home/alexander/tmp/2dGraspWrenchSpace/taskEll.txt");



    /*

    cout << "Testing Grasp:" << endl;



    double torqueArm=grasp.getMaxTorqueArm();

    cout << "Max Torque Arm: " << torqueArm << endl;


    //compute CONEWS for contact1:

    WrenchCone wrenchcone1;

    wrenchcone1.setFriction(0.5);
    wrenchcone1.setNrFactes(16);
    wrenchcone1.setContact(contact1);
    wrenchcone1.setTorqueArm(torqueArm);


    wrenchcone1.computePrimitiveWrenches();

    std::vector<Vector6d> primWrenches = wrenchcone1.getPrimitiveWrenches();

    cout << "Computed Primitive Wrenches:" << endl;

    for(int i = 0; i < primWrenches.size(); i++)
    {
        cout << primWrenches[i].transpose() << endl;
    }


    //computeAll

    WrenchConesAll wrenchCones(grasp);

    wrenchCones.computeAllWrenchCones(8);
    vector<double> allWrenches=wrenchCones.getAllWrenches();

    cout << "Nr wrenches " << wrenchCones.getNrWrenches() << endl;

    for (int i=0; i < allWrenches.size(); i++){

        cout << allWrenches[i] <<  " " << endl;

    }



    double *wrenchArray = allWrenches.data();
    SharedDoublePtr wrenchPtr(wrenchArray);

    DiscreteWrenchSpace discreteWrenchspace(6,wrenchPtr,wrenchCones.getNrWrenches());

    discreteWrenchspace.computeConvexHull();

    vector<double> taskWrench;

    taskWrench.push_back(0);
    taskWrench.push_back(0);
    taskWrench.push_back(0);
    taskWrench.push_back(0);
    taskWrench.push_back(0);
    taskWrench.push_back(0);

    SharedDoublePtr taskW(taskWrench.data());
    cout << "minDist: = " << discreteWrenchspace.computeDistToHull(taskW) << std::endl;




    //compute second convex hull from 3d points of forces:
     vector<double> allForces=wrenchCones.getAllForces();

     cout << "Nr forces " << allForces.size()/3 << endl;

     for (int i=0; i < allForces.size(); i++){

         cout << allForces[i] <<  " " << endl;

     }


     double *forceArray = allForces.data();
     SharedDoublePtr forcePtr(forceArray);

     DiscreteWrenchSpace forceWrenchspace(3,forcePtr,wrenchCones.getNrWrenches());

     forceWrenchspace.computeConvexHull();


    cout << discreteWrenchspace << endl;

    cout << forceWrenchspace << endl;
    vector<double> taskForce;

    taskForce.push_back(0);
    taskForce.push_back(0);
    taskForce.push_back(0);
    SharedDoublePtr taskF(taskForce.data());
    cout << "minDist: = " << forceWrenchspace.computeDistToHull(taskF) << std::endl;

    forceWrenchspace.writeToOffFile("/home/alexander/tmp/offtest/forcespace.off");

    ///visualizing pure torquespace:

    vector<double> allTorques=wrenchCones.getAllTorques();

    SharedDoublePtr torquePtr(allTorques.data());

    DiscreteWrenchSpace torqueWrenchspace(3,torquePtr,wrenchCones.getNrWrenches());

    torqueWrenchspace.computeConvexHull();
    cout << torqueWrenchspace << endl;
    torqueWrenchspace.writeToOffFile("/home/alexander/tmp/offtest/torquespace.off");

    */

    return 0;


}
