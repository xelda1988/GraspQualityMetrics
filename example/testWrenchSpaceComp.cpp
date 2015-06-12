//Testing the grasp, conews and gws classes!

#include "grasp.h"
#include "wrench_cone.h"
#include "wrenchspace.h"
#include "taskwrenchellispoid.h"
#include "twsgraspquality.h"

#include <chrono>
#include <time.h>
#include <fstream>

using namespace std;

void writeWrenchesToFile(const SharedDoublePtr & data, uint nrWrenches, const string & file){

    uint dim=6;
    std::ofstream of(file);
    for(int i=0; i < nrWrenches*dim; i++)
    {
        //cout << data.get()[i] << " ";
        //if((i+1)%6==0){cout << endl;}
        if(of.is_open()){
            of << data.get()[i] << " ";
            if((i+1)%6==0){of << endl;}
        }
    }
}

int main(int argc, char** argv)
{

    //Settings:

    //From regression algorithm
    double mass(1); //1kg
    double sigmaMass(0.2); //uncertainty: 0.2kg
    Eigen::Vector3d com(0.5,0,0); //center of mass in which frame??
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

    //contact4 << 0,0,0.03,  0,0,-1; //add grasp from above!

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
    tws_ell.setCameraCalib(camera_to_world);
    tws_ell.setGripperPose(world_to_hand);
    tws_ell.setGravityNormal(gravity_normal);

    tws_ell.sampleComEllipsoid();
    tws_ell.computeTorqueEllipsoid();

    end = clock();
    time_spent = (double)(end - begin) / CLOCKS_PER_SEC;

    cout<<"Computation Time for Ellipsoid: "<< time_spent << endl;

    tws_ell.writeSampledPts("/home/alexander/tmp/2dGraspWrenchSpace/sampledPts.txt");
     tws_ell.writeTaskEllipse("/home/alexander/tmp/2dGraspWrenchSpace/taskEll.txt");



    Transform hand_to_ellipse;
    Eigen::Vector3d semiAxesTorque;
    Eigen::Vector3d semiAxesForces;
    Eigen::Vector3d forceOffset; //in pca frame!
    tws_ell.getLinearTransform(hand_to_ellipse, semiAxesTorque, semiAxesForces,forceOffset);

    //writing the semiaxes + rotation  also to a file!



    //After get Linear Transform!!!!!!!!
    tws_ell.writeTransformedPts("/home/alexander/tmp/2dGraspWrenchSpace/sampledPtsTrans.txt");//torque points transformed to gripper frame
    tws_ell.writeForceTorqueEllipse("/home/alexander/tmp/2dGraspWrenchSpace/ellpsoidTrans.txt");


    cout << "Transformation for the wrenches:" << endl;
    cout << "Hand to Ellipse:" << endl;
    cout << hand_to_ellipse.hom << endl;
    cout << "SemiAxis Torque:" << endl;
    cout << semiAxesTorque.transpose() << endl;
    cout << "SemiAxis Force:" << endl;
    cout << semiAxesForces.transpose() << endl;

    cout << "Computing primitive wrenches from cones contacts " << endl;

    WrenchConesAll wrenchCones(grasp);
    wrenchCones.setMaxForce(maxForceHand);
    wrenchCones.computeAllWrenchCones(8);
    wrenchCones.addZeroToWrenches(); //adding zero force grasp

    vector<double> allWrenches=wrenchCones.getAllWrenches();
    cout << "Nr wrenches " << wrenchCones.getNrWrenches() << endl;
    //for (int i=0; i < allWrenches.size(); i++){
    //    cout << allWrenches[i] <<  " " << endl;
    //}
    double *wrenchArray = allWrenches.data();
    SharedDoublePtr wrenchPtr(wrenchArray);

    cout << "Transforming wrenches with transformations!" << endl;

    std::vector<Vector6d> wrenchVec=wrenchCones.get6dEigen();

    twsGraspQuality gwsTransform;
    gwsTransform.setParams(wrenchVec,
                           hand_to_ellipse,
                           semiAxesTorque,
                           semiAxesForces,
                           forceOffset);

    gwsTransform.computeTransform();

    SharedDoublePtr wrenchPtrTrans;
    uint nrWrenches;
    gwsTransform.getOutput(wrenchPtrTrans,nrWrenches);

    vector<double> taskWrench;

    taskWrench.push_back(0);
    taskWrench.push_back(0);
    taskWrench.push_back(0);
    taskWrench.push_back(0);
    taskWrench.push_back(0);
    taskWrench.push_back(0);

    cout << "Computing convex Hull on original wrenches:" << endl;
    DiscreteWrenchSpace discreteWrenchspace(6,wrenchPtr,wrenchCones.getNrWrenches());
    discreteWrenchspace.computeConvexHull();
    cout << discreteWrenchspace << endl;

    SharedDoublePtr taskW(taskWrench.data());
    //cout << "minDist: = " << discreteWrenchspace.computeDistToHull(taskW) << std::endl;

    cout << "Computing convex Hull on transformed wrenches:" << endl;
    DiscreteWrenchSpace discreteWrenchspaceTrans(6,wrenchPtrTrans,nrWrenches);
    discreteWrenchspaceTrans.computeConvexHull();
    cout << discreteWrenchspaceTrans << endl;


    // cout << "minDist: = " << discreteWrenchspaceTrans.computeDistToHull(taskW) << std::endl;

    cout << "Writing wrenches to file for visualization!" << endl;

    writeWrenchesToFile(wrenchPtr,wrenchCones.getNrWrenches(),"/home/alexander/tmp/2dGraspWrenchSpace/wrenches_orig.txt");
    writeWrenchesToFile(wrenchPtrTrans,nrWrenches,"/home/alexander/tmp/2dGraspWrenchSpace/wrenches_trans.txt");

    //compute second convex hull from 3d points of forces:
    vector<double> allForces=wrenchCones.getAllForces();
    double *forceArray = allForces.data();
    SharedDoublePtr forcePtr(forceArray);
    DiscreteWrenchSpace forceWrenchspace(3,forcePtr,wrenchCones.getNrWrenches());
    forceWrenchspace.computeConvexHull();
    cout << forceWrenchspace << endl;
    forceWrenchspace.writeToOffFile("/home/alexander/tmp/offtest/forcespace.off"); //To compare to the Mathematica visualization!



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
