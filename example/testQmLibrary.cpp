//Testing the grasp, conews and gws classes!

#include <chrono>
#include <time.h>
#include <fstream>
#include "computeGraspQuality.h"


#define DEBUGQM 0

using namespace std;

int main(int argc, char** argv)
{

    //Settings:

    //From regression algorithm
    double mass(3); //1kg
    double sigmaMass(0.3); //uncertainty: 0.2kg
    Eigen::Vector3d com(0.0,0,0); //center of mass in which frame??
    Eigen::Vector3d sigmaCom(0.0325,0.0125,0.0125); //uncertainty center of mass in meters
    Eigen::Matrix3d camera_to_pca_rot=Eigen::Matrix3d::Identity();
    Eigen::Vector3d gravity_normal(0,0,-1); //in the frame of the camera, but here cam to pca is identity for testing

    double maxForceHand=50; //maximum contact force in each contact is 50 Newtons
    double noiseForce=0.1; //1 Newton noise in force
    double noiseTorque=noiseForce*0.005; // Noise in Torques, actually could be computed from the maximum bounding sphere of sigmaCom

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
    contact1 << 0, 0.06                ,0.01,   0,0,1;
    contact2 << -0.06*0.5,-0.06*0.87   ,0.01,   0,0,1;
    contact3 <<  0.06*0.5,-0.06*0.87   ,0.01,   0,0,1;
    //contact4 << 0,0,0.03,  0,0,-1; //add grasp from above!
    grasp.addContact(contact1);
    grasp.addContact(contact2);
    grasp.addContact(contact3);
    //grasp.addContact(contact4);
    grasp.setFriction(0.1);

    //starting clock, computing Task Wrench Ellipsoids

    clock_t begin, end;
    double time_spent;
    begin = clock();

    cout << "Starting Timer ..." << endl;
double graspQuality;

//ComputeGraspQuality compGraspQuality;

    Transform world_to_hand2;
    world_to_hand2.hom=Eigen::Matrix4d::Identity();
    world_to_hand2.hom(0,3)=0.0;
    world_to_hand2.hom(1,3)=0.0;
    world_to_hand2.hom(2,3)=0.0;
    world_to_hand2.setFromHomog();

    ComputeGraspQuality compGraspQuality;
    compGraspQuality.setInput(mass,
                              sigmaMass,
                              com,
                              sigmaCom,
                              camera_to_pca_rot,
                              gravity_normal,
                              grasp,
                              camera_to_world);

    compGraspQuality.setPose(world_to_hand2);
    compGraspQuality.changeStdSettings(maxForceHand,noiseForce,noiseTorque,8);

    compGraspQuality.initComputation();



    for(int i=0; i < 1000; i++){
        world_to_hand2.hom(0,3)=((double) i)/1.0;
        world_to_hand2.hom(1,3)=((double) i)/1.0;
        world_to_hand2.hom(2,3)=((double) i)/1.0;

        world_to_hand2.setFromHomog();
        compGraspQuality.setPose(world_to_hand2);
        compGraspQuality.recomputeQM();
        graspQuality=compGraspQuality.getQM();
    //cout<<graspQuality<< endl;
    }
    //cout<<"[DEBUG2]"<< endl;

    end = clock();
    time_spent = (double)(end - begin) /1000.0 / CLOCKS_PER_SEC;

    cout<<"Computation Time for Quality Measure (us): "<< time_spent*1e6 << endl;
    cout<<"Value: "<< graspQuality << endl;

    return 0;

}
