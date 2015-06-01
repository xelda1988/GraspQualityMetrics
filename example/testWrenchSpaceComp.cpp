//Testing the grasp, conews and gws classes!

#include "grasp.h"
#include "wrench_cone.h"
#include "wrenchspace.h"

using namespace std;

int main(int argc, char** argv)
{

    cout << "Testing Grasp:" << endl;

    Grasp grasp;

    Vector6d contact1, contact2, contact3, contact4;

    //4 Finger Grasp, hard finger contact with friction!
    contact1 << 0,1,0, 0,-1,0;
    contact2 << 1,0,0,-1,0,0;
    contact3 << 0,-1,0,0,1,0;
    contact4 << -1,0,0,1,0,0;

    grasp.addContact(contact1);
    grasp.addContact(contact2);
    grasp.addContact(contact3);
    grasp.addContact(contact4);

    grasp.setFriction(0.5);

    double torqueArm=grasp.getMaxTorqueArm();

    cout << "Max Torque Arm: " << torqueArm << endl;


    //compute CONEWS for contact1:

    WrenchCone wrenchcone1;

    wrenchcone1.setFriction(0.5);
    wrenchcone1.setNrFactes(8);
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

    wrenchCones.computeAllWrenchCones(4);
    vector<double> allWrenches=wrenchCones.getAllWrenches();

    cout << "Nr wrenches " << wrenchCones.getNrWrenches() << endl;

    for (int i=0; i < allWrenches.size(); i++){

        cout << allWrenches[i] <<  " " << endl;

    }



    double *wrenchArray = allWrenches.data();
    SharedDoublePtr wrenchPtr(wrenchArray);

    DiscreteWrenchSpace discreteWrenchspace(6,wrenchPtr,wrenchCones.getNrWrenches());

    discreteWrenchspace.computeConvexHull();

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


    return 0;
}
