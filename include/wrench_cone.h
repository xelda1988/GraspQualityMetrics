#ifndef WRENCH_CONE_H
#define WRENCH_CONE_H

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include "grasp.h"

//wrench cone computes the primitve wrenches to constructe the GWS from a single contact

class WrenchCone {

private:
    double friction_;
    uint nrFacets_; //e.g 8 or 6 or so;
    uint fingerId_;
    Vector6d contact_;

    double torqueArm_; //should be set from grasp itself!

    std::vector<Vector6d> primitiveWrenches_;
    //void computeTorqueArm();

    void rescaleTorques(double scale);
    void rescaleTorques();//by torque arm
    void scaleWrenches(double scale);

public:

    void setFriction(double friction) {
        friction_=friction;
    }
    void setNrFactes(uint nrFacets){
        nrFacets_=nrFacets;
    }
    void setContact(Vector6d & contact){
        contact_=contact;
    }
    setFingerId(uint fingerId){
        fingerId_=fingerId;
    }


    void computePrimitiveWrenches();

    //void getPrimitiveWrenches();

    //Constr
    WrenchCone();

};


#endif // WRENCH_CONE_H
