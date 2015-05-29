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
    void setFingerId(uint fingerId){
        fingerId_=fingerId;
    }

    void setTorqueArm(double torqueArm){

        torqueArm_=torqueArm;

    }


    void computePrimitiveWrenches();

    std::vector<Vector6d> getPrimitiveWrenches(){

        return primitiveWrenches_;
    }

    //void getPrimitiveWrenches();

    //Constr
    WrenchCone(){}

};


class WrenchConesAll{

private:

    std::vector<WrenchCone> allCones_;
    std::vector<double> allWrenches_; //easily mappable to double array or shared_ptr
    std::vector<double> allForces_;//For visualization of pure force hull
    std::vector<double> allTorques_; //For visualization of pure torque hull
    uint nrWrenches_;
    Grasp grasp_;

public:

    void setGrasp(const Grasp & grasp){
        grasp_=grasp;
    }

    void computeAllWrenchCones(uint nrFacets);

    std::vector<double> getAllWrenches()
    {
        return allWrenches_;
    }

    std::vector<double> getAllForces()
    {
        return allForces_;
    }

    std::vector<double> getAllTorques()
    {
        return allTorques_;
    }

    uint getNrWrenches()
    {
        return nrWrenches_;

    }

    WrenchConesAll(const Grasp & grasp){
        grasp_=grasp;
    }

};


#endif // WRENCH_CONE_H
