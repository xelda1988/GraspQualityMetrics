#ifndef TWSGRASPQUALITY_H
#define TWSGRASPQUALITY_H

#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <list>
#include <vector>
#include <tr1/memory>
#include <Eigen/Core>
#include <iostream>

#include "wrenchspace.h"
#include "taskwrenchellispoid.h"

class twsGraspQuality
{
    std::vector<Vector6d> wrenches_in_;
    Transform hand_to_ellipse_;
    Eigen::Vector3d semiAxesTorque_;
    Eigen::Vector3d semiAxesForces_;
    Eigen::Vector3d forceOffset_;

    std::vector<Vector6d> wrenches_out_;
    std::vector<double> wrenches_vector_out_;
    SharedDoublePtr transformedWrenches;
    uint nrWrenches_;
    uint dim_;
public:
    twsGraspQuality();
    void setParams(const std::vector<Vector6d> & wrenches_in,
                   const Transform & hand_to_ellipse,
                   const Eigen::Vector3d  & semiAxesTorque,
                   const Eigen::Vector3d & semiAxesForces,
                   const Eigen::Vector3d & forceOffset){
        wrenches_in_=wrenches_in;
        hand_to_ellipse_=hand_to_ellipse; //the translation is not metric but a torque measure
        semiAxesTorque_=semiAxesTorque;
        semiAxesForces_=semiAxesForces;
        nrWrenches_=wrenches_in.size();
        forceOffset_=forceOffset;
        dim_=6;
    }
    void computeTransform();
    void getOutput(SharedDoublePtr & transformedWrenchs, uint & nrWrenches){

        transformedWrenchs=transformedWrenches;
        nrWrenches=nrWrenches_;

    }
    std::vector<double> getVector(){

        return wrenches_vector_out_;

    }

};

#endif // TWSGRASPQUALITY_H
