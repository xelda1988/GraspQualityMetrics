#include "include/wrench_cone.h"


//from torquearm

void WrenchCone::rescaleTorques(double scale){

    for(int i=0; i < primitiveWrenches_.size();i ++){

        primitiveWrenches_.at(i)(3)*scale;
        primitiveWrenches_.at(i)(4)*scale;
        primitiveWrenches_.at(i)(5)*scale;

    }
}

void WrenchCone::rescaleTorques(){
    rescaleTorques(torqueArm_);
}

void WrenchCone::scaleWrenches(double scale){

    for(int i=0; i < primitiveWrenches_.size();i ++){
        primitiveWrenches_.at(i)*scale;

    }
}

void WrenchCone::computePrimitiveWrenches(){

    //normal vector + tangential vector in current angle!



    Eigen::Vector3d fx, fy, fn, up, ps, force, torque;

    ps(0)=contact_(0);
    ps(1)=contact_(1);
    ps(2)=contact_(2);

    fn(0)=contact_(3);
    fn(1)=contact_(4);
    fn(2)=contact_(5);

    //compute a vector which is normal to fn
    up << 1,0,0;
    if(fn.cross(up).norm() < 0.0001) up << 0,1,0;

    fx = fn.cross(up).normalized();
    fy = fx.cross(fn).normalized();

    std::vector<Vector6d> prWrenches;
    Vector6d currWrench;

    for(int i =0; i < nrFacets_; i++){

        //current force for ith facet

        force = fn + fx*friction_*sin(2*M_PI*i/nrFacets_) + fy*friction_*sin(2*M_PI*i/nrFacets_);
        torque = ps.cross(force);

        currWrench << force,torque;
        prWrenches.push_back(currWrench);

    }
    primitiveWrenches_=prWrenches;

}
