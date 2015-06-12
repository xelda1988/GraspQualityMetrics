#include "wrench_cone.h"


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
        //std::cout << "[DEBUG:] maxForce!" << maxForce_ << std::endl;
        force = maxForce_*(fn + fx*friction_*cos(2*M_PI*i/nrFacets_) + fy*friction_*sin(2*M_PI*i/nrFacets_));//scale them with max force!
        torque = ps.cross(force);

        currWrench << force,torque;
        prWrenches.push_back(currWrench);

    }

    primitiveWrenches_=prWrenches;

}

void WrenchConesAll::computeAllWrenchCones(uint nrFacets){


    double friction=grasp_.getFriction();
    double maxTorqueArm=grasp_.getMaxTorqueArm();

    if(allWrenches_.size()!=0) {
        std::cout << "Clearing already computed wrenchcones" << std::endl;
        allWrenches_.clear();
        allCones_.clear();
        allForces_.clear();
        allTorques_.clear();
        vec6d_.clear();
    }

    nrWrenches_=0;

    for(int i=0; i < grasp_.getNrContacts(); i++){

        Vector6d contact=grasp_.getContacts().at(i);
        WrenchCone wrenchCone;

        wrenchCone.setFriction(friction);
        wrenchCone.setNrFactes(nrFacets);
        wrenchCone.setContact(contact);
        wrenchCone.setTorqueArm(maxTorqueArm);
        //std::cout << "[DEBUG:] maxForce before!" << maxForce_ << std::endl;
        wrenchCone.setMaxForce(maxForce_);


        wrenchCone.computePrimitiveWrenches();
        allCones_.push_back(wrenchCone);

        std::vector<Vector6d> prWrenches= wrenchCone.getPrimitiveWrenches();

        nrWrenches_ += prWrenches.size();

        for(int j=0; j < prWrenches.size(); j++){
            Vector6d currWrench=prWrenches.at(j);
            for(int k=0; k < 6; k++){
                allWrenches_.push_back(currWrench(k));
            }
            vec6d_.push_back(currWrench);

            for(int k=0; k < 3; k++){
                allForces_.push_back(currWrench(k));
            }

            for(int k=3; k < 6; k++){
                allTorques_.push_back(currWrench(k));
            }


        }
    }
}
