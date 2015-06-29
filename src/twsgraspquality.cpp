#include "twsgraspquality.h"
#define DEBUGQM 1

namespace GraspQm{


void twsGraspQuality::computeTransform(){

    wrenches_out_.clear();
    wrenches_vector_out_.clear();


    Eigen::Matrix3d fScale = semiAxesForces_.asDiagonal().inverse();
    Eigen::Matrix3d tScale = semiAxesTorque_.asDiagonal().inverse();


 #if DEBUGQM
    std::cout << "fScale:" << std::endl;
    std::cout << fScale << std::endl;
    std::cout << tScale << std::endl;

#endif

    for(int i=0; i < nrWrenches_; i++){


        Vector6d wrench_out;
        Eigen::Vector3d force = wrenches_in_.at(i).head(3);
        Eigen::Vector3d torque = wrenches_in_.at(i).tail(3);

        Eigen::Vector3d torque_in_ellipse, force_t, torque_t;


        torque_in_ellipse=hand_to_ellipse_.translation;

        force_t = fScale*hand_to_ellipse_.rotation*(force-forceOffset_);
        torque_t = tScale*hand_to_ellipse_.rotation*(torque-torque_in_ellipse);


 #if DEBUGQM
        std::cout << "TRANSFORMED FORCE AND TORQUE" << std::endl;
        std::cout << force_t << std::endl;
        std::cout << torque_t << std::endl;

#endif
        wrench_out << force_t,torque_t;
        wrenches_out_.push_back(wrench_out);

        toPtr.push_back(force_t(0));
        toPtr.push_back(force_t(1));
        toPtr.push_back(force_t(2));

        toPtr.push_back(torque_t(0));
        toPtr.push_back(torque_t(1));
        toPtr.push_back(torque_t(2));

        wrenches_vector_out_.push_back(force_t(0));
        wrenches_vector_out_.push_back(force_t(1));
        wrenches_vector_out_.push_back(force_t(2));

        wrenches_vector_out_.push_back(torque_t(0));
        wrenches_vector_out_.push_back(torque_t(1));
        wrenches_vector_out_.push_back(torque_t(2));

    }

    //filling ptr
    //double *wrenches_trans=toPtr.data();

    //double *wrenchArray =toPtr.data();
   // SharedDoublePtr tmpSharedPtr(wrenchArray); //like allocation?
    //transformedWrenches=tmpSharedPtr;
   // transformedWrenches = wrenchArray;
    //std::cout << "[finished comp]" << std::endl;

}

twsGraspQuality::twsGraspQuality()
{
}

}//ns
