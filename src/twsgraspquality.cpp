#include "twsgraspquality.h"

void twsGraspQuality::computeTransform(){

    wrenches_out_.clear();
    wrenches_vector_out_.clear();
    std::vector<double> toPtr;

    Eigen::Matrix3d fScale = semiAxesForces_.asDiagonal().inverse();
    Eigen::Matrix3d tScale = semiAxesTorque_.asDiagonal().inverse();



    std::cout << "fScale:" << std::endl;
    std::cout << fScale << std::endl;
    std::cout << tScale << std::endl;

    for(int i=0; i < nrWrenches_; i++){


        Vector6d wrench_out;
        Eigen::Vector3d force = wrenches_in_.at(i).head(3);
        Eigen::Vector3d torque = wrenches_in_.at(i).tail(3);

        Eigen::Vector3d torque_in_ellipse, force_t, torque_t;


        torque_in_ellipse=hand_to_ellipse_.translation;

        force_t = fScale*hand_to_ellipse_.rotation*(force-forceOffset_);
        torque_t = tScale*hand_to_ellipse_.rotation*(torque-torque_in_ellipse);



        std::cout << "TRANSFORMED FORCE AND TORQUE" << std::endl;
        std::cout << force_t << std::endl;
        std::cout << torque_t << std::endl;

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

    double *wrenchArray =toPtr.data();
    SharedDoublePtr tmpSharedPtr(wrenchArray); //like allocation?
    transformedWrenches=tmpSharedPtr;

}

twsGraspQuality::twsGraspQuality()
{
}
