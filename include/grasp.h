#ifndef GRASP_H
#define GRASP_H

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include "grasp.h"

//Grasp defined by list of contact points with normals and friction

namespace GraspQm{

typedef Eigen::Matrix<double,6,1> Vector6d;
typedef unsigned int uint;

class Grasp{

private:

    std::vector<Vector6d> contacts_;
    uint nr_contacts_;
    uint id_;
    double friction_;

public:

    void setFriction(double friction){
        friction_=friction;
    }
    void setId(uint id){
        id_=id;
    }
    void setContacts(const std::vector<Vector6d> & contacts){
        contacts_=contacts;
        nr_contacts_=contacts.size();
    }

    void addContact(const Vector6d & contact){
        contacts_.push_back(contact);
    }

    double getMaxTorqueArm();

    double getFriction(){

        return friction_;
    }

    std::vector<Vector6d> getContacts(){
        return contacts_;
    }

    uint getNrContacts()
    {
        return contacts_.size();

    }

    //Constr
    Grasp(): nr_contacts_(0), id_(0), friction_(0.5){}
    //Destr
};


}//ns

#endif // GRASP_H
