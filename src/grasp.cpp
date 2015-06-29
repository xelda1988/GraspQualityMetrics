#include "grasp.h"

namespace GraspQm{

double Grasp::getMaxTorqueArm(){

    double largestDist=0;

    for(int i=0; i < contacts_.size(); i++ )
    {
        double dist = contacts_[i].head(3).norm();
        if(dist > largestDist)
            largestDist=dist;

    }
    return largestDist;
}

}//ns
