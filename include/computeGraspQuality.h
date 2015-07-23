#ifndef COMPUTEGRASPQUALITY_H
#define COMPUTEGRASPQUALITY_H

#include "grasp.h"
#include "wrench_cone.h"
#include "wrenchspace.h"
#include "taskwrenchellispoid.h"
#include "twsgraspquality.h"
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

//create only a single instance
//recomputation of the force-torque ellipsoid should not be needed

//Passed to the algorithm is a Grasp, all the Transforms

namespace GraspQm{

//Functions for recomputing one thing
//Function for recomputation!: hand_to_ellipse
typedef boost::shared_ptr<Grasp> GraspPtr;
class ComputeGraspQuality{

private:

    //Input:

    //Output Learning
    double mass; //kg
    double sigmaMass; //kg
    Eigen::Vector3d com; //center of mass in which frame??
    Eigen::Vector3d sigmaCom; //uncertainty center of mass in meters
    Eigen::Matrix3d camera_to_pca_rot;
    Eigen::Vector3d gravity_normal; //in the frame of the camera

    //Transforms
    Transform camera_to_world;
    Transform world_to_hand;

    //settings hand and environment
    double maxForceHand; //maximum contact force in each contact is 50 Newtons
    double noiseForce; //1 Newton noise in force space
    double noiseTorque;// in Newon Meters
    uint nrFacets;

    GraspPtr graspPtr;

    //other classes can be generated inside which are not Passed
    WrenchConesAll wrenchCones;
    taskwrenchEllipsoid tws_ell;
    //DiscreteWrenchSpace discreteWrenchspaceTrans;

    DiscreteWrenchSpacePtr discreteWrenchspaceTransPtr;
    twsGraspQuality gwsTransform;


    ///////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////

    //intermediate Variables:
    Transform hand_to_ellipse;
    Eigen::Vector3d semiAxesTorque;
    Eigen::Vector3d semiAxesForces;
    Eigen::Vector3d forceOffset; //in pca frame!

    std::vector<Vector6d> wrenchVec;
    SharedDoublePtr wrenchPtrTrans;
    uint nrWrenches;

    //output:
    double roc_insphere;

    //has to be done once only, maybe with higher resolution then

    void computeTwsEll(){

        tws_ell.setInput(mass,sigmaMass, com, sigmaCom, camera_to_pca_rot);
        tws_ell.setNoise(noiseForce,noiseTorque);
        tws_ell.setCameraCalib(camera_to_world);
        tws_ell.setGripperPose(world_to_hand);
        tws_ell.setGravityNormal(gravity_normal);

        tws_ell.sampleComEllipsoid();
        tws_ell.computeTorqueEllipsoid();
        //output

        tws_ell.getLinearTransform(hand_to_ellipse, semiAxesTorque, semiAxesForces,forceOffset);
    }



    void computeWrenchCones(){
        wrenchCones.setGrasp(*graspPtr);
        wrenchCones.setMaxForce(maxForceHand);
        wrenchCones.computeAllWrenchCones(nrFacets);
        wrenchCones.addZeroToWrenches(); //adding zero force grasp

        wrenchVec = wrenchCones.get6dEigen();
        //std::cout << "Adress " << &wrenchVec << std::endl;
    }
    //periodic
    void recomputeHandToEllipse(){
        tws_ell.setGripperPose(world_to_hand);
        tws_ell.getHandToEllipse(hand_to_ellipse);
    }

    void transformWrenches(){
#ifdef DEBUGGQM
        std::cout << "Nr Wrenches before Computation" << std::endl;
        std::cout << wrenchVec.size() << std::endl;
        std::cout << "Adress " << &wrenchVec << std::endl;
#endif
        gwsTransform.setParams(wrenchVec,
                               hand_to_ellipse,
                               semiAxesTorque,
                               semiAxesForces,
                               forceOffset);
        gwsTransform.computeTransform();
        //gwsTransform.getOutput(wrenchPtrTrans,nrWrenches);

        std::vector<double> wrenchTransformed=gwsTransform.getVector();
#ifdef DEBUGGQM
        std::cout << "Nr Wrenches after Computation" << std::endl;
        std::cout << wrenchTransformed.size()/6 << std::endl;
#endif
        int vecSize = wrenchTransformed.size();
        double* copyVec = new double[vecSize];
        //double copyVec[vecSize];
        for(int i = 0; i < vecSize; ++i)
            copyVec[i] = wrenchTransformed.at(i);

       // std::fill_n(a, 6, 0);
        //SharedDoublePtr xx(a);
        wrenchPtrTrans=SharedDoublePtr (copyVec);
        nrWrenches=wrenchTransformed.size()/6;

    }

    void coutSharedPtr(){

        uint dim=6;
        for(uint i=0; i < nrWrenches*dim; i++)
        {
            //cout << data.get()[i] << " ";
            //if((i+1)%6==0){cout << endl;}

            std::cout << wrenchPtrTrans.get()[i] << " ";
            if((i+1)%6==0){std::cout << endl;}
        }
    }


    void computeGWS(){

        //coutSharedPtr();

        //DiscreteWrenchSpace * test = new DiscreteWrenchSpace;
        //discreteWrenchspaceTrans;
        //DiscreteWrenchSpace tmp;
        //discreteWrenchspaceTrans = tmp;
#ifdef DEBUGGQM
        std::cout << "Before Setting" <<endl;
        std::cout << *discreteWrenchspaceTransPtr <<endl;
#endif
        discreteWrenchspaceTransPtr->setWrenches(6,wrenchPtrTrans,nrWrenches);

#ifdef DEBUGGQM
        std::cout << "After Setting" <<endl;
        std::cout << *discreteWrenchspaceTransPtr <<endl;
#endif
        //discreteWrenchspaceTrans.
        discreteWrenchspaceTransPtr->computeConvexHull();
#ifdef DEBUGGQM
        std::cout << *discreteWrenchspaceTransPtr <<endl;
#endif
        roc_insphere=discreteWrenchspaceTransPtr->getOcInsphereRadius();
    }

public:

    //Constructor to set most important variables:

    ComputeGraspQuality(){
        //set a bunch of parameters
        maxForceHand=50;
        noiseForce=1;
        noiseTorque=noiseForce*0.05;
        nrFacets=8;
    }

    void setInput(
            double mass_in,
            double sigmaMass_in,
            const Eigen::Vector3d & com_in,
            const Eigen::Vector3d & sigmaCom_in,
            const Eigen::Matrix3d & camera_to_pca_rot_in,
            const Eigen::Vector3d & gravity_normal_in,
            const Grasp & grasp,
            const Transform & camera_to_world_in ){

        std::cout << "debugSetInput1\n" << std::flush;

        mass=mass_in;
        std::cout << "debugSetInput2\n" << std::flush;
        sigmaMass=sigmaMass_in;
        std::cout << "debugSetInput3\n" << std::flush;
        com=com_in;
        std::cout << "debugSetInput4\n" << std::flush;
        sigmaCom=sigmaCom_in;
        camera_to_pca_rot = camera_to_pca_rot_in;
        gravity_normal=gravity_normal_in;


        std::cout << "debugSetInput1" << std::flush;
        graspPtr.reset(new Grasp(grasp));
        camera_to_world=camera_to_world_in;

        std::cout << "debugSetInput" << std::flush;
    }

    void changeStdSettings(double maxForceHand_in,
                           double noiseForce_in,
                           double noiseTorque_in,
                           uint nrFacets_in){

        maxForceHand=maxForceHand_in;
        noiseForce=noiseForce_in;
        noiseTorque=noiseTorque_in;
        nrFacets=nrFacets_in;
    }

    void setPose(const Transform & world_to_hand_in){
        world_to_hand=world_to_hand_in;
    }
    void initComputation(){


        discreteWrenchspaceTransPtr.reset(new DiscreteWrenchSpace());
        computeTwsEll();
        std::cout << "bla" << std::endl;
        computeWrenchCones();
        std::cout << "bla1" << std::endl;
        transformWrenches();
        std::cout << "bla3" << std::endl;
        computeGWS();
        std::cout << "bla4" << std::endl;

    }
    void recomputeQM(){

        recomputeHandToEllipse();
        transformWrenches();
        discreteWrenchspaceTransPtr.reset(new DiscreteWrenchSpace());
        computeGWS();
    }

    double getQM(){
        return roc_insphere;
    }

    void printInputVars(){

        std::cout << "Mass: " << mass << std::endl;
        std::cout << "SigmaMass: " << sigmaMass << std::endl;
        std::cout << "Center of Mass (pcaFrame):\n" << com.transpose() << std::endl;
        std::cout << "SigmaCenterofMass (pcaFrame):\n" << sigmaCom.transpose() << std::endl;
        std::cout << "camera_to_pca_rot:\n" << camera_to_pca_rot << std::endl;
        std::cout << "gravity_normal (camFrame):\n" << camera_to_pca_rot << std::endl;

        std::cout << "CamToWorld:\n" << camera_to_world.hom << std::endl;
        std::cout << "WorldToHand:\n" << world_to_hand.hom << std::endl;


        std::cout << "maxForceHand (N): " << maxForceHand << std::endl;
        std::cout << "noiseForce (N): " << noiseForce << std::endl;
        std::cout << "noiseTorque (Nm): " << noiseTorque << std::endl;

        std::cout << "nrFacets: " << nrFacets << std::endl;

        std::cout << "GraspContacts: " << std::endl;
        for(uint i =0; i <  graspPtr->getContacts().size(); i++){
            std::cout << graspPtr->getContacts().at(i).transpose() << std::endl;
        }
    }
};

}//ns


#endif // COMPUTEGRASPQUALITY_H
