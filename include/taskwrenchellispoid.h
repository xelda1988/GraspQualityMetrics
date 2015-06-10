#ifndef TASKWRENCHELLISPOID_H
#define TASKWRENCHELLISPOID_H

#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <list>
#include <vector>
#include <tr1/memory>
#include <Eigen/Core>
#include <iostream>

/*
  Input: one function is to sample a paramtric form of the task wrench space
  Check speed of 3D eigenDecomposition, if not enough switch to 2D or if degeneration makes problems

  Here I use fixed size 3D array by sampling from a 2D parametric form: 20x20 = 400 points as input for pca

  input m,com, dm, SigmaCom as Complete Matrix

  Returns Transformation Matrix and Linear Transformation for Wrenches

  */

typedef Eigen::Matrix<double, 3, 400> MatSampledPts;

struct Transform{
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    Eigen::Matrix4d hom;
    void inverse(){
        translation=-1*translation;
        rotation=rotation.inverse();
        hom.inverse();
    }
    void setFromHomog(){
        translation=hom.col(3).head(3);
        rotation=hom.block<3,3>(0,0);
    }
    void setHomFromRotTrans(){
        hom.col(0) << rotation.col(0),0;
        hom.col(1) << rotation.col(1),0;
        hom.col(2) << rotation.col(2),0;
        hom.col(3) << translation,1;
    }
};


class taskwrenchEllipsoid{

private:

    MatSampledPts sampledPointsEllipse_;
    MatSampledPts comEllipse_;
    double mass_;
    double sigmaMass_;
    Eigen::Vector3d com_;
    Eigen::Vector3d sigmaCom_;
    Eigen::Matrix3d camera_to_pca_rot_;
    Eigen::Matrix3d gripperRot_; //setting pose
    Eigen::Vector3d gravityNormal_;//unit vector parallel to gravity

    //Transformation writing properly

    Transform world_to_hand_; //Gripper Pose
    Transform camera_to_world_; //Camera calibration
    Transform camera_to_pca_; //Transformation to the frame of the pca of the Center of mass estimation
    Transform pca_to_ellipse_; //Transformation to the ellipse frame, which are the Eigenvectors of the torque ellispoid.

    //Noise for forces and torques
    Eigen::Vector3d noiseForce_;
    Eigen::Vector3d noiseTorque_;

    //output:
    Eigen::Matrix3d pcaMat_;
    Eigen::Vector3d pcaEv_;



public:

    void setInput(const double & mass,
                  const double sigmaMass,
                  const Eigen::Vector3d & com,
                  const Eigen::Vector3d & sigmaCom,
                  const Eigen::Matrix3d & camera_to_pca_rot){
        mass_=mass;
        sigmaMass_=sigmaMass;
        com_=com;
        sigmaCom_=sigmaCom;
        camera_to_pca_rot_=camera_to_pca_rot;

        //setting transforms
        camera_to_pca_.rotation=camera_to_pca_rot_;
        camera_to_pca_.translation=com_;
    }

    void setGripperPose(const Transform & world_to_hand){
        world_to_hand_=world_to_hand;
    }

    void setCameraCalib(const Transform & camera_to_world){
        camera_to_world_=camera_to_world;
    }

    //e.g 1N nf. 0.1Nm nt
    void setNoise(double nf, double nt){
        noiseForce_=Eigen::Vector3d(nf,nf,nf);
        noiseTorque_=Eigen::Vector3d(nt,nt,nt);
    }


    void sampleComEllipsoid(); //Sampling from Parametric form of Ellipsoid
    void computeTorqueEllipsoid();

    void getLinearTransform(Transform & hand_to_ellipse,
                            Eigen::Vector3d & semiAxesTorque,
                            Eigen::Vector3d & semiAxesForces);

    void writeSampledPts(const std::string & filename);
    void writeTaskEllipse(const std::string & filename);

    taskwrenchEllipsoid(){
        gravityNormal_<<0,0,1;
        gripperRot_=Eigen::Matrix3d::Identity();
    }




};

#endif // TASKWRENCHELLISPOID_H
