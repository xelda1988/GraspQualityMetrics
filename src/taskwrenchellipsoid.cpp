#include "taskwrenchellispoid.h"
#include <sstream>
#include <fstream>

namespace GraspQm{


void taskwrenchEllipsoid::sampleComEllipsoid()
{

    //sampleParametricForm, might make more sense to sample more in one direction

    //also sample over mass from m-delta -> m=delta

    int iMax=10;
    int jMax=10;
    int lMax=4;
    int k=0;



    for(int i=0; i < iMax; i++){
        for(int j=0; j < jMax; j++){
            for(int l=0; l<lMax;l++){

                //u von -Pi/2 bis Pi/2
                //v von -Pi bis Pi


                double u=-M_PI/2 + ((double) i / (double) iMax)*M_PI;
                double v=-M_PI + ((double) j / (double) jMax)*2*M_PI;
                double w=mass_ - sigmaMass_ + ((double) l / (double) lMax)*2*sigmaMass_;

                Eigen::Vector3d currentCom; //parametrized in PCA frame
                Eigen::Vector3d force;
                Eigen::Vector3d torque;
                Eigen::Vector3d comInRotFrame;

                currentCom(0)=sigmaCom_(0)*cos(u)*cos(v);
                currentCom(1)=sigmaCom_(1)*cos(u)*sin(v);
                currentCom(2)=sigmaCom_(2)*sin(u);

                //std::cout << currentCom << std::endl;
                //std::cout << u << std::endl;
                //std::cout << v << std::endl;

                force=-9.81*w*gripperRot_*gravityNormal_;//use SI Units for now, transform into hand frame, not sure about pose!
                //comInRotFrame=comRotFrame_*currentCom; // sampling done in the pca frame!

                if(l==0){force_min=force;}
                if(l==lMax-1){force_max=force;}


                torque=currentCom.cross(force);
                comEllipse_.col(k)=comInRotFrame;
                sampledPointsEllipse_.col(k)=torque;
                k++;
                //
            }

        }
    }
}

void taskwrenchEllipsoid::computeTorqueEllipsoid()
{

    //Computing pca on the pointset and scale such that it encloses!
    Eigen::Vector3d mean=sampledPointsEllipse_.rowwise().mean();
    MatSampledPts aligned = sampledPointsEllipse_.colwise() - mean;
    Eigen::Matrix3d covMat=aligned*aligned.transpose(); //maybe already enough??
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covMat);
    Eigen::Vector3d eigenValues=eigensolver.eigenvalues();
    Eigen::Matrix3d eigenVectors=eigensolver.eigenvectors();

    //Now compute scaling of Eigenvectors such to have a circumscribed ellipsoid. transforming
    //Points into the PCA::Frame and get Max in each direction!

    pcaMat_=eigenVectors;
    pcaEv_=eigenValues;

#if DEBUGQM
    std::cout << "[Debug]:" << std::endl;
    std::cout << "mean: " << mean.transpose() << std::endl;
    std::cout << "eigenVectors: " << eigenVectors << std::endl;
    std::cout << "eigenValues: " << eigenValues.transpose() << std::endl;



    //Transform Pointset to EigenFrame
    //getting the max in the eigenvectorFrame

    //check if right handed rotation matrix!
    std::cout << "Determinant" << std::endl;
    std::cout << pcaMat_.determinant() << std::endl;
#endif

    MatSampledPts transf=pcaMat_.inverse()*aligned; //why inverse seems to work?? debug that more!!
    Eigen::Vector3d max3D=transf.rowwise().maxCoeff();
    //get Enclosed Ellipse by scaling Eigenvectors with maxCoeff
    double delta=0.00001;
    Eigen::Matrix3d scalMat=Eigen::Vector3d(pow(max3D(0)+delta,-2),pow(max3D(1)+delta,-2),pow(max3D(2)+delta,-2)).asDiagonal();
    Eigen::Vector3d maxScale=scalMat*eigenValues;//eigenvalues in new frame, why does the ordering change?!
    double scale=maxScale.maxCoeff();

#if DEBUGQM
    std::cout << "max3D" << max3D << std::endl;
    std::cout << "maxScale" << maxScale<<std::endl;
    std::cout << "scale" << scale<<std::endl;

#endif
    pcaEv_ << pow((eigenValues(0)+0)/scale,0.5), pow(eigenValues(1)/scale,0.5),pow(eigenValues(2)/scale,0.5); //Now scaled to halfaxes!

    pca_to_ellipse_.rotation=pcaMat_;
    pca_to_ellipse_.translation<<0,0,0;


}

void taskwrenchEllipsoid::getLinearTransform(Transform & hand_to_ellipse,
                                             Eigen::Vector3d & semiAxesTorque,
                                             Eigen::Vector3d & semiAxesForces,
                                             Eigen::Vector3d & forceOffset){

    //get transformation hand to ellipse:

    pca_to_ellipse_.setHomFromRotTrans();
    camera_to_pca_.setHomFromRotTrans();
    camera_to_world_.setHomFromRotTrans();
    world_to_hand_.setHomFromRotTrans();
#if DEBUGQM
    std::cout << "pca_to_ellipse_" << std::endl;
    std::cout << pca_to_ellipse_.hom << std::endl;
    std::cout << "camera_to_pca_:" << std::endl;
    std::cout << camera_to_pca_.hom << std::endl;
    std::cout << "camera_to_world:" << std::endl;
    std::cout << camera_to_world_.hom << std::endl;
    std::cout << "world_to_hand_:" << std::endl;
    std::cout << world_to_hand_.hom << std::endl;
#endif
    //test
    hand_to_ellipse.hom = ((pca_to_ellipse_.hom)*(camera_to_pca_.hom.inverse())*(camera_to_world_.hom)*(world_to_hand_.hom)).inverse();
    hand_to_ellipse.setFromHomog();

    semiAxesTorque = pcaEv_ + noiseTorque_;
    semiAxesForces = pca_to_ellipse_.rotation.inverse()*(force_max-force_min)/2 + noiseForce_; //create a translation offset

    //bind the tws first by a cone, then by its steiner ellipse - which accounts both for scaling in magnitude and noise

    forceOffset=(force_max+force_min)*0.5; // translate force ellipse such that non-force closure grasp is possible!

#if DEBUGQM

    std::cout << "Debug, force vector in pca frame" << std::endl;
    std::cout << pca_to_ellipse_.rotation.inverse()*force_max << std::endl;

#endif
    hand_to_ellipse_=hand_to_ellipse;
    semiAxesTorque_=semiAxesTorque;
    semiAxesForces_=semiAxesForces;
    forceOffset_=forceOffset;
    ///go on from here!!!!!

}

void taskwrenchEllipsoid::getHandToEllipse(Transform & hand_to_ellipse){
    pca_to_ellipse_.setHomFromRotTrans();
    camera_to_pca_.setHomFromRotTrans();
    camera_to_world_.setHomFromRotTrans();
    world_to_hand_.setHomFromRotTrans();

    hand_to_ellipse.hom = ((pca_to_ellipse_.hom)*(camera_to_pca_.hom.inverse())*(camera_to_world_.hom)*(world_to_hand_.hom)).inverse();
    hand_to_ellipse.setFromHomog();
    hand_to_ellipse_=hand_to_ellipse;
}


void taskwrenchEllipsoid::writeSampledPts(const std::string & filename)
{
    std::ofstream of(filename.c_str());
    if(of.is_open()){

        of << sampledPointsEllipse_.transpose();
        //of << comEllipse_.transpose();

    }
    of.close();


}
void taskwrenchEllipsoid::writeTaskEllipse(const std::string & filename){

    std::ofstream of(filename.c_str());
    if(of.is_open()){

        of << pcaMat_ << std::endl;
        of << pcaEv_.transpose() << std::endl;
    }
    of.close();
}

void taskwrenchEllipsoid::writeForceTorqueEllipse(const std::string & filename){

    std::ofstream of(filename.c_str());
    if(of.is_open()){

        of << hand_to_ellipse_.hom << std::endl;
        of << semiAxesForces_.transpose() << " 0" << std::endl;
        of << semiAxesTorque_.transpose() <<  " 0" << std::endl;
        of << (pca_to_ellipse_.rotation.inverse()*forceOffset_).transpose() <<  " 0" << std::endl; //to translate to the ellipse frame!
    }
    of.close();
}

void taskwrenchEllipsoid::writeTransformedPts(const std::string & filename)
{
    std::ofstream of(filename.c_str());
    if(of.is_open()){

        //torquesamples - offset ? points are in pca frame, not ellipse frame!

        //first transform these guys into the ellipse frame!!

        MatSampledPts translated=sampledPointsEllipse_.colwise() - hand_to_ellipse_.rotation.inverse()*hand_to_ellipse_.translation;
        std::cout << hand_to_ellipse_.translation.transpose() << std::endl;
        std::cout << hand_to_ellipse_.rotation.inverse() << std::endl;
        std::cout << hand_to_ellipse_.hom << std::endl;

        std::cout << translated.transpose() << std::endl;
        of << (hand_to_ellipse_.rotation.inverse()*pca_to_ellipse_.rotation.inverse()*(translated)).transpose();
        //of << comEllipse_.transpose();

    }
    of.close();
}

}//ns


