#include "taskwrenchellispoid.h"
#include <sstream>
#include <fstream>

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

                Eigen::Vector3d currentCom; //parametrized in PCA frame;
                Eigen::Vector3d force;
                Eigen::Vector3d torque;
                Eigen::Vector3d comInRotFrame;

                currentCom(0)=sigmaCom_(0)*cos(u)*cos(v);
                currentCom(1)=sigmaCom_(1)*cos(u)*sin(v);
                currentCom(2)=sigmaCom_(2)*sin(u);

                //std::cout << currentCom << std::endl;
                std::cout << u << std::endl;
                std::cout << v << std::endl;

                force=9.81*w*gripperRot_*gravityNormal_;//use SI Units for now, transform into hand frame, not sure about pose!
               //comInRotFrame=comRotFrame_*currentCom; // sampling done in the pca frame!

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

    std::cout << "Debug:" << std::endl;
    std::cout << "mean: " << mean.transpose() << std::endl;
    std::cout << "eigenVectors: " << eigenVectors << std::endl;
    std::cout << "eigenValues: " << eigenValues.transpose() << std::endl;
    pcaMat_=eigenVectors;
    pcaEv_=eigenValues;

    //Transform Pointset to EigenFrame
    //getting the max in the eigenvectorFrame

    MatSampledPts transf=pcaMat_*aligned;
    Eigen::Vector3d max3D=transf.rowwise().maxCoeff();
    //get Enclosed Ellipse by scaling Eigenvectors with maxCoeff
    double delta=0.0000001;
    Eigen::Matrix3d scalMat=Eigen::Vector3d(pow(max3D(0)+delta,-2),pow(max3D(1)+delta,-2),pow(max3D(2)+delta,-2)).asDiagonal();
    Eigen::Vector3d maxScale=scalMat*eigenValues;
    double scale=maxScale.maxCoeff();

    std::cout << "max3D" << max3D << std::endl;
    std::cout << "maxScale" << maxScale<<std::endl;
    std::cout << "scale" << scale<<std::endl;
    pcaEv_ << pow((eigenValues(0)+0)/scale,0.5), pow(eigenValues(1)/scale,0.5),pow(eigenValues(2)/scale,0.5); //Now scaled to halfaxes!

}

void taskwrenchEllipsoid::getLinearTransform(Transform & hand_to_ellipse,
                                             Eigen::Vector3d & semiAxesTorque,
                                             Eigen::Vector3d & semiAxesForces){

    //get transformation hand to ellipse:

    hand_to_ellipse.hom = ((pca_to_ellipse_.hom).inverse()*(camera_to_pca_.hom.inverse())*(camera_to_world_.hom)*(world_to_hand_.hom)).inverse();
    hand_to_ellipse.setFromHomog();

    semiAxesTorque = pcaEv_ + noiseTorque_;
    semiAxesForces = noiseForce_;


    ///go on from here!!!!!

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


