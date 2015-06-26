#ifndef WRENCHSPACE_H
#define WRENCHSPACE_H

#include <Eigen/StdVector>
#include <list>
#include <vector>
#include <tr1/memory>
#include <Eigen/Core>
#include <iostream>
#include <libqhullcpp/Qhull.h>

#define EPSILON_WRENCH_CONE_ROTATION 1e-10
#define EPSILON_UNIT_NORMAL 1e-6
#define EPSILON_FORCE_CLOSURE 1e-10
#define PI 3.14159265358979323846264338327950
#define NOT_VISITED         -1
#define NOT_EXPLORED         0
#define EXPLORED_QUALIFIED   1
#define EXPLORED_UNQUALIFIED 2


typedef unsigned int uint;
typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6Xd;
typedef Eigen::Matrix<uint,1, Eigen::Dynamic > RowVectorXui;
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<uint,Eigen::Dynamic,6> MatrixX6ui;
typedef Eigen::Array<uint,Eigen::Dynamic,6> ArrayX6ui;
typedef Eigen::Array<uint,1,6> Array6ui;
typedef Eigen::Matrix<uint,Eigen::Dynamic,1> VectorXui;
typedef std::list<uint> IndexList;
typedef std::list<uint>::iterator IndexListIterator;
typedef std::list<uint>::const_iterator ConstIndexListIterator;
typedef std::tr1::shared_ptr<double>  SharedDoublePtr;

typedef std::tr1::shared_ptr<Eigen::Vector3d> Vector3dPtr;

/*!
 *  \brief A discrete 6D-wrench space used to describe the Grasp Wrench Space
 */
class DiscreteWrenchSpace
{

private:

    bool full_dim_;
    bool contains_origin_;
    /*!
    *  The radius of the largest origin-centered ball contained by the convex hull of the wrench space
    */
    double r_oc_insphere_;
    double volume_;
    double area_;
    uint dimension_;

    bool ch_computed_;
    orgQhull::Qhull conv_hull_;
    uint num_wrenches_;
    uint num_vtx_;
    uint num_facets_;
    SharedDoublePtr wrenches_;

public:

    DiscreteWrenchSpace();
    DiscreteWrenchSpace(uint dimension);
    DiscreteWrenchSpace(uint dimension,SharedDoublePtr wrenches,uint num_wrenches);
    //DiscreteWrenchSpace(DiscreteWrenchSpace const& src);
    //DiscreteWrenchSpace& operator=(DiscreteWrenchSpace const& src);

    friend std::ostream& operator<<(std::ostream& stream,DiscreteWrenchSpace const& d_wrench_space);

    /*!
 *  Uses Qhull to compute the convex hull over wrenches
 */
    bool isFullDimension()const;
    bool containsOrigin()const;
    double getOcInsphereRadius()const{return r_oc_insphere_;}
    double getVolume()const;
    double getArea()const;
    uint getDimension()const;

    void computeConvexHull();
    orgQhull::Qhull const* getConvexHull()const;
    bool convHullComputed()const;
    uint getNumWrenches()const;
    uint getNumVertices()const;
    uint getNumFacets()const;
    SharedDoublePtr getWrenches()const;
    bool writeToFile(const std::string& path)const;
    bool writeToOffFile(const std::string& path)const;
    double computeDistToHull(SharedDoublePtr wrenches)const;
    void setWrenches(uint dim, SharedDoublePtr wrenches,uint num_wrenches);
};

typedef std::tr1::shared_ptr<DiscreteWrenchSpace> DiscreteWrenchSpacePtr;

#endif // WRENCHSPACE_H
