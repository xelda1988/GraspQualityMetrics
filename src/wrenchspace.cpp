#include "wrenchspace.h"
#include <iostream>
#include <math.h>
#include "assert.h"

//---------------------------------------------------------------------------------
DiscreteWrenchSpace::DiscreteWrenchSpace() : ch_computed_(false),num_wrenches_(0),num_vtx_(0),num_facets_(0)
{
//  type_=Discrete;
}
//---------------------------------------------------------------------------------
DiscreteWrenchSpace::DiscreteWrenchSpace(uint dimension) : ch_computed_(false),num_wrenches_(0),num_vtx_(0),num_facets_(0)
{
//  type_=Discrete;
}
//---------------------------------------------------------------------------------
DiscreteWrenchSpace::DiscreteWrenchSpace(uint dimension,SharedDoublePtr wrenches, uint num_wrenches) : ch_computed_(false),num_wrenches_(0),num_vtx_(0),num_facets_(0)
{
  assert(num_wrenches > 0);
  assert(wrenches.get() != NULL);
  wrenches_=wrenches;
  num_wrenches_=num_wrenches;
//  type_=Discrete;
}
//--------------------------------------------------------------------------
//DiscreteWrenchSpace::DiscreteWrenchSpace(DiscreteWrenchSpace const& src) : WrenchSpace(src),ch_computed_(src.ch_computed_),
//                                       conv_hull_(src.conv_hull_),num_wrenches_(src.num_wrenches_),
//                                       num_vtx_(src.num_vtx_),num_facets_(src.num_facets_),wrenches_(src.wrenches_){}
//--------------------------------------------------------------------------
//DiscreteWrenchSpace& DiscreteWrenchSpace::operator=(DiscreteWrenchSpace const& src)
//{
//  if (this !=&src)
//    {
//  WrenchSpace::operator=(src);
//  ch_computed_=src.ch_computed_;
//  conv_hull_=src.conv_hull_;
//  num_wrenches_=src.num_wrenches_;
//  num_vtx_=src.num_vtx_;
//  num_facets_=src.num_facets_;
//  wrenches_=src.wrenches_;
//    }
//  return *this;
//}
//--------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& stream, DiscreteWrenchSpace const& d_wrench_space)
{
//  std::string wrench_space_type;
//  if (d_wrench_space.type_==Discrete) wrench_space_type="Discrete";
//  else wrench_space_type="Warning in DiscreteWrenchSpace: Invalid rule type!";

  stream <<'\n'<<"DISCRETE WRENCH SPACE: "<<'\n'
//     <<"Wrench space type: "<<wrench_space_type<<'\n'
     <<"Dimension: "<<d_wrench_space.dimension_<<'\n'
     <<"Convex hull computed: "<<d_wrench_space.ch_computed_<<'\n'
     <<"Contains origin: "<<d_wrench_space.contains_origin_<<'\n'
     <<"Has full dimension: "<<d_wrench_space.full_dim_<<'\n'
     <<"Origin-centered insphere radius: "<<d_wrench_space.r_oc_insphere_<<'\n'
     <<"Volume: "<<d_wrench_space.volume_<<'\n'
     <<"Area: "<<d_wrench_space.area_<<'\n'
     <<"Number of input wrenches: "<<d_wrench_space.num_wrenches_<<'\n'
     <<"Number of vertices: "<<d_wrench_space.num_vtx_<<'\n'
     <<"Number of facets: "<<d_wrench_space.num_facets_<<'\n'<<'\n';

  return stream;
}
//--------------------------------------------------------------------------
//DiscreteWrenchSpace::~DiscreteWrenchSpace(){}
//--------------------------------------------------------------------------
bool DiscreteWrenchSpace::convHullComputed()const{return ch_computed_;}
//--------------------------------------------------------------------------
orgQhull::Qhull const* DiscreteWrenchSpace::getConvexHull()const
{
  assert(ch_computed_);
  return &conv_hull_;
}
//--------------------------------------------------------------------------
void DiscreteWrenchSpace::computeConvexHull()
{
  assert(wrenches_.get() != NULL);
  assert(num_wrenches_ > dimension_);

  try{
    conv_hull_.runQhull("", dimension_,num_wrenches_,wrenches_.get() ,"Q0 Qt"); //Qx doesn't merge coplanar facets, Q0 does, QJ joggles
  }
  catch(std::exception& exc)
    {
#ifdef DEBUG_QHULL
  std::cout<<exc.what()<<std::endl;
#endif
  contains_origin_=false;
  full_dim_=false;
  ch_computed_=true;
  return;
    }

  conv_hull_.defineVertexNeighborFacets();
  area_=conv_hull_.area();
  volume_=conv_hull_.volume();
  num_vtx_=conv_hull_.vertexCount();
  num_facets_=conv_hull_.facetCount();

  facetT* curr_f=conv_hull_.beginFacet().getFacetT();
  r_oc_insphere_=-(curr_f->offset);
  for(uint i=0;i< num_facets_;i++)
    {
  curr_f->id=i; //Replaces the Qhull runtime indexing with indices 0 - num_facets_
  r_oc_insphere_ = (-(curr_f->offset) < r_oc_insphere_) ? -(curr_f->offset) : r_oc_insphere_;
  curr_f=curr_f->next;
    }

  contains_origin_=(r_oc_insphere_ > EPSILON_FORCE_CLOSURE) ? true : false;
  full_dim_=true;
  ch_computed_=true;
}
//--------------------------------------------------------------------------
uint DiscreteWrenchSpace::getNumWrenches()const{return num_wrenches_;}
//--------------------------------------------------------------------------
uint DiscreteWrenchSpace::getNumVertices()const{return num_vtx_;}
//--------------------------------------------------------------------------
uint DiscreteWrenchSpace::getNumFacets()const{return num_facets_;}
//--------------------------------------------------------------------------
SharedDoublePtr DiscreteWrenchSpace::getWrenches()const{return wrenches_;}
//--------------------------------------------------------------------------
void DiscreteWrenchSpace::setWrenches(SharedDoublePtr wrenches,uint num_wrenches)
{
  assert(num_wrenches > 0);
  assert(wrenches.get() != NULL);
  wrenches_=wrenches;
  num_wrenches_=num_wrenches;
}
//---------------------------------------------------------------------------------
bool DiscreteWrenchSpace::writeToFile(const std::string& path)const
{
  if (!ch_computed_)
    {
  std::cout<<"Warning in DiscreteWrenchSpace::writeToFile(const std::string& path)const - Convex hull not computed, cannot write to file"<<std::endl;
  return false;
    }

  remove(path.c_str());
  FILE* hp=fopen (path.c_str(),"a");
  if(!hp)
    {
  std::cout<<"Warning in DiscreteWrenchSpace::writeToFile(const std::string& path)const - Couldn't write to file"<<std::endl;
  return false;
    }

  facetT* curr_f=conv_hull_.beginFacet().getFacetT();
  for(uint i=0;i< conv_hull_.facetCount();i++)
    {
  if (dimension_==3)
    fprintf(hp, "% f % f % f % f  \n",-(curr_f->normal)[0],-(curr_f->normal)[1],-(curr_f->normal)[2],-(curr_f->offset));
  else if (dimension_==6)
    fprintf(hp, "% f % f % f % f % f % f %f \n",-(curr_f->normal)[0],-(curr_f->normal)[1],-(curr_f->normal)[2],-(curr_f->normal)[3],-(curr_f->normal)[4],-(curr_f->normal)[5],-(curr_f->offset));
  else
    {
      std::cout<<"Warning in DiscreteWrenchSpace::writeToFile(const std::string& path)const - cann only write 3- or 6D hyperplanes to file!"<<std::endl;
      return false;
    }
  curr_f=curr_f->next;
    }
  fclose (hp);
  return true;
}
