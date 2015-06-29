#include "wrenchspace.h"
#include <iostream>
#include <math.h>
#include "assert.h"

namespace GraspQm{


//---------------------------------------------------------------------------------
DiscreteWrenchSpace::DiscreteWrenchSpace() : ch_computed_(false),num_wrenches_(0),num_vtx_(0),num_facets_(0)
{
//  type_=Discrete;
}
//---------------------------------------------------------------------------------
DiscreteWrenchSpace::DiscreteWrenchSpace(uint dimension) : ch_computed_(false),num_wrenches_(0),num_vtx_(0),num_facets_(0)
{
//  type_=Discrete;
    dimension_=dimension;
}
//---------------------------------------------------------------------------------
DiscreteWrenchSpace::DiscreteWrenchSpace(uint dimension,SharedDoublePtr wrenches, uint num_wrenches) : ch_computed_(false),num_wrenches_(0),num_vtx_(0),num_facets_(0)
{
  assert(num_wrenches > 0);
  assert(wrenches.get() != NULL);
  wrenches_=wrenches;
  num_wrenches_=num_wrenches;
  dimension_=dimension;
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
  // #define DEBUG_QHULL
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

  //test

  //

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
void DiscreteWrenchSpace::setWrenches(uint dimension, SharedDoublePtr wrenches,uint num_wrenches)
{
  assert(num_wrenches > 0);
  assert(wrenches.get() != NULL);
  wrenches_=wrenches;
  num_wrenches_=num_wrenches;
  dimension_=dimension;
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

bool DiscreteWrenchSpace::writeToOffFile(const std::string& path)const
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

  vertexT* curr_v=conv_hull_.beginVertex().getVertexT();

  fprintf(hp, "OFF \n%i %i 0\n", conv_hull_.vertexCount(), conv_hull_.facetCount());

//  for(uint i=0; i<conv_hull_.vertexCount(); i++){
//      std::cout << "Id: " <<  i << " - " << qh_pointid(curr_v->point) <<  std::endl;
//      curr_v=curr_v->next;
//  }
  curr_v=conv_hull_.beginVertex().getVertexT();

  for(uint i=0; i < conv_hull_.vertexCount(); i++){

      //if (dimension_==3)
      //order the vertices by id! bug in qhull???
      //if(i==4)  fprintf(hp, "0 0 0\n");
      //for(uint j=0; j< conv_hull_.vertexCount(); j++){
      //std::cout << i << std::endl;
      //Rather set id
      //if(i==qh_pointid(curr_v->point)){
      //writing dummy index at place nr 4, no idea check if it changes for other hulls
      //fprintf(hp, "%f %f %f %i\n",(curr_v->point)[0],(curr_v->point)[1],(curr_v->point)[2],curr_v->id);
      //std::cout << "POINTID " << qh_pointid(curr_v->point) << std::endl;
      //setting point id
      curr_v->id=i;
      fprintf(hp, "%f %f %f \n",(curr_v->point)[0],(curr_v->point)[1],(curr_v->point)[2]);

      //break;
      //}
      curr_v=curr_v->next;
  }
      //curr_v=conv_hull_.beginVertex().getVertexT();
  //}

  facetT* curr_f=conv_hull_.beginFacet().getFacetT();

  for(uint i=0;i< conv_hull_.facetCount();i++)
  {
      if (dimension_==3){

          setT* vertSet=curr_f->vertices;
          fprintf(hp, "%i ",vertSet->maxsize);
          for(int j=0; j < vertSet->maxsize; j++){

              //writing fck vertexptr
              //setelemT* currV = ;
              vertexT* currVert = (vertexT*) vertSet->e[j].p;
              fprintf(hp, "%i ",currVert->id);

          }
          fprintf(hp, "\n");
      }

      //fprintf(hp, "% f % f % f % f  \n",-(curr_f->vertices)[0],-(curr_f->normal)[1],-(curr_f->normal)[2],-(curr_f->offset));
      else
      {
          std::cout<<"Warning in DiscreteWrenchSpace::writeToFile(const std::string& path)const - cann only write 3-D facets to off sfile!"<<std::endl;
          return false;
      }
      curr_f=curr_f->next;
  }
  fclose (hp);

  return true;
}


double DiscreteWrenchSpace::computeDistToHull(SharedDoublePtr wrenches)const{

    //compute signed distance to every facet! - for now only for a single wrench

    //std::vector<double> dists;
    facetT* curr_f=conv_hull_.beginFacet().getFacetT();

    double minDist(100);

    for(uint i=0;i < conv_hull_.facetCount();i++)
    {

        //fprintf(hp, "% f % f % f % f  \n",-(curr_f->vertices)[0],-(curr_f->normal)[1],-(curr_f->normal)[2],-(curr_f->offset));
        double sumPx(0),sqSum(0);
        for(uint j=0; j < dimension_; j++){
            sumPx += wrenches.get()[j] * (curr_f->normal)[j];
            sqSum += (curr_f->normal)[j] * (curr_f->normal)[j];
        }

        double dist = -(sumPx + curr_f->offset)/pow(sqSum,0.5);
        std::cout << "minDist" << dist << std::endl;

        if(dist<minDist) minDist=dist;

        curr_f=curr_f->next;
    }

    return minDist;
}

}//ns
