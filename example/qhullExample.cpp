//#include "../../include/icr.h"
#include <iostream>
#include <sys/time.h>
#include <time.h>
#include <libqhullcpp/QhullVertexSet.h>
#include <libqhullcpp/Qhull.h>
#include "libqhullcpp/RboxPoints.h"
#include "libqhullcpp/QhullError.h"
#include "libqhullcpp/QhullQh.h"
#include "libqhullcpp/QhullFacet.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullLinkedList.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/Qhull.h"

int main()
{ 
  orgQhull::Qhull conv_hull;

  double input[12]={-1,-1,-1,1,0,1.5,1,1,1,-1,0,-1.5};

  conv_hull.runQhull("", 2,6 ,input ,"Qx Qt");

  conv_hull.defineVertexNeighborFacets();
  double area=conv_hull.area();
  double volume=conv_hull.volume();
  double num_vtx=conv_hull.vertexCount();
  double num_facets=conv_hull.facetCount();
 
  facetT* curr_f=conv_hull.beginFacet().getFacetT();
  facetT* end_f=conv_hull.endFacet().getFacetT();
  double r_oc_insphere=-(curr_f->offset);
  orgQhull::QhullFacet curr_Qf=conv_hull.beginFacet();


  //toStdVector() 
  for(uint i=0;i< num_facets;i++)
    {
      curr_f->id=i; //Replaces the Qhull runtime indexing with indices 0 - num_facets_
      r_oc_insphere = (-(curr_f->offset) < r_oc_insphere) ? -(curr_f->offset) : r_oc_insphere;

      setT* vertices=curr_f->vertices;
      orgQhull::QhullVertexSet vertices_Q=curr_Qf.vertices();
      //   std::cout<<std::endl<<"C - style..."<<std::endl;
      for(uint j=0;j<qh_setsize(vertices) ;j++)
	{
	  coordT* p =((vertexT*)vertices->e[j].p)->point;
	  vertexT* v=  (vertexT*)vertices->e[j].p;
	  //	  std::cout<<v->id<<" ";
	}

      //  std::cout<<std::endl<<"CPP - style..."<<std::endl;

      for(uint j=0;j<qh_setsize(vertices) ;j++)
	{
	  orgQhull::QhullVertex v_Q=vertices_Q[j];  
	  std::cout<<v_Q.point().id()<<" ";
	}
      std::cout<<std::endl;
      curr_f=curr_f->next;
      curr_Qf=curr_Qf.next();
    }

  return 0;
}
