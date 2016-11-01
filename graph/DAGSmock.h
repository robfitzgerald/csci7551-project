#ifndef CSCI7551_PROJECT_GRAPH_DAGSMOCK_H_
#define CSCI7551_PROJECT_GRAPH_DAGSMOCK_H_

#include "Vertex.h"

#include <map>
#include <string>

namespace csci7551_project
{
  class DAGSmock
  {
  public:
    DAGSmock () {}
    void addIntersection (int,int,std::string);
    void addRoadway (std::string,std::string,double,double);

  private:
    std::map<std::string,Vertex*> V;
  };

  double cartesianDistance (Vertex*,Vertex*);
}

#endif