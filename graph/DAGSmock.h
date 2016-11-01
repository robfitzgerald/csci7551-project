#ifndef CSCI7551_PROJECT_GRAPH_DAGSMOCK_H_
#define CSCI7551_PROJECT_GRAPH_DAGSMOCK_H_

#include "Vertex.h"

#include <unordered_map>
#include <string>

namespace csci7551_project
{
  class DAGSmock
  {
  public:
    DAG_P_Smock () {}
    void addIntersection (int,int,string);
    void addRoadway (string,string,double,double);

  private:
    std::unordered_map<string,Vertex*> V;
  };
}

#endif