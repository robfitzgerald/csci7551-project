#ifndef CSCI7551_PROJECT_GRAPH_DAGSMOCK_H_
#define CSCI7551_PROJECT_GRAPH_DAGSMOCK_H_

#include "Vertex.h"
#include "Edge.h"

#include <map>
#include <vector>
#include <string>

namespace csci7551_project
{
  class DAGSmock
  {
  public:
    DAGSmock () {}
    void addIntersection (int,int,std::string);
    void addRoadway (std::string,std::string,double,double);
    std::string toString();
  private:
    std::map<std::string,Vertex*> V;
    std::vector<Edge*> E;
  };

  double cartesianDistance (Vertex*,Vertex*);
}

#endif