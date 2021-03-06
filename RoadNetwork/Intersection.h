#ifndef _CSCI7551_PROJECT_INTERSECTION_H_
#define _CSCI7551_PROJECT_INTERSECTION_H_

#include "Graph.h"
#include "IntersectionProperty.h"
#include "Roadway.h"
#include "CostFunction.h"

namespace csci7551_project
{
  class Roadway;

  struct NodeCostTuple {
    NodeCostTuple(Intersection* a, Roadway* r, double d, double c): 
      node(a),
      distance(d),
      cost(c),
      road(r) {}
    Intersection* node;
    Roadway* road;
    double distance;
    double cost;
  };

  class Intersection : public Vertex
  {
  public:
    Intersection(IntersectionProperty* v): Vertex(v) {}
    Roadway* connect(Intersection*, CostFunction*);
    inline IntersectionProperty* getIntersectionProperties() const
    {
      return (IntersectionProperty*) this->getProps();
    }
    std::vector<Roadway*> getInRoads();
    std::vector<Roadway*> getOutRoads();
    std::vector<NodeCostTuple> getForwardNeighbors();
    std::vector<NodeCostTuple> getReverseNeighbors();
    inline std::string getName() const
    {
      IntersectionProperty* temp = this->getIntersectionProperties();
      return temp->getName();
    }
    void printTree();
    void printOutTree(Intersection*, int);
  };

  inline bool operator < (const Intersection& lhs, const Intersection& rhs)
  {
    return lhs.getName() < rhs.getName();
  }

  double euclidianDistance (Intersection*,Intersection*);
}

#endif