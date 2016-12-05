#ifndef _CSCI7551_PROJECT_INTERSECTION_H_
#define _CSCI7551_PROJECT_INTERSECTION_H_

#include "Graph.h"
#include "IntersectionProperty.h"
#include "Roadway.h"
#include "CostFunction.h"

namespace csci7551_project
{
  class Roadway;
  class Intersection : public Vertex
  {
  public:
    Intersection(IntersectionProperty* v): Vertex(v) {}
    Roadway* connect(Intersection*, CostFunction*);
    inline IntersectionProperty* getIntersectionProperties()
    {
      return (IntersectionProperty*) this->getProps();
    }
    std::vector<Roadway*> getInRoads();
    std::vector<Roadway*> getOutRoads();
    std::vector<std::pair<Intersection*, double> > getNeighbors();
  };
}

#endif