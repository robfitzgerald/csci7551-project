#ifndef _CSCI7551_PROJECT_INTERSECTION_H_
#define _CSCI7551_PROJECT_INTERSECTION_H_

#include "Graph.h"
#include "IntersectionProperty.h"
#include "Roadway.h"
#include "CostFunction.h"

namespace csci7551_project
{
  class Intersection : public Vertex
  {
  public:
    Intersection(IntersectionProperty* v): Vertex(v) {}
    Roadway* connect(Intersection*, CostFunction*);
  };
}

#endif