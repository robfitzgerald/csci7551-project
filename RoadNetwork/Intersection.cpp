#include "Graph.h"
#include "Intersection.h"
#include "Roadway.h"
#include "CostFunction.h"

namespace csci7551_project
{
  Roadway* Intersection::connect (Intersection* d, CostFunction* c)
  {
    // Edge* e = new Roadway(this,d,c);
    // this->connectOutflow(e);
    // d->connectInflow(e);
    Roadway* e = new Roadway(this,d,c);
    return (Roadway*) Vertex::connect(d, e);
    // return e;
  }
}