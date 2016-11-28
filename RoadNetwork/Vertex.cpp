#include "Vertex.h"
#include "Edge-Roadway.h"
#include "CostFunction.h"

namespace csci7551_project
{
  Edge* Vertex::connect(Vertex* d, CostFunction* c)
  {
    Edge* e = new Roadway(this,d,c);
    this->connectOutflow(e);
    d->connectInflow(e);
    return e;
  }

  // Edge* Vertex::connect(Vertex* d)
  // {
  //   Edge* e = new Roadway(this,d);
  //   this->connectOutflow(e);
  //   d->connectInflow(e);
  //   return e;
  // }

  // Edge* Vertex::connect(Vertex* d, double w)
  // {
  //   Edge* e = new Roadway(this,d,w);
  //   this->connectOutflow(e);
  //   d->connectInflow(e);
  //   return e;
  // }

  // Roadway* Vertex::connect(Vertex* d, EdgeProperty* w)
  // {
  //   Edge* e = new Roadway(this,d,w);
  //   this->connectOutflow(e);
  //   d->connectInflow(e);
  //   return e;
  // }
  // int Edge::s_id = 0;  // @TODO: should be in a separate Edge.cpp
  int Vertex::s_id = 0;
}
