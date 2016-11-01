#include "Vertex.h"

namespace csci7551_project
{
  Vertex* Vertex::connect(Vertex* d)
  {
    Edge* e = new Edge(this,d);
    this->connectOutflow(e);
    d->connectInflow(e);
    return e;
  }

  Vertex* Vertex::connect(Vertex* d, double w)
  {
    Edge* e = new Edge(this,d,w);
    this->connectOutflow(e);
    d->connectInflow(e);
    return e;
  }

  Vertex* Vertex::connect(Vertex* d, EdgeProperty* w)
  {
    Edge* e = new Edge(this,d,w);
    this->connectOutflow(e);
    d->connectInflow(e);
    return e;
  }
  int Edge::s_id = 0;  // @TODO: should be in a separate Edge.cpp
  int Vertex::s_id = 0;
}
