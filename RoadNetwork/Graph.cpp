#include "Graph.h"

namespace csci7551_project
{
  Edge* Vertex::connect(Vertex* d)
  {
    Edge* e = new Edge(this,d);
    this->connectOutflow(e);
    d->connectInflow(e);
    return e;
  }

  Edge* Vertex::connect(Vertex* d, Edge* e)
  {
    this->connectOutflow(e);
    d->connectInflow(e);
    return e;
  }

  int Vertex::s_id = 0;
  int Edge::s_id = 0;
}