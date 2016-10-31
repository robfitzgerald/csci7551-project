#include "graph.h"

namespace csci7551_project
{
  void Vertex::connect(Vertex* d)
  {
    Edge* e = new Edge(this,d);
    this->connectOutflow(e);
    d->connectInflow(e);
  }

  void Vertex::connect(Vertex* d, double w)
  {
    Edge* e = new Edge(this,d,w);
    this->connectOutflow(e);
    d->connectInflow(e);
  }

  void Vertex::connect(Vertex* d, Property* w)
  {
    Edge* e = new Edge(this,d,w);
    this->connectOutflow(e);
    d->connectInflow(e);
  }
  
  int Vertex::s_id = 0;
  int Edge::s_id = 0;
}
