#ifndef CSCI7551_PROJECT_GRAPH_ROADWAY_H_
#define CSCI7551_PROJECT_GRAPH_ROADWAY_H_

#include <vector>
#include "Vector.h"

namespace csci7551_project
{
  class Roadway : public Edge
  {
  public:
    Edge(Vertex* s, Vertex* d, double w): src(s), dest(d)
    {
      props = new DefaultEdgeProperty(w);
      assignID();
    }
    Edge(Vertex* s, Vertex* d, EdgeProperty* p): src(s), dest(d), props(p)
    {
      assignID();
    }   
    int getID () const { return id; }
    inline Vertex* getSource () const { return src; }
    inline Vertex* getDestination () const { return dest; }
    inline EdgeProperty* getProps () const { return props; }
  private:
    Vertex* src;
    Vertex* dest;
    int id;
    EdgeProperty* props;
    static int s_id;
    inline void assignID () { id = s_id++; }
  };
}

#endif  