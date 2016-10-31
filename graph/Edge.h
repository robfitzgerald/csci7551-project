#ifndef csci7551_project_edge
#define csci7551_project_edge

#include <vector>
#include "Property.h"

namespace csci7551_project
{
  class Vertex;

  class Edge
  {
  public:
    Edge(Vertex* s, Vertex* d): src(s), dest(d) 
    {
      assignID();
    }
    Edge(Vertex* s, Vertex* d, double w): src(s), dest(d)
    {
      props = new DefaultProperty(w);
      assignID();
    }
    Edge(Vertex* s, Vertex* d, Property* p): src(s), dest(d), props(p)
    {
      assignID();
    }   
    int getID () const { return id; }
    Vertex* getSource () const { return src; }
    Vertex* getDestination () const { return dest; }
    Property* getProps () const { return props; }
  private:
    Vertex* src;
    Vertex* dest;
    int id;
    Property* props;
    static int s_id;
    void assignID () { id = s_id++; }
  };
}

#endif  