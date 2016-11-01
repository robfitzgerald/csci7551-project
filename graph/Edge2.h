#ifndef CSCI7551_PROJECT_GRAPH_EDGE_H_
#define CSCI7551_PROJECT_GRAPH_EDGE_H_

// should be included with Vector.h

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
    inline int getID () const { return id; }
    inline Vertex* getSource () const { return src; }
    inline Vertex* getDestination () const { return dest; }
  private:
    Vertex* src;
    Vertex* dest;
    int id;
    static int s_id;
    inline void assignID () { id = s_id++; }
  };
}

#endif