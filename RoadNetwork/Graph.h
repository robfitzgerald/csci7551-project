#ifndef _CSCI7551_PROJECT_GRAPH_H_
#define _CSCI7551_PROJECT_GRAPH_H_

#include <vector>

#include "CostFunction.h"

namespace csci7551_project
{
  class Vertex;

  class VertexProperty
  {
  public:
    VertexProperty () { id = -1; }
    VertexProperty (int p): id(p) {}
  private:
    int id;
  };

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

  class Vertex
  {
  public: 
    Vertex() 
    { 
      int myId = assignID();
      props = new VertexProperty(myId);
    }
    Vertex(VertexProperty* v): props(v)
    {
      int myId = assignID();
    }
    ~Vertex()
    {
      delete props;
    }
    inline int getID () const { return id; }
    inline VertexProperty* getProps() { return props; }
    Edge* connect(Vertex*);
    Edge* connect(Vertex*, Edge*);
    std::vector<Edge*> getInflows() { return in; }
    std::vector<Edge*> getOutflows() { return out; }
  private:
    VertexProperty* props;
    std::vector<Edge*> in, out;
    int id;
    static int s_id;

    inline void connectInflow (Edge* e) { in.push_back(e); }
    inline void connectOutflow (Edge* e) { out.push_back(e); }
    inline int assignID () { id = s_id++; return id; }
  };
}

#endif