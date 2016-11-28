#ifndef CSCI7551_PROJECT_GRAPH_VERTEX_H_
#define CSCI7551_PROJECT_GRAPH_VERTEX_H_

#include <vector>
#include "Edge.h"
#include "VertexProperty.h"
#include "CostFunction.h"

namespace csci7551_project
{
  class Vertex
  {
  public: 
    Vertex() { assignID(); }
    Vertex(VertexProperty* V): props(V) {}
    inline int getID () const { return id; }
    inline VertexProperty* getProps() { return props; }
    Edge* connect(Vertex*,CostFunction*);
    // Edge* connect(Vertex*);
    // Edge* connect(Vertex*, double);
    // Edge* connect(Vertex*, EdgeProperty*);
    std::vector<Edge*> getInflows() { return in; }
    std::vector<Edge*> getOutflows() { return out; }
  private:
    VertexProperty* props;
    std::vector<Edge*> in, out;
    inline void connectInflow (Edge* e) { in.push_back(e); }
    inline void connectOutflow (Edge* e) { out.push_back(e); }
    int id;
    static int s_id;
    inline void assignID () { id = s_id++; }
  };
}

#endif