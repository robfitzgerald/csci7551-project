#ifndef csci7551_project_vertex
#define csci7551_project_vertex

#include <vector>
#include "Edge.h"

namespace csci7551_project
{
  class Vertex
  {
  public: 
    Vertex()
    {
      assignID();
    }
    int getID () const { return id; }
    void connect(Vertex*);
    void connect(Vertex*, double);
    void connect(Vertex*, Property*);
    std::vector<Edge*> getInflows() { return in; }
    std::vector<Edge*> getOutflows() { return out; }
  private:
    std::vector<Edge*> in, out;
    int id;
    static int s_id;
    void assignID () { id = s_id++; }
    void connectInflow (Edge* e) { in.push_back(e); }
    void connectOutflow (Edge* e) { out.push_back(e); }
  };
}

#endif