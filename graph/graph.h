#ifndef csci7551_project_graph
#define csci7551_project_graph

#include <vector>
#include <math.h>
#include "Properties.h"

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
    Edge(Vertex* s, Vertex* d, double w): src(s), dest(d), weight(w)
    {
      assignID();
    }
    Edge(Vertex* s, Vertex* d, Property* p): src(s), dest(d), props(p)
    {
      assignID();
    }   
    int getID () const { return id; }
    Vertex* getSource () const { return src; }
    Vertex* getDestination () const { return dest; }
    double getWeight () const { return weight; }
    Property* getProps () const { return props; }
  private:
    Vertex* src;
    Vertex* dest;
    int id;
    Property* props;
    double weight;
    static int s_id;
    void assignID () { id = s_id++; }
  };

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