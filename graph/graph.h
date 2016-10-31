#ifndef csci7551_project_graph2
#define csci7551_project_graph2

#include <vector>
#include <math.h>

namespace csci7551_project
{
  class Property 
  {
  public:
    virtual double cost() = 0;
    virtual double cost(double) = 0;
  };

  class TestProperty : public Property
  {
  public:
    TestProperty(double w): weight(w) {}
    double cost() { return weight; }
    double cost(double V) { return weight * V; }
  private:
    double weight;
  };

  class P_Smock : public Property
  {
  public:
    P_Smock(double f, double s): freeFlowTravelTime(f), steadyStateCapacity(s) 
    {
      flowVPH = 0;
    }
    void setV(double V) { flowVPH = V; }
    double cost()
    {
      return freeFlowTravelTime * exp(flowVPH / steadyStateCapacity);
    }
    double cost(double V)
    {
      return freeFlowTravelTime * exp(V / steadyStateCapacity);
    }
  private:
    double freeFlowTravelTime, steadyStateCapacity, flowVPH;
  };

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
    void connect(Vertex* d)
    {
      Edge* e = new Edge(this,d);
      this->connectOutflow(e);
      d->connectInflow(e);
    }
    void connect(Vertex* d, double w)
    {
      Edge* e = new Edge(this,d,w);
      this->connectOutflow(e);
      d->connectInflow(e);
    }
    void connect(Vertex* d, Property* w)
    {
      Edge* e = new Edge(this,d,w);
      this->connectOutflow(e);
      d->connectInflow(e);
    }
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

  int Vertex::s_id = 0;
  int Edge::s_id = 0;
}

#endif