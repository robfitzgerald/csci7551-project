#ifndef csci7551_project_property
#define csci7551_project_property

#include <math.h>

namespace csci7551_project
{
  class Property 
  {
  public:
    virtual double cost() = 0;
    virtual double cost(double) = 0;
    virtual double weight() const = 0;
  };

  class DefaultProperty : public Property
  {
  public:
    DefaultProperty(double w): edgeWeight(w) {}
    double cost() { return edgeWeight; }
    double cost(double V) { return edgeWeight * V; }
    double weight() const { return edgeWeight; }
  private:
    double edgeWeight;
  };

  class P_Smock : public Property
  {
  public:
    P_Smock(double w, double f, double s): edgeWeight(w), freeFlowTravelTime(f), steadyStateCapacity(s) 
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
    double weight() const { return edgeWeight; }
  private:
    double freeFlowTravelTime, steadyStateCapacity, flowVPH, edgeWeight;
  };  
}

#endif