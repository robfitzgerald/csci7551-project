#ifndef CSCI7551_PROJECT_GRAPH_EDGEPROPERTY_H_
#define CSCI7551_PROJECT_GRAPH_EDGEPROPERTY_H_

#include <cmath>

// @TODO: calculate and store the angle of the roadway for GUI output

namespace csci7551_project
{
  class EdgeProperty 
  {
  public:
    virtual double cost() = 0;
    virtual double cost(double) = 0;
    virtual double weight() const = 0;
  };

  class DefaultEdgeProperty : public EdgeProperty
  {
  public:
    DefaultEdgeProperty(double w): edgeWeight(w) {}
    inline double cost() { return edgeWeight; }
    inline double cost(double V) { return edgeWeight * V; }
    inline double weight() const { return edgeWeight; }
  private:
    double edgeWeight;
  };

  class P_Smock : public EdgeProperty
  {
  public:
    P_Smock(double w, double t, double c): edgeWeight(w), freeFlowTravelTime(t), steadyStateCapacity(c) 
    {
      flowVPH = 0; 
    }
    inline void setV(double V) { flowVPH = V; }
    inline double cost()
    {
      return freeFlowTravelTime * exp(flowVPH / steadyStateCapacity);
    }
    inline double cost(double V)
    {
      return freeFlowTravelTime * exp(V / steadyStateCapacity);
    }
    inline double weight() const { return edgeWeight; }
    inline double distance() const { return edgeWeight; }
  private:
    double freeFlowTravelTime, steadyStateCapacity, flowVPH, edgeWeight;
  };  
}

#endif