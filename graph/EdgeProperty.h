#ifndef CSCI7551_PROJECT_GRAPH_EDGEPROPERTY_H_
#define CSCI7551_PROJECT_GRAPH_EDGEPROPERTY_H_

#include <cmath>
#include <iostream>

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
      // std::cout << "New EdgeProperty\n" 
      //      << "Distance: " << edgeWeight << "\n"
      //      << "Free flow travel time: " << freeFlowTravelTime << "\n"
      //      << "Steady state capacity: " << steadyStateCapacity << "\n"
      //      << "-----\n";
    }
    inline double setV(double V) { flowVPH = V; return flowVPH; }
    inline double incV() { flowVPH += 1; return flowVPH; }
    inline double incV(double v) { flowVPH += v; return flowVPH; }
    inline double getV() { return flowVPH; }
    inline double cost()
    {
      return freeFlowTravelTime * exp(flowVPH / steadyStateCapacity);
    }
    inline double cost(double V)
    {
      return freeFlowTravelTime * exp((V + flowVPH) / steadyStateCapacity);
    }
    inline double weight() const { return edgeWeight; }
    inline double distance() const { return edgeWeight; }
  private:
    double freeFlowTravelTime, steadyStateCapacity, flowVPH, edgeWeight;
  };  
}

#endif