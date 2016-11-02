#ifndef CSCI7551_PROJECT_GRAPH_ROADWAY_H_
#define CSCI7551_PROJECT_GRAPH_ROADWAY_H_

#include "Vertex.h"
#include "Edge2.h"
#include "CostFunction.h"

namespace csci7551_project
{
  class Roadway : public Edge
  {
  public:
    Roadway(Vertex* s, Vertex* d, CostFunction* c):  
      Edge(s,d),
      costFunction(c) {}
    inline double getDistance() { return distance; }
    inline double getFlow() { return edgeFlow; }
    inline double getFreeFlowTime() { return freeFlowTravelTime; }
    inline double getCapacity() { return steadyStateCapacity; }

    inline Roadway* setDistance(double d) { distance = d; return this; }
    inline Roadway* setFlow(double flow) { edgeFlow = flow; return this; }
    inline Roadway* incFlow() { edgeFlow += 1; return this; }
    inline Roadway* incFlow(double f) { edgeFlow += f; return this; }
    inline Roadway* setFreeFlowTime(double time) { freeFlowTravelTime = time; return this; }
    inline Roadway* setCapacity(double cap) { steadyStateCapacity = cap; return this; }
    
    inline double cost()
    { 
      return costFunction->cost(edgeFlow, freeFlowTravelTime, steadyStateCapacity); 
    } 
    inline double weight() { return distance; }
  private:
    double distance, edgeFlow, freeFlowTravelTime, steadyStateCapacity;
    CostFunction* costFunction;
  };
}

#endif  