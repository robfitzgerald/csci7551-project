#ifndef CSCI7551_PROJECT_ROADNETWORK_H_
#define CSCI7551_PROJECT_ROADNETWORK_H_

#include <map>
#include <vector>
#include <string>

#include "CostFunction.h"
#include "Intersection.h"
#include "Roadway.h"

namespace csci7551_project
{
  class RoadNetwork
  {
  public:
    RoadNetwork (COST_FUNCTION c, double alpha, double beta) 
    {
      CostFunctionFactory costFunctionFactory(alpha, beta);
      this->costFunction = costFunctionFactory(c);
    }
    ~RoadNetwork()
    {
      V.erase(V.begin(), V.end());
      E.erase(E.begin(), E.end());
      delete costFunction;
    }    
    void addIntersection (double,double,std::string);
    void addRoadway (std::string,std::string,double,double);
    std::string toString();
  private:
    std::map<std::string,Intersection*> V;
    std::vector<Roadway*> E;
    CostFunction* costFunction;
  };

  double cartesianDistance (Intersection*,Intersection*);
}

#endif