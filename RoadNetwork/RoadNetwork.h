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
  struct Path 
  {
  public:
    Path(Intersection* s, Intersection* e, unsigned f): start(s), end(e), flow(f) {}
    ~Path(){}
    Intersection *start, *end;
    unsigned flow;
    std::vector<Roadway*> route;
  };

  struct ODPair 
  {
    ODPair(Intersection *o, Intersection *d, unsigned f): origin(o), destination(d), flow(f) {}
    ~ODPair(){}
    Intersection *origin, *destination;
    unsigned flow;
  };

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
    Intersection* getIntersection(std::string);
    void runAllShortestPaths(std::vector<ODPair>);
    std::string toString();
  private:
    Path shortestPath (ODPair, double, std::vector<double>&, int, bool);
    bool stoppingTest (double, const std::vector<double>&, int, bool);
    std::map<std::string,Intersection*> V;
    std::vector<Roadway*> E;
    CostFunction* costFunction;
  };

  double euclidianDistance (Intersection*,Intersection*);
  bool isLocalMaster (int);
  void printTree (Intersection*, int);
}

#endif