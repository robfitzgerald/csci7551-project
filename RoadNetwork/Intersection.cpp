#include <vector>
#include <utility>

#include "Graph.h"
#include "Intersection.h"
#include "Roadway.h"
#include "CostFunction.h"

namespace csci7551_project
{
  Roadway* Intersection::connect (Intersection* d, CostFunction* c)
  {
    // Edge* e = new Roadway(this,d,c);
    // this->connectOutflow(e);
    // d->connectInflow(e);
    Roadway* e = new Roadway(this,d,c);
    return (Roadway*) Vertex::connect(d, e);
    // return e;
  }

  std::vector<Roadway*> Intersection::getInRoads()
  { 
    std::vector<Edge*> flows = this->getInflows();
    std::vector<Roadway*> result;
    for (int i = 0; i < flows.size(); ++i)
      result.push_back((Roadway*) flows[i]);
    return result;
  }
  std::vector<Roadway*> Intersection::getOutRoads()
  { 
    std::vector<Edge*> flows = this->getOutflows();
    std::vector<Roadway*> result;
    for (int i = 0; i < flows.size(); ++i)
      result.push_back((Roadway*) flows[i]);
    return result;
  }  

  std::vector<std::pair<Intersection*, double> > Intersection::getForwardNeighbors()
  {
    std::vector<Edge*> flows = this->getOutflows();
    std::vector<std::pair<Intersection*, double> > result;
    for (int i = 0; i < flows.size(); ++i)
    {
      Intersection* d = ((Roadway*) flows[i])->getDestinationIntersection();
      double c = ((Roadway*) flows[i])->cost();
      std::pair<Intersection*, double> neighbor(d,c);
      result.push_back(neighbor);
    } 
    return result;
  }

  std::vector<std::pair<Intersection*, double> > Intersection::getReverseNeighbors()
  {
    std::vector<Edge*> flows = this->getInflows();
    std::vector<std::pair<Intersection*, double> > result;
    for (int i = 0; i < flows.size(); ++i)
    {
      Intersection* d = ((Roadway*) flows[i])->getSourceIntersection();
      double c = ((Roadway*) flows[i])->cost();
      std::pair<Intersection*, double> neighbor(d,c);
      result.push_back(neighbor);
    } 
    return result;
  }  
}


