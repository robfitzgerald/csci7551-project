#include <vector>
#include <utility>
#include <iostream>

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

  std::vector<NodeCostTuple> Intersection::getForwardNeighbors()
  {
    std::vector<Edge*> flows = this->getOutflows();
    std::vector<NodeCostTuple> result;
    for (int i = 0; i < flows.size(); ++i)
    {
      Intersection* dest = ((Roadway*) flows[i])->getDestinationIntersection();
      double c = ((Roadway*) flows[i])->cost();
      double d = ((Roadway*) flows[i])->getDistance();
      NodeCostTuple neighbor(dest,d,c);
      result.push_back(neighbor);
    } 
    return result;
  }

  std::vector<NodeCostTuple> Intersection::getReverseNeighbors()
  {
    std::vector<Edge*> flows = this->getInflows();
    std::vector<NodeCostTuple> result;
    for (int i = 0; i < flows.size(); ++i)
    {
      Intersection* dest = ((Roadway*) flows[i])->getSourceIntersection();
      double c = ((Roadway*) flows[i])->cost();
      double d = ((Roadway*) flows[i])->getDistance();
      NodeCostTuple neighbor(dest,d,c);
      result.push_back(neighbor);
    } 
    return result;
  } 

  double euclidianDistance (Intersection* s, Intersection* d)
  {
    IntersectionProperty* source = (IntersectionProperty*) s->getProps();
    IntersectionProperty* destination = (IntersectionProperty*) d->getProps();
    double x = destination->getX() - source->getX();
    double y = destination->getY() - source->getY();
    return sqrt(x*x + y*y);
  }  

  void Intersection::printTree()
  {
    printOutTree(this, 0);
  }

  void Intersection::printOutTree (Intersection* s, int depth)
  {
    IntersectionProperty* prop = s->getIntersectionProperties();
    std::string output = "";
    for (int j = 0; j < depth; ++j)
        output += " ";
    output += prop->getName();
    std::cout << output << std::endl;

    std::vector<Roadway*> outflows = s->getOutRoads();
    for (int i = 0; i < outflows.size(); ++i)
    {
      Intersection* dest = outflows[i]->getDestinationIntersection();
      printOutTree(dest, depth+1);
    }
  }

}


