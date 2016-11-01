#include "DAGSmock.h"
#include "Vertex.h"

#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <iostream>
#include <cmath>

namespace csci7551_project
{
  typedef std::map<std::string,Vertex*>::iterator VerticesIterator; 

  void DAGSmock::addIntersection (int x, int y, std::string n)
  {
    Vertex* v = new Vertex(new RoadIntersection(x,y,n));
    this->V.insert(std::pair<std::string,Vertex*>(n,v));
  }

  void DAGSmock::addRoadway (std::string s, std::string d, double t, double c)
  {
    VerticesIterator sourceIterator = V.find(s), destinationIterator = V.find(d);
    if (sourceIterator == V.end() || destinationIterator == V.end())
    {
      std::cerr << "!unable to add roadway [" << s << " " << d << " " << t << " " << c << "]" << std::endl; 
    } 
    else 
    {
      Vertex* source = sourceIterator->second;
      Vertex* destination = destinationIterator->second;
      Edge* newEdge = source->connect(destination, new P_Smock(cartesianDistance(source,destination),t,c));
      E.push_back(newEdge);
    }
  }

  std::string DAGSmock::toString()
  {
    std::stringstream output;
    output << "----- Graph State -----\n";
    output << "-= Vertices =----------\n";
    for (VerticesIterator i = V.begin(); i != V.end(); i++)
    {
      Vertex* v = i->second;
      RoadIntersection* vProps = v->getProps();
      output << "[" << i->first << "] (" << vProps->getX() << "," << vProps->getY() << "): " <<  v->getInflows().size() << " in, " << v->getOutflows().size() << " out." << std::endl;
    }
    output << "-= Edges =-------------\n";
    for (std::vector<Edge*>::iterator i = E.begin(); i != E.end(); i++)
    {
      Edge* e = *i;
      RoadIntersection* sProps = e->getSource()->getProps();
      RoadIntersection* dProps = e->getDestination()->getProps();
      EdgeProperty* eProps = e->getProps();
      output << "(" << sProps->getName() << ")-->(" << dProps->getName() << ") distance: " << eProps->weight() << ", cost (V=10): " << eProps->cost(10) << std::endl;
    }
    return output.str();
  }

  double cartesianDistance (Vertex* s, Vertex* d)
  {
    RoadIntersection* source = s->getProps();
    RoadIntersection* destination = d->getProps();
    double x = destination->getX() - source->getX();
    double y = destination->getY() - source->getY();
    return sqrt(x*x + y*y);
  }
}