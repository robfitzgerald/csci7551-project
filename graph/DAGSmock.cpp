#include "DAGSmock.h"
#include "Vertex.h"

#include <string>
#include <map>
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
      source->connect(destination, new P_Smock(cartesianDistance(source,destination),t,c));
    }
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