#include "DAGSmock.h"

#include <string>
#include <unordered_map>
#include <iostream>
#include <cmath>

namespace csci7551_project
{
  typedef std::unordered_map<std::string,Vertex*>::const_iterator VerticesIterator; 

  void DAGSmock::addIntersection (int x, int y, string n)
  {
    Vertex* v = new Vertex(new RoadIntersection(x,y,n));
    this->V.emplace(n,v);
  }
  void DAGSmock::addRoadway (string s, string d, double t, double c)
  {
    VerticesIterator sourceIterator = V.find(s), destinationIterator = V.find(d);
    if (sourceIterator == V.end() || destinationIterator == V.end())
    {
      cerr << "!unable to add roadway [" << s << " " << d << " " << t << " " << c << "]" << std::endl; 
    } 
    else 
    {
      Vertex* source = sourceIterator->second, destination = destinationIterator->second;
      source->connect(destination, new P_Smock(cartesianDistance(source,destination),t,c));
    }
  }

  double cartesianDistance (Vertex* s, Vertex* d)
  {
    double x = d->getProps()->x - s->getProps()->x;
    double y = d->getProps()->y - s->getProps()->y;
    return sqrt(x*x + y*y);
  }
}