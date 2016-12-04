#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <iostream>
#include <cmath>

#include "RoadNetwork.h"
#include "Graph.h"
#include "IntersectionProperty.h"
#include "Intersection.h"
#include "Roadway.h"

namespace csci7551_project
{
  typedef std::map<std::string,Intersection*>::iterator IntersectionIterator; 
  typedef std::vector<Roadway*>::iterator RoadwayIterator;

  void RoadNetwork::addIntersection (double x, double y, std::string n)
  {
    Intersection* v = new Intersection(new IntersectionProperty(x,y,n));
    this->V.insert(std::pair<std::string,Intersection*>(n,v));
  }

  void RoadNetwork::addRoadway (std::string s, std::string d, double t, double c)
  {
    IntersectionIterator sourceIterator = V.find(s), destinationIterator = V.find(d);
    if (sourceIterator == V.end() || destinationIterator == V.end())
    {
      std::cerr << "!unable to add roadway [" << s << " " << d << " " << t << " " << c << "]" << std::endl; 
    } 
    else 
    {
      Intersection* source = sourceIterator->second;
      Intersection* destination = destinationIterator->second;
      Roadway* newEdge = (Roadway*) source->connect(destination, this->costFunction);
      newEdge->setFreeFlowTime(t)->setCapacity(c)->setDistance(cartesianDistance(source, destination));
      E.push_back(newEdge);
    }
  }

  Intersection* RoadNetwork::getIntersection(std::string name)
  {
    IntersectionIterator i = V.find(name);
    if (i == V.end())
      std::cerr << "!unable to find intersection [" << name << "]" << std::endl; 
    else 
      return i->second;
  }

  // invariant: odpairs have been "found" aka for each o/d name we have found it's Intersection*
  void RoadNetwork::runAllShortestPaths (std::vector<ODPair> odPairs)
  {
    int i;
    std::vector<Path> paths;
    std::vector<unsigned> topDistances(odPairs.size() * 2, 0);
    // do i need to share(V,E)? or, since the OD pairs are pointers to intersections, can
    // i find the other intersections they are connected to?
    #pragma omp parallel shared(odPairs,topDistances) private(i)
    {
      #pragma omp for schedule(static)
      for (i = 0; i < odPairs.size() * 2; ++i)
      {
        Path result = shortestPath(odPairs[i/2], topDistances, i);
        if (isLocalMaster(i))
        {
          #pragma omp critical
          paths.push_back(result);
        }
      }
    }

    for (int j = 0; j < paths.size(); ++j)
    {
      std::string startName = paths[j].start->getIntersectionProperties()->getName();
      std::string endName = paths[j].end->getIntersectionProperties()->getName();
      std::cout << "start: " << startName << ", end: " << endName << ", flow: " << paths[j].flow << ", top values: " << topDistances[j*2] << ", " << topDistances[(j*2)+1] << std::endl;
    }
  }

  Path RoadNetwork::shortestPath (ODPair od, std::vector<unsigned> &top, int jobID)
  {
    Path shortestPath(od.origin, od.destination, od.flow);
    bool stoppingConditionNotMet = true;
    if (isLocalMaster(jobID))
    {
      // do forward search
      shortestPath.flow += 1000;
      top[jobID] = shortestPath.flow;
    } else 
    {
      // do backward search
      shortestPath.flow += 0; 
      top[jobID] = shortestPath.flow;
    }
    return shortestPath;
  }

  std::string RoadNetwork::toString ()
  {
    std::stringstream output;
    output << "----- Graph State -----\n";
    output << "-= Vertices =----------\n";
    for (IntersectionIterator i = V.begin(); i != V.end(); i++)
    {
      Intersection* v = i->second;
      IntersectionProperty* vProps = (IntersectionProperty*) v->getProps();
      output << "[" << i->first << "] (" << vProps->getX() << "," << vProps->getY() << "): " <<  v->getInflows().size() << " in, " << v->getOutflows().size() << " out." << std::endl;
    }
    output << "-= Edges =-------------\n";
    for (RoadwayIterator i = E.begin(); i != E.end(); i++)
    {
      Roadway* e = *i;
      IntersectionProperty* sProps = (IntersectionProperty*) e->getSource()->getProps();
      IntersectionProperty* dProps = (IntersectionProperty*) e->getDestination()->getProps();
      output << "(" << sProps->getName() << ")-->(" << dProps->getName() << ") distance: " << e->weight() << ", cost (V=200): " << e->setFlow(200)->cost() << std::endl;
    }
    return output.str();
  }

  double cartesianDistance (Intersection* s, Intersection* d)
  {
    IntersectionProperty* source = (IntersectionProperty*) s->getProps();
    IntersectionProperty* destination = (IntersectionProperty*) d->getProps();
    double x = destination->getX() - source->getX();
    double y = destination->getY() - source->getY();
    return sqrt(x*x + y*y);
  }

  // even numbered pid's will be master to their odd-numbered counterparts pid+1
  bool isLocalMaster (int pid)
  {
    return (pid % 2) == 0;
  }
}