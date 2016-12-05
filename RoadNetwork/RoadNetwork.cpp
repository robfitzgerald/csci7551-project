#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <iostream>
#include <cmath>

// #include "omp.h"

#include "RoadNetwork.h"
#include "Graph.h"
#include "IntersectionProperty.h"
#include "Intersection.h"
#include "Roadway.h"
#include "ShortestPathTree.h"

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
      newEdge->setFreeFlowTime(t)->setCapacity(c)->setDistance(euclidianDistance(source, destination));
      E.push_back(newEdge);
    }
  }

  Intersection* RoadNetwork::getIntersection(std::string name)
  {
    IntersectionIterator i = V.find(name);
    if (i == V.end()) 
    {
      std::cerr << "!unable to find intersection [" << name << "]" << std::endl; 
      return 0;
    }
    else 
      return i->second;
  }

  // invariant: odpairs have been "found" aka for each o/d name we have found it's Intersection*
  void RoadNetwork::runAllShortestPaths (std::vector<ODPair> odPairs)
  {
    int i;
    bool stoppingConditionNotMet = true;
    std::vector<Path> paths;
    std::vector<double> topDistances(odPairs.size() * 2, 0);
    // std::vector<ShortestPathTree> trees(odPairs.size() * 2);
    #pragma omp parallel shared(odPairs,topDistances) firstprivate(stoppingConditionNotMet) private(i)
    {
      #pragma omp for schedule(static)
      for (i = 0; i < odPairs.size() * 2; ++i)
      {
        Intersection* thisSource = odPairs[i/2].origin;
        Intersection* thisDestination = odPairs[i/2].destination;
        double distance = euclidianDistance(thisSource,thisDestination);
        Path result = shortestPath(odPairs[i/2], distance, topDistances, i, stoppingConditionNotMet);
        if (isLocalMaster(i))
        {
          #pragma omp critical
          paths.push_back(result);
        }
      }
    }

    // for (int j = 0; j < paths.size(); ++j)
    // {
    //   std::string startName = paths[j].start->getIntersectionProperties()->getName();
    //   std::string endName = paths[j].end->getIntersectionProperties()->getName();
    //   std::cout << "start: " << startName << ", end: " << endName << ", flow: " << paths[j].flow << ", top values: " << topDistances[j*2] << ", " << topDistances[(j*2)+1] << std::endl;
    // }
  }

  Path RoadNetwork::shortestPath (ODPair od, double dist, std::vector<double> &top, int jobID, bool stoppingConditionNotMet)
  {
    Path result(od.origin, od.destination, od.flow);
    while (stoppingConditionNotMet)
    {
      if (isLocalMaster(jobID))
      {
        // do forward search
        result.flow += 1000;
        printTree(od.origin,0);
        top[jobID] = result.flow;
      } else 
      {
        // do backward search
        result.flow += 0; 
        top[jobID] = result.flow;
      }
      bool unexplored = true;
      stoppingConditionNotMet = false;
      #pragma omp critical
      if (stoppingTest(dist, top, jobID, unexplored))
      {
        // wrap it up buddy. merge, put a bow on it.
          
        stoppingConditionNotMet = false;
      }
    }
    
    return result;
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

  double euclidianDistance (Intersection* s, Intersection* d)
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

  // should not 
  bool RoadNetwork::stoppingTest (double dist, const std::vector<double> &top, int jobID, bool unexploredIntersections)
  {
    if (unexploredIntersections == false)
      return true;
    int otherID;
    if (isLocalMaster(jobID))
      otherID = jobID + 1;
    else
      otherID = jobID - 1;
    return (top[jobID] + top[otherID]) > dist;
  }

  void printTree (Intersection* s, int depth)
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
      printTree(dest, depth+1);
    }
  }

}