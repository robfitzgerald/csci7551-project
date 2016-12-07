#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <list>
#include <iostream>
#include <cmath>

// #include "omp.h"

#include "RoadNetwork.h"
#include "Graph.h"
#include "IntersectionProperty.h"
#include "Intersection.h"
#include "Roadway.h"
#include "BidirectionalAStar.h"

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

  void RoadNetwork::assignMSA(std::vector<ODPair>* odPairs)
  {
    resetFlows();
    std::vector<double> flows (E.size(), 0);
    std::vector<Roadway*> populatedRoadways;
    calculateNetworkFlows(flows);  // step 0

    for (int i = 1; i < simulationIterations; ++i)
    {
      double phi = 1.0 / i;
      std::vector<Path*>* paths = new std::vector<Path*>();
      runAllShortestPaths(odPairs, paths);  // omp enabled
      loadAllOrNothing(paths);
      std::vector<double> allOrNothingFlows, allOrNothingCosts;
      calculateNetworkFlows(allOrNothingFlows);
      calculateNetworkFlowCosts(allOrNothingCosts);
      // @TODO: 
      // #pragma omp parallel shared() private()
      // {
      // #pragma for schedule(static)
      for (int j = 0; j < E.size(); ++i)
      {
        flows[j] = calculateCurrentFlow(phi, E[j]->getFlow(), flows[j]);
      }
      // }
    }
  }

  // @TODO: figure out objective function Z ..
  void RoadNetwork::assignFrankWolfe (std::vector<ODPair>* odPairs) {}

  double RoadNetwork::calculateCurrentFlow (double phi, double AONflow, double previousFlow)
  {
    return ((1 - phi) * previousFlow) + (phi * AONflow);
  }

  void RoadNetwork::resetFlows ()
  {
    for (std::vector<Roadway*>::iterator i = E.begin(); i != E.end(); ++i)
    {
      (*i)->setFlow(0);
    }
  }

  void RoadNetwork::calculateNetworkFlows (std::vector<double>& flows)
  {
    for (int i = 0; i < E.size(); ++i)
    {
      flows[i] = E[i]->getFlow();
    }
  }

  void RoadNetwork::calculateNetworkFlowCosts (std::vector<double>& costs)
  {
    for (int i = 0; i < E.size(); ++i)
    {
      costs[i] = E[i]->cost();
    }
  }

  void RoadNetwork::loadAllOrNothing (std::vector<Path*>* paths)
  {
    for (std::vector<Path*>::iterator i = paths->begin(); i != paths->end(); ++i)
    {
      for (std::list<Roadway*>::iterator j = (*i)->route.begin(); j != (*i)->route.end(); ++j)
      {
        (*j)->incFlow((*i)->flow);
      }
    }
  }

  double RoadNetwork::relGapConvergenceTest (std::vector<double> currentFlows, std::vector<double> aonFlows, std::vector<double> costs)
  {
    double currentCostFlows, aonCostFlows;
    for (int i = 0; i < currentFlows.size(); ++i)
    {
      currentCostFlows += currentFlows[i] * costs[i];
      aonCostFlows += aonFlows[i] * costs[i];
    }
    return ((currentCostFlows - aonCostFlows) / currentCostFlows);
  }

  // invariant: odpairs have been "found" aka for each o/d name we have found it's Intersection*
  void RoadNetwork::runAllShortestPaths (std::vector<ODPair>* odPairs, std::vector<Path*>* paths)
  {
    int i;
    std::vector<bool> stoppingConditionNotMet(odPairs->size() * 2, true);
    std::vector<std::list<Intersection*> > intersections(odPairs->size() * 2);
    std::vector<std::list<std::pair<double,double> > > coordinates(odPairs->size() * 2);
    std::vector<std::list<double> > distances(odPairs->size() * 2);
    std::vector<BidirectionalAStar*> search(odPairs->size() * 2, 0);
    #pragma omp parallel shared(odPairs,intersections,coordinates,distances,search) firstprivate(stoppingConditionNotMet) private(i)
    {
      #pragma omp for schedule(static)
      for (i = 0; i < odPairs->size() * 2; ++i)
      {
        Intersection* thisSource = (*odPairs)[i/2].origin;
        Intersection* thisDestination = (*odPairs)[i/2].destination;
        if ((i % 2) == 0)
          search[i] = new BidirectionalAStar(thisSource, FORWARD);
        else
          search[i] = new BidirectionalAStar(thisDestination, BACKWARD);
        double distance = euclidianDistance(thisSource,thisDestination);
        Path* result = shortestPath((*odPairs)[i/2], distance, intersections, coordinates, distances, search, i, stoppingConditionNotMet);
        #pragma omp critical
        if (isLocalMaster(i))
        {
          paths->push_back(result);
        }
        search.erase(search.begin() + i);
      }
    }

    // for (int j = 0; j < paths.size(); ++j)
    // {
    //   std::string startName = paths[j].start->getIntersectionProperties()->getName();
    //   std::string endName = paths[j].end->getIntersectionProperties()->getName();
    //   std::cout << "start: " << startName << ", end: " << endName << ", flow: " << paths[j].flow << ", top values: " << topDistances[j*2] << ", " << topDistances[(j*2)+1] << std::endl;
    // }
  }

  Path* RoadNetwork::shortestPath (ODPair od, double dist, std::vector<std::list<Intersection*> > &intersections, std::vector<std::list<std::pair<double,double> > > coordinates, std::vector<std::list<double> > distances, std::vector<BidirectionalAStar*> search, int jobID, std::vector<bool> stoppingConditionNotMet)
  {
    Path* result = new Path(od.origin, od.destination, od.flow);
    while (stoppingConditionNotMet[jobID])
    {
      search[jobID]->updateFrontier();
      search[jobID]->loadCompareList(intersections[jobID],coordinates[jobID],distances[jobID]);

      #pragma omp critical
      if (isLocalMaster(jobID))
      {
        // 
        if (stoppingTest(intersections[jobID],intersections[jobID+1]))
        {
          // if stoppingTest was true, it also had the side effect of removing all
          // intersections except the ones that match (since finding those is part
          // of how the stopping test occurs, that O(n^2) operation should only
          // happen once)
          stoppingConditionNotMet[jobID] = false;
          stoppingConditionNotMet[jobID+1] = false;
        }
      }
      #pragma omp critical
      if (stoppingConditionNotMet[jobID])
      {
        // continue search [master,slave]
        compareLists(intersections[jobID],coordinates[jobID],distances[jobID],intersections[jobID+1],coordinates[jobID+1],distances[jobID+1]);
        search[jobID]->moveToSelected(intersections[jobID].front());
        intersections.clear();
      }
      else if (isLocalMaster(jobID))
      {
        // end search [master]
        // merge paths found
        // intersections has our intersection where they met
        // attach the two paths from search
        result->route = search[jobID]->mergeBidirectionalPaths(search[jobID+1], intersections[jobID].front());
        clearLists(intersections[jobID],coordinates[jobID],distances[jobID],intersections[jobID+1],coordinates[jobID+1],distances[jobID+1]);
      }
      else
      {
        // end search [slave]
        // no action here
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
      output << "(" << sProps->getName() << ")-->(" << dProps->getName() << ") distance: " << e->weight() << ", cost: " << e->cost() << std::endl;
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

  bool RoadNetwork::stoppingTest (std::list<Intersection*>& a, std::list<Intersection*>& b)
  {
    bool matchFound = false;
    Intersection *aMatch, *bMatch;
    for (std::list<Intersection*>::iterator i = a.begin(); i != a.end(); ++i)
    {
      for (std::list<Intersection*>::iterator j = b.begin(); j != b.end(); ++j)
      {
        if ((*i)==(*j))
        {
          aMatch = (*i);
          bMatch = (*j);
          matchFound = true;
        }
      }
    }
    if (matchFound)
    {
      a.clear();
      b.clear();
      a.push_back(aMatch);
      b.push_back(bMatch);
      return true;
    }
    else
      return false;
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

  A_STAR_DIRECTION pickSearchDirection (int id)
  {
    return ((id % 2) == 0) ? FORWARD : BACKWARD;
  }

}