#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <list>
#include <iostream>
#include <iomanip>
#include <cmath>

#include "omp.h"

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
      newEdge->setFlow(0)->setFreeFlowTime(t)->setCapacity(c)->setDistance(euclidianDistance(source, destination));
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
    std::cout << "starting RoadNetwork::assignMSA" << std::endl;
    resetFlows();
    std::vector<double> flows (E.size(), 0);
    std::vector<Roadway*> populatedRoadways;
    calculateNetworkFlows(flows);  // step 0

    for (int i = 1; i < simulationIterations; ++i)
    {
      double phi = 1.0 / i;
      std::cout << "assignMSA iteration " << i << ", phi " << phi << std::endl;
      std::vector<Path*>* paths = new std::vector<Path*>();
      runAllShortestPaths(odPairs, paths);  // omp enabled
      std::cout << "assignMSA iteration " << i << " runAllShortestPaths done. loading AON." << std::endl;
      loadAllOrNothing(paths);
      std::cout << "assignMSA iteration " << i << " calculating the flows and costs now" << std::endl;
      std::vector<double> allOrNothingFlows, allOrNothingCosts;
      calculateNetworkFlows(allOrNothingFlows);
      calculateNetworkFlowCosts(allOrNothingCosts);
      std::cout << "assignMSA iteration " << i << " calculated. " << std::endl;
      // @TODO: 
      // #pragma omp parallel shared() private()
      // {
      // #pragma for schedule(static)
      for (int j = 0; j < E.size(); ++i)
      {
        flows[j] = calculateCurrentFlow(phi, E[j]->getFlow(), flows[j]);
      }
      // }
      std::cout << "assignMSA iteration " << i << " done." << std::endl;
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
    int i, previousSelectedCount;
    std::vector<bool> buddyWait(odPairs->size() * 2, true);
    std::vector<bool> stoppingConditionNotMet(odPairs->size() * 2, true);
    std::vector<bool> hasFrontier(odPairs->size() * 2, true);
    std::list<Intersection*> temp;
    std::vector<std::list<Intersection*> > intersections(odPairs->size() * 2, temp);
    std::vector<std::list<std::pair<double,double> > > coordinates(odPairs->size() * 2);
    std::vector<std::list<double> > distances(odPairs->size() * 2);
    std::vector<BidirectionalAStar*> search(odPairs->size() * 2, 0);
    Intersection *forwardMeeting = 0, *reverseMeeting = 0;
    bool meetingPointFound = false, searchSpaceExhausted = false;
    Path* result;
    // TODO: cannot share member variables
    std::map<std::string,Intersection*> sharedV = V;
    std::vector<Roadway*> sharedE = E;

    // std::cout << "Sizes of shared memory buckets" << std::endl;
    // std::cout << stoppingConditionNotMet.size() << " " << hasFrontier.size() << " " << intersections.size() << " " << coordinates.size() << " " << distances.size() << " " << search.size() << std::endl;

    #pragma omp parallel shared(sharedV,sharedE,odPairs,paths,intersections,coordinates,distances,search,hasFrontier,stoppingConditionNotMet,buddyWait) private(i,result) firstprivate(forwardMeeting,reverseMeeting,meetingPointFound,searchSpaceExhausted)
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
        // std::cout << "calling shortestPath() for jobID " << i << std::endl;
        shortestPath((*odPairs)[i/2], result, intersections, coordinates, distances, search, i, hasFrontier, stoppingConditionNotMet, buddyWait, forwardMeeting, reverseMeeting, meetingPointFound, searchSpaceExhausted);
        // std::cout << "ended shortestPath() for jobID " << i << std::endl;
        // #pragma omp critical
        
        // buddyWait[jobID] = false;
        // // manual spin-wait for my buddy
        // while (waitForMyBuddy(jobID,buddyWait)) 
        // {
        //   std::cout << ".";
        // }
        // buddyWait[jobID] = true;

        if (isLocalMaster(i))
        {
          std::cout << "returning path" << std::endl;
          paths->push_back(result);
        }
        search.erase(search.begin() + i);
        std::cout << "done with job " << i << std::endl; 
      }
    }
  }

  void RoadNetwork::shortestPath (ODPair& od, Path*& result, std::vector<std::list<Intersection*> >& intersections, std::vector<std::list<std::pair<double,double> > >& coordinates, std::vector<std::list<double> >& distances, std::vector<BidirectionalAStar*>& search, int& jobID, std::vector<bool>& hasFrontier, std::vector<bool>& stoppingConditionNotMet, std::vector<bool>& buddyWait, Intersection*& forwardMeeting, Intersection*& reverseMeeting, bool& meetingPointFound, bool& searchSpaceExhausted)
  {
    // std::cout << myDirection(jobID) << " entering shortestPath()" << std::endl;
    result = new Path(od.origin, od.destination, od.flow);

    // tell my buddy that i'm here:
    buddyWait[jobID] = false;
    // manual spin-wait for my buddy
    while (waitForMyBuddy(jobID,buddyWait)) 
    {
      // std::cout << ".";
    }
    buddyWait[jobID] = true;
    
    while (stoppingConditionNotMet[jobID])
    {
      // std::cout << myDirection(jobID) << " in while loop after setup of initial variables." << std::endl;

      // load data from my half of the search into the shared buckets
      // comparison is a cross-product heuristic to find the best candidate
      // intersections to expand each half of the search
      search[jobID]->loadCompareList(intersections[jobID],coordinates[jobID],distances[jobID]);
      // std::cout << myDirection(jobID) << " compare list loaded from this jobID's BidirectionalAStar search" << std::endl;
      // there may be no intersections left to select
      if (intersections[jobID].size() == 0)
        hasFrontier[jobID] = false;

      buddyWait[jobID] = false;
      // manual spin-wait for my buddy
      while (waitForMyBuddy(jobID,buddyWait)) 
      {
        // std::cout << ".";
      }
      buddyWait[jobID] = true;      

      // master section
      #pragma omp critical
      if (isLocalMaster(jobID))
      {
        // std::cout << "frontier set: " << std::endl;
        // for (std::vector<std::list<Intersection*> >::iterator iter = intersections[jobID].begin(); iter != intersections[jobID].end(); ++iter)
        // {
          // std::cout << " loopy" << std::endl;
        // for (std::list<Intersection*>::iterator iter = intersections[jobID].begin(); iter != intersections[jobID].end(); iter++)
        // {
        //   std::cout << "found intersection: ";
        //   IntersectionProperty* prop = (IntersectionProperty*) (*iter)->getProps();
        //   std::cout << prop->getName() << " ";
        // }
        // }
        std::cout << myDirection(jobID) << " master - critical section" << std::endl;
        meetingPointFound = stoppingTest(search[jobID],search[jobID+1],forwardMeeting,reverseMeeting);
        std::cout << "meeting point: " << forwardMeeting << " == " << reverseMeeting << ": " << ((forwardMeeting!=0)&&(forwardMeeting==reverseMeeting) ? (forwardMeeting->getName()) : "not found") << std::endl;
        searchSpaceExhausted = (!hasFrontier[jobID] && !hasFrontier[jobID+1]);
        // do we have a meeting point?  or, are both frontiers empty?
        std::cout << "meetingPointFound: " << meetingPointFound << " searchSpaceExhausted: " << searchSpaceExhausted << std::endl;
        if (meetingPointFound || searchSpaceExhausted)
        {
          std::cout << myDirection(jobID) << " master - stopping condition or search space exhausted" << std::endl;
          // if stoppingTest was true, it also had the side effect of removing all
          // intersections except the ones that match (since finding those is part
          // of how the stopping test occurs, that O(n^2) operation should only
          // happen once)
          stoppingConditionNotMet[jobID] = false;
          stoppingConditionNotMet[jobID+1] = false;
        }
        // choose next selection for both searches 
        else
        {
          std::cout << myDirection(jobID) << " master - selecting best frontier node and then clearing lists" << std::endl;
          // this also clears the lists
          search[jobID]->compareLists(intersections[jobID],coordinates[jobID],distances[jobID],intersections[jobID+1],coordinates[jobID+1],distances[jobID+1]);
          std::cout << myDirection(jobID) << " master - selecting best frontier complete" << std::endl;
          std::cout << myDirection(jobID) << " master - best frontier buckets have sizes " << intersections[jobID].size() << " and " << intersections[jobID].size() << std::endl;
          Intersection* left = intersections[jobID].front();
          Intersection* right = intersections[jobID+1].front();
          std::cout << "I have my pointers" << std::endl;
          std::cout << myDirection(jobID) << " master - chose: " << left->getName() << " and " << right->getName() << std::endl;
        }
      }

      // {master|slave} section
      #pragma omp critical
      if (stoppingConditionNotMet[jobID])
      {
        if (hasFrontier[jobID])
        {
          // std::cout << myDirection(jobID) << " master|slave - here's my selected node to move" << std::endl;

          std::cout << intersections[jobID].front()->getName() << std::endl;
          // continue search
          // std::cout << myDirection(jobID) << " master|slave - move new node from frontier to selected" << std::endl;
          search[jobID]->moveToSelected(intersections[jobID].front());
          // std::cout << myDirection(jobID) << " master|slave - move completed, this iteration's intersections cleared" << std::endl;
          std::cout << myDirection(jobID) << " master|slave - printing current search state" << std::endl;
          search[jobID]->printLists();
        }
      }

      buddyWait[jobID] = false;
      // manual spin-wait for my buddy
      while (waitForMyBuddy(jobID,buddyWait)) 
      {
        // std::cout << ".";
      }
      buddyWait[jobID] = true; 

      // master section
      #pragma omp critical
      if (isLocalMaster(jobID))
      {
        if (meetingPointFound)
        {
          std::cout << myDirection(jobID) << " master - meeting point found, merging" << std::endl;

          // shortest path found
          // end search [master]
          // merge paths found
          // intersections has our intersection where they met
          // attach the two paths from search
          std::cout << myDirection(jobID) << " master - merge the searches" << std::endl;
          result->route = search[jobID]->mergeBidirectionalPaths(search[jobID+1], forwardMeeting);
          std::cout << myDirection(jobID) << " master - merge completed" << std::endl;
          search[jobID]->clearLists(intersections[jobID],coordinates[jobID],distances[jobID],intersections[jobID+1],coordinates[jobID+1],distances[jobID+1]); 
          std::cout << myDirection(jobID) << " master - all shared memory for this job pair was cleared" << std::endl;
        }
        else if (searchSpaceExhausted)
        {
          // search space was exhausted, so stoppingConditionNotMet should be tripped false
          std::cout << myDirection(jobID) << " master - clearing the buckets" << std::endl;
          search[jobID]->clearLists(intersections[jobID],coordinates[jobID],distances[jobID],intersections[jobID+1],coordinates[jobID+1],distances[jobID+1]); 
          std::cout << myDirection(jobID) << " master - doing nothing since search space was exhausted" << std::endl;
        }
        else
        {
          // continue
          std::cout << myDirection(jobID) << " master - meeting point not found, continuing search" << std::endl;
          std::cout << myDirection(jobID) << " master - clearing the buckets" << std::endl;
          search[jobID]->clearLists(intersections[jobID],coordinates[jobID],distances[jobID],intersections[jobID+1],coordinates[jobID+1],distances[jobID+1]); 
        }
      }
      buddyWait[jobID] = false;
      // manual spin-wait for my buddy
      while (waitForMyBuddy(jobID,buddyWait)) 
      {
        // std::cout << ".";
      }
      buddyWait[jobID] = true;  
    }
    std::cout << myDirection(jobID) << " master|slave - end of shortestPath" << std::endl;
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

  // even numbered pid's will be master to their odd-numbered counterparts pid+1
  bool isLocalMaster (int pid)
  {
    return (pid % 2) == 0;
  }

  // test the two selected lists for a connection
  bool RoadNetwork::stoppingTest (BidirectionalAStar* lhs, BidirectionalAStar* rhs, Intersection*& aMatch, Intersection*& bMatch)
  {
    std::pair<AStarMapIterator,AStarMapIterator> a = lhs->getSelectedIterator();
    std::pair<AStarMapIterator,AStarMapIterator> b = rhs->getSelectedIterator();
    for (AStarMapIterator i = a.first; i != a.second; ++i)
    {
      for (AStarMapIterator j = b.first; j != b.second; ++j)
      {
        if ((*i).first!=0 && (*i).first==(*j).first)
        {
          // @TODO: these should be the same so maybe just "match"
          aMatch = (*i).first;
          bMatch = (*j).first;
          return true;
        }
      }
    }  
    return false;
  }

  std::string myDirection (int id)
  {
    return (((id % 2) == 0) ? "forward " : "backward "); 
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

  bool waitForMyBuddy(int jobID, std::vector<bool> buddyWait)
  {
    if (jobID % 2 == 0)
      return buddyWait[jobID+1];
    else
      return buddyWait[jobID-1];
  }

  A_STAR_DIRECTION pickSearchDirection (int id)
  {
    return ((id % 2) == 0) ? FORWARD : BACKWARD;
  }
}