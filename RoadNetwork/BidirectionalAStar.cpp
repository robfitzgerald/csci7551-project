#include <list>
#include <vector>
#include <cmath>
#include <limits>
#include <iostream>
#include <string>

#include "BidirectionalAStar.h"
#include "Intersection.h"

namespace csci7551_project
{
  typedef std::vector<NodeCostTuple> NeighborVector;
  typedef std::vector<NodeCostTuple>::iterator NeighborIterator;
  typedef std::list<std::pair<double, double> >::iterator CoordinatesIterator;
  typedef std::list<double>::iterator DistancesIterator;
  typedef std::list<Intersection*>::iterator IntersectionIterator;
  typedef std::list<FrontierCost>::iterator FrontierCostIterator;
  typedef std::pair<Intersection*,AStarNode*> AStarMapTuple;
  typedef std::pair<std::list<Intersection*>::iterator,std::list<Intersection*>::iterator> SlidingIntersectionIterator;
  typedef std::pair<std::list<Intersection*>::iterator,std::list<Intersection*>::iterator> SlidingReverseIntersectionIterator;
  
  const double SENTINEL = std::numeric_limits<double>::max();

  void BidirectionalAStar::updateFrontier(Intersection* i)
  {
    AStarMapIterator parentIterator = selected.find(i);
    if (parentIterator != selected.end())
    {
      std::string directionString = ((direction == FORWARD) ? "forward" : "backward");
      AStarNode* parent = parentIterator->second;
      NeighborVector neighbors;
      if (direction == FORWARD)
        neighbors = i->getForwardNeighbors();
      else if (direction == BACKWARD)
        neighbors = i->getReverseNeighbors();

      std::cout << directionString << " updateFrontier, found parent: " << parentIterator->second->node->getName() << std::endl;

      // for each neighbor of each selected node
      for (NeighborIterator j = neighbors.begin(); j != neighbors.end(); ++j)
      {
        AStarMapIterator frontierIterator = frontier.find(j->node);
        AStarMapIterator selectedIterator = selected.find(j->node);
        double totalDistanceViaThisPath = parent->getPathDistance() + j->distance;
        double totalCostViaThisPath = parent->getPathCost() + j->cost;

        // if it's not already in the frontier and not in selected, add it to the frontier        
        if ((frontierIterator == frontier.end()) && (selectedIterator == selected.end()))
        {
          AStarNode* newFrontier = new AStarNode(*j);
          // calculate g for it (may be overwritten if found again)
          newFrontier->setPathDistance(totalDistanceViaThisPath);
          newFrontier->setPathCost(totalCostViaThisPath);
          // add path so far
          newFrontier->setPathAndAppend(parent->getPath(), parent->node);
          // add it to the frontier
          frontier.insert(AStarMapTuple((*j).node,newFrontier));
          std::cout << directionString << " updateFrontier, found child: " << (*j).node->getName() << std::endl;
        }
        // if it's already in frontier (only), check that this isn't a lower-cost path to it
        else if ((frontierIterator != frontier.end()) && (selectedIterator == selected.end()))
        {
          AStarNode* node = frontierIterator->second;
          if (totalCostViaThisPath < node->getPathCost())
          {
            // the way we got to it this time is quicker. update path and distance
            node->setPathDistance(totalDistanceViaThisPath);
            node->setPathCost(totalCostViaThisPath);
            node->setPathAndAppend(parent->getPath(), parent->node);
            std::cout << directionString << " updateFrontier, updated child cost: " << (*j).node->getName() << std::endl;
          }
        }
      }
    }
  }

  std::list<FrontierCost> BidirectionalAStar::frontierCosts()
  {
    std::list<FrontierCost> output;
    // std::cout << "starting frontierCosts with frontier of size " << frontier.size() << std::endl;
    for (AStarMapIterator iter = frontier.begin(); iter != frontier.end(); ++iter)
    {
      double pathCost = iter->second->getPathCost();
      IntersectionProperty* props = iter->second->node->getIntersectionProperties();
      // std::cout << "inside frontierCosts: " << pathCost << ", " << props->getName() << std::endl;
      FrontierCost tuple(iter->first, Coordinate(props->getX(), props->getY()), pathCost);
      output.push_back(tuple);
    }
    return output;
  }

  void BidirectionalAStar::loadCompareList (std::list<Intersection*>& intersections, std::list<std::pair<double, double> >& coordinates, std::list<double>& distances)
  {
    std::list<FrontierCost> costs = frontierCosts();
    for (FrontierCostIterator iter = costs.begin(); iter != costs.end(); ++iter)
    {
      intersections.push_back(iter->node);
      coordinates.push_back(Coordinate(iter->x,iter->y));
      distances.push_back(iter->cost);
    }
  }

  bool BidirectionalAStar::moveToSelected (Intersection* i)
  {
    AStarMapIterator iter = frontier.find(i);
    if (iter != frontier.end())
    {
      Intersection* key = iter->first;
      AStarNode* value = iter->second;
      value->setDirection(this->direction);
      selected.insert(AStarMapTuple(key, value));
      frontier.erase(i);
      updateFrontier(i);
      // setTopDistance(value->getPathDistance());
      return true;
    }
    else
      return false;
  }

  std::list<Roadway*> BidirectionalAStar::mergeBidirectionalPaths(BidirectionalAStar* reverseSearch, Intersection* meetingPoint)
  {
    std::list<Roadway*> output;
    std::list<Intersection*> leftPath = this->getAStarNodeFromIntersection(meetingPoint)->getPath();
    std::list<Intersection*> rightPath = reverseSearch->getAStarNodeFromIntersection(meetingPoint)->getPath();
    
    std::cout << "left path: ";
    for (std::list<Intersection*>::iterator i = leftPath.begin(); i != leftPath.end(); ++i)
    {
      std::cout << (*i)->getName() << ", ";
    }
    std::cout << std::endl;

    std::cout << "merge with boilerplate init complete" << std::endl;
    for (SlidingIntersectionIterator i = SlidingIntersectionIterator(leftPath.begin(),(leftPath.begin()++)); i.second != leftPath.end(); (i.first)++, (i.second)++)
    {
      // find all the edges between successive values
      std::cout << "merge - we have an iterator for leftPath" << std::endl;
      std::vector<Roadway*> outRoads = (*i.first)->getOutRoads();
      for (std::vector<Roadway*>::iterator j = outRoads.begin(); j != outRoads.end(); ++j) 
      {
        std::cout << "merge - we have an iterator for the roadways" << std::endl;
        std::cout << "comparing " << (*j)->getDestinationIntersection()->getName() << " with " << (*(i.second))->getName() << std::endl;
        if ((*j)->getDestinationIntersection() == (*(i.second)))
        {
          std::cout << "merge - pushing back road between " << (*(i.first))->getName() << " and " << (*(i.second))->getName() << std::endl;
          output.push_back((*j));
          j = outRoads.end();
        }
      }
    }

    for (SlidingReverseIntersectionIterator i = SlidingIntersectionIterator((leftPath.end()--),(leftPath.end())); i.second != leftPath.begin(); (i.first)--, (i.second)--)
    {
      // find all the edges between successive values
      std::vector<Roadway*> inRoads = (*(i.second))->getInRoads();
      std::cout << "merge - we have an iterator for rightPath" << std::endl;
      for (std::vector<Roadway*>::iterator j = inRoads.begin(); j != inRoads.end(); ++j)
      {
        std::cout << "merge - we have an iterator for the second roadways" << std::endl;
        std::cout << "comparing " << (*j)->getDestinationIntersection()->getName() << " with " << (*(i.second))->getName() << std::endl;
        if ((*j)->getSourceIntersection() == (*(i.first)))
        {
          std::cout << "merge - pushing back road between " << (*(i.first))->getName() << " and " << (*(i.second))->getName() << std::endl;
          output.push_back((*j));
          j = inRoads.end();
        }
      }
    }
    return output;
  }

  AStarNode* BidirectionalAStar::getAStarNodeFromIntersection (Intersection* target)
  {
    return selected.find(target)->second;
  }

  void BidirectionalAStar::printLists()
  {
    std::cout << "----- Printing Lists in this search -----" << std::endl;
    std::cout << "-- Frontier Nodes --" << std::endl;
    for (std::map<Intersection*, AStarNode*>::iterator i = frontier.begin(); i != frontier.end(); ++i)
    {
      std::cout << (*i).first->getName() << ", distance: " << (*i).second->getPathDistance() << ", cost: " << (*i).second->getPathCost() << std::endl;
    }
    std::cout << "-- Selected Nodes --" << std::endl;
    for (std::map<Intersection*, AStarNode*>::iterator i = selected.begin(); i != selected.end(); ++i)
    {
      std::cout << (*i).first->getName() << ", distance: " << (*i).second->getPathDistance() << ", cost: " << (*i).second->getPathCost() << std::endl;      
    }
  }

  struct CompareListIter
  {
    CompareListIter(std::list<Intersection*>::iterator a, std::list<std::pair<double, double> >::iterator b, std::list<double>::iterator c)
    {
      ints = a;
      coords = b;
      dists = c;
    }
    std::list<Intersection*>::iterator ints;
    std::list<std::pair<double, double> >::iterator coords;
    std::list<double>::iterator dists;
  };

  // at the end, best picks will be in leftIntersections[0] and rightIntersections[0]
  void BidirectionalAStar::compareLists (std::list<Intersection*>& leftIntersections, std::list<std::pair<double, double> >& leftCoordinates, std::list<double>& leftDistances, std::list<Intersection*>& rightIntersections, std::list<std::pair<double, double> >& rightCoordinates, std::list<double>& rightDistances)
  {
    // std::cout << "compareLists" << std::endl;
    Intersection *bestLeft = 0, *bestRight = 0;
    double bestHeuristic = SENTINEL;
    std::list<std::pair<double, double> >::iterator leftCoordIter = leftCoordinates.begin();
    std::list<double>::iterator leftDistIter = leftDistances.begin();
    
    for (CompareListIter leftIter(leftIntersections.begin(),leftCoordinates.begin(),leftDistances.begin()); \
      leftIter.ints != leftIntersections.end() && leftIter.coords != leftCoordinates.end() && leftIter.dists != leftDistances.end(); \
      leftIter.ints++, leftIter.coords++, leftIter.dists++)
    {
      // std::cout << "outer loop: " << (*(leftIter.ints))->getName() << std::endl;
      for (CompareListIter rightIter(rightIntersections.begin(),rightCoordinates.begin(),rightDistances.begin()); \
        rightIter.ints != rightIntersections.end() && rightIter.coords != rightCoordinates.end() && rightIter.dists != rightDistances.end(); \
        rightIter.ints++, rightIter.coords++, rightIter.dists++)
      {
        // std::cout << "looking at " << (*(leftIter.ints))->getName() << "," << (*(rightIter.ints))->getName() << std::endl;
        double thisHeuristic = heuristic(*(leftIter.coords), *(leftIter.dists), *(rightIter.coords), *(rightIter.dists));
        // std::cout << "thisHeuristic: " << thisHeuristic << " from center distance: " << euclidianDistance(*(leftIter.coords),*(rightIter.coords)) << std::endl;
        // std::cout << "left, right intersections: " << (*(leftIter.ints))->getName() << ", " << (*(rightIter.ints))->getName() << std::endl;
        if (thisHeuristic < bestHeuristic)
        {
          bestLeft = *(leftIter.ints);
          bestRight = *(rightIter.ints);
          bestHeuristic = thisHeuristic;
        }
      }
    }
    if (bestLeft != 0 && bestRight != 0)
      std::cout << "compareLists result, choose " << bestLeft->getName() << " " << bestRight->getName() << " with heuristic " << bestHeuristic << std::endl;
    clearLists(leftIntersections,leftCoordinates,leftDistances,rightIntersections,rightCoordinates,rightDistances);
    leftIntersections.push_front(bestLeft);
    rightIntersections.push_front(bestRight);
  }

  void BidirectionalAStar::clearLists (std::list<Intersection*>& a, std::list<std::pair<double, double> >& b, std::list<double>& c, std::list<Intersection*>& d, std::list<std::pair<double, double> >& e, std::list<double>& f)
  {
    // std::cout << "a.clear();" << std::endl;
    a.clear();
    // std::cout << "b.clear();" << std::endl;
    b.clear();
    // std::cout << "c.clear();" << std::endl;
    c.clear();
    // std::cout << "d.clear();" << std::endl;
    d.clear();
    // std::cout << "e.clear();" << std::endl;
    e.clear();
    // std::cout << "f.clear();" << std::endl;
    f.clear();
  }

  double heuristic (Coordinate left, double leftD, Coordinate right, double rightD)
  {
    return leftD + euclidianDistance(left, right) + rightD;
  }

  double euclidianDistance (Coordinate l, Coordinate r)
  {
    // std::cout << "BidirectionalAStar euclidianDistance(): ";    
    double x = l.first - r.first;
    double y = l.second - r.second;
    // std::cout << "(" << x << "," << y << ") - " << (((x + y) == 0) ? 0 : sqrt(x*x + y*y)) << std::endl;
    return ((x + y) == 0) ? 0 : sqrt(x*x + y*y);
  }
}