#include <list>
#include <vector>
#include <cmath>
#include <limits>
#include <iostream>

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
      AStarNode* parent = parentIterator->second;
      NeighborVector neighbors;
      if (direction == FORWARD)
        neighbors = i->getForwardNeighbors();
      else if (direction == BACKWARD)
        neighbors = i->getReverseNeighbors();

      std::cout << "updateFrontier, found parent: " << parentIterator->second->node->getIntersectionProperties()->getName() << std::endl;

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
    for (SlidingIntersectionIterator i = SlidingIntersectionIterator(leftPath.begin(),(leftPath.begin()++)); i.second != leftPath.end(); i.first++, i.second++)
    {
      // find all the edges between successive values
      std::vector<Roadway*> outRoads = (*i.first)->getOutRoads();
      for (std::vector<Roadway*>::iterator j = outRoads.begin(); j != outRoads.end(); ++j) 
      {
        if ((*j)->getDestinationIntersection() == (*i.second))
        {
          output.push_back((*j));
          j = outRoads.end();
        }
      }
    }

    for (SlidingReverseIntersectionIterator i = SlidingIntersectionIterator((leftPath.end()--),(leftPath.end())); i.second != leftPath.begin(); i.first--, i.second--)
    {
      // find all the edges between successive values
      std::vector<Roadway*> inRoads = (*i.second)->getInRoads();
      for (std::vector<Roadway*>::iterator j = inRoads.begin(); j != inRoads.end(); ++j)
      {
        if ((*j)->getSourceIntersection() == (*i.first))
        {
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

  void clearLists (std::list<Intersection*>& a, std::list<std::pair<double, double> >& b, std::list<double>& c, std::list<Intersection*>& d, std::list<std::pair<double, double> >& e, std::list<double>& f)
  {
    a.clear();
    b.clear();
    c.clear();
    d.clear();
    e.clear();
    f.clear();
  }

  // at the end, best picks will be in leftIntersections[0] and rightIntersections[0]
  void compareLists (std::list<Intersection*>& leftIntersections, std::list<std::pair<double, double> >& leftCoordinates, std::list<double>& leftDistances, std::list<Intersection*>& rightIntersections, std::list<std::pair<double, double> >& rightCoordinates, std::list<double>& rightDistances)
  {
    Intersection *bestLeft = 0, *bestRight = 0;
    double bestHeuristic = SENTINEL;
    std::list<std::pair<double, double> >::iterator leftCoordIter = leftCoordinates.begin();
    std::list<double>::iterator leftDistIter = leftDistances.begin();
    for (std::list<Intersection*>::iterator leftIntIter = leftIntersections.begin(); leftIntIter != leftIntersections.end(); ++leftIntIter)
    {
      std::list<std::pair<double, double> >::iterator rightCoordIter = rightCoordinates.begin();
      std::list<double>::iterator rightDistIter = rightDistances.begin();
      for (std::list<Intersection*>::iterator rightIntIter = rightIntersections.begin(); rightIntIter != rightIntersections.end(); ++rightIntIter)
      {
        double thisHeuristic = heuristic((*leftCoordIter), (*leftDistIter), (*rightCoordIter), (*rightDistIter));
        std::cout << "thisHeuristic: " << thisHeuristic << " from center distance: " << euclidianDistance((*leftCoordIter),(*rightCoordIter)) << std::endl;
        std::cout << "left, right intersections: " << (*leftIntIter)->getName() << ", " << (*rightIntIter)->getName() << std::endl;
        if (thisHeuristic < bestHeuristic)
        {
          bestLeft = (*leftIntIter);
          bestRight = (*rightIntIter);
          bestHeuristic = thisHeuristic;
        }
        ++rightCoordIter;
        ++rightDistIter;        
      }
      ++leftCoordIter;
      ++leftDistIter;
    }
    clearLists(leftIntersections,leftCoordinates,leftDistances,rightIntersections,rightCoordinates,rightDistances);
    leftIntersections.push_back(bestLeft);
    rightIntersections.push_back(bestRight);
  }

  double heuristic (Coordinate left, double leftD, Coordinate right, double rightD)
  {
    return leftD + euclidianDistance(left, right) + rightD;
  }

  double euclidianDistance (Coordinate l, Coordinate r)
  {
    double x = l.first - r.first;
    double y = l.second - r.second;
    return sqrt(x*x + y*y);
  }
}