#include <list>
#include <vector>
#include <limits>
#include <cmath>

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

  const double SENTINEL = std::numeric_limits<double>::max();
  
  void BidirectionalAStar::updateFrontier(Intersection* i)
  {
    AStarMapIterator parentIterator = selected.find(i);
    if (parentIterator != selected.end())
    {
      AStarNode* parent = parentIterator->second;
      NeighborVector* neighbors;
      if (direction == FORWARD)
        neighbors = i->node->getForwardNeighbors();
      else if (direction == BACKWARD)
        neighbors = i->node->getReverseNeighbors();

      // for each neighbor of each selected node
      for (NeighborIterator j = neighbors->begin(); j != neighbors->end(); ++j)
      {
        AStarMapIterator frontierIterator = frontier.find(j->first);
        AStarMapIterator selectedIterator = selected.find(j->first);
        double totalDistanceViaThisPath = parent->getPathDistance() + j->second;

        // if it's not already in the frontier and not in selected, add it to the frontier        
        if ((frontierIterator == frontier.end()) && (selectedIterator == selected.end()))
        {
          AStarNode* newFrontier = new AStarNode(NodeCostTuple(*j));
          // calculate g for it (may be overwritten if found again)
          newFrontier->setPathDistance(totalDistanceViaThisPath);
          // add path so far
          newFrontier->setPathAndAppend(parent->getPath(), parent);
          // add it to the frontier
          frontier.insert(newFrontier);
        }
        // if it's already in frontier (only), check that this isn't a lower-cost path to it
        else if ((frontierIterator != frontier.end()) && (selectedIterator == selected.end()))
        {
          AStarNode* node = frontierIterator->second;
          if (totalDistanceViaThisPath < node->getPathDistance())
          {
            // the way we got to it this time is quicker. update path and distance
            node->setPathDistance(totalDistanceViaThisPath);
            node->setPathAndAppend(parent->getPath(), parent);
          }
        }
      }
    }
  }

  std::list<FrontierCost> BidirectionalAStar::frontierCosts()
  {
    std::list<FrontierCost> output;
    
    for (AStarMapIterator iter = frontier.begin(); iter != frontier.end(); ++iter)
    {
      double pathDistance = iter->getPathDistance();
      intersectionProperty* props = iter->second->node->getIntersectionProperties();
      FrontierCost tuple(iter->first, Coordinate(props->getX(), props->getY()), pathDistance);
      output.push_back(tuple);
    }
    return output;
  }

  void loadCompareList (std::list<Intersection*>& intersections, std::list<std::pair<double, double> >& coordinates, std::list<double>& distances)
  {
    std::list<FrontierCost> costs = frontierCosts();
    for (FrontierCostIterator iter = costs.begin(); iter != costs.end(); ++i)
    {
      intersections.push_back(iter->node);
      coordinates.push_back(Coordinate(iter->x,iter->y));
      distances.push_back(iter->distance);
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
      selected.insert(std::pair<Intersection*,AStarNode*>(key, value));
      frontier.erase(i);
      updateFrontier(i);
      setTopDistance(value->getPathDistance());
      return true;
    }
    else
      return false;
  }

  std::list<Roadway*> mergeBidirectionalPaths(search[jobID+1] reverseSearch, Intersection* meetingPoint)
  {
    std::list<Roadway*> output;
    std::list<Intersection*> leftPath = this->getAStarNodeFromIntersection(meetingPoint)->getPath();
    std::list<Intersection*> rightPath = reverseSearch->getAStarNodeFromIntersection(meetingPoint)->getPath();
    for (int i = 0; i < leftPath.size(); ++i)
    {
      // find all the edges between successive values
      std::vector<Roadway*> outRoads = leftPath[i]->getOutRoads()
      for (std::vector<Roadway*>::iterator j = outRoads.begin(); j != outRoads.end(); ++j)
      {
        if (outRoads[j]->getDestinationIntersection() == leftPath[i+1])
        {
          output.push_back(outRoads[j]);
          j = outRoads.end();
        }
      }
    }
    for (int i = rightPath.size()-1; i >= 0; --i)
    {
      // find all the edges between successive values
      std::vector<Roadway*> inRoads = leftPath[i]->getInRoads()
      for (std::vector<Roadway*>::iterator j = inRoads.begin(); j != inRoads.end(); ++j)
      {
        if (inRoads[j]->getSourceIntersection() == leftPath[i-1])
        {
          output.push_back(inRoads[j]);
          j = inRoads.end();
        }
      }
    }
    return output;
  }

  AStarNode* BidirectionalAStar::getAStarNodeFromIntersection (Intersection* target)
  {
    return selected.find(meetingPoint);
  }

  // at the end, best picks will be in leftIntersections[0] and rightIntersections[0]
  void compareLists (std::list<Intersection*>& leftIntersections, std::list<std::pair<double, double> >& leftCoordinates, std::list<double>& leftDistances, std::list<Intersection*>& rightIntersections, std::list<std::pair<double, double> >& rightCoordinates, std::list<double>& rightDistances)
  {
    Intersection *bestLeft = 0, *bestRight = 0;
    double bestHeuristic = SENTINEL;
    for (int left = 0; left < leftIntersections.size(); ++left)
    {
      for (int right = 0; right < rightIntersections.size(); ++right)
      {
        double thisHeuristic = heuristic(leftCoordinates[left].first, leftCoordinates[left].second, leftDistances[left], rightCoordinates[right].first, rightCoordinates[right].second, rightDistances[right]);
        if (thisHeuristic < bestHeuristic)
        {
          bestLeft = leftIntersections[left];
          bestRight = rightIntersections[right];
          bestHeuristic = thisHeuristic;
        }
      }
    }
    clearLists(leftIntersections,leftCoordinates,leftDistances,rightIntersections,rightCoordinates,rightDistances);
    leftIntersections.push_back(bestLeft);
    rightIntersections.push_back(bestRight);
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