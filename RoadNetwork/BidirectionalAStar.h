#ifndef CSCI7551_PROJECT_BIDIRECTIONAL_A_STAR_H_
#define CSCI7551_PROJECT_BIDIRECTIONAL_A_STAR_H_

// #include <queue>
// #include <vector>
#include <utility>
#include <map>
#include <list>

#include "Intersection.h"
#include "Roadway.h"

namespace csci7551_project
{
  enum A_STAR_DIRECTION { NODIRECTION, FORWARD, BACKWARD };
  enum EXPLORE_STATE {UNSET, FRONTIER, SELECTED};
  typedef std::pair<double, double> Coordinate;
  typedef std::pair<Intersection*, double> NodeCostTuple;
  struct FrontierCost
  {
    FrontierCost(Intersection* n, Coordinate pos, double d):
      node(n),
      x(pos.first),
      y(pos.second),
      distance(d) {}
    Intersection* node;
    double x;
    double y;
    double distance;
  };

  class AStarNode
  {
  public:
    AStarNode(NodeCostTuple n): 
      node(n.first), 
      distance(n.second), 
      direction(NODIRECTION) {}
    ~AStarNode() { path.clear(); }

    inline std::list<Intersection*> getPath () { return path; }
    inline void setPath (Intersection* i) { path.push_back(i); }
    inline void setPath (std::list<Intersection*> p) { path = p; }
    inline void setPathAndAppend (std::list<Intersection*> p, Intersection* v) 
    { 
      path.clear(); 
      path = p; 
      path.push_back(v);
    }
    inline void setPathDistance (double d) { pathDistance = d; }
    inline double getPathDistance () { return pathDistance; }
    inline bool setDirection (A_STAR_DIRECTION d)
    {
      if (direction == 0) 
      {
        direction = d;
        return true;
      }
      else
        return false;
    }
    Intersection* node;
  private:
    double distance;
    double pathDistance;
    A_STAR_DIRECTION direction;
    EXPLORE_STATE state;
    std::list<Intersection*> path;
  };

  typedef std::map<Intersection*, AStarNode*>::iterator AStarMapIterator;

  class BidirectionalAStar
  {
  public:
    BidirectionalAStar (Intersection* s, A_STAR_DIRECTION d): 
      // topDistance(0),
      latest(s),
      direction(d)
    {
      NodeCostTuple source(s,0);
      AStarNode* n = new AStarNode(source);
      n->setPath(s);
      selected.insert(std::pair<Intersection*,AStarNode*>(s,n));
    }
    ~BidirectionalAStar ()
    {
      for (AStarMapIterator i = selected.begin(); i != selected.end(); ++i)
      {
        delete i->second;
      }
      for (AStarMapIterator i = frontier.begin(); i != frontier.end(); ++i)
      {
        delete i->second;
      }
      selected.clear();
      frontier.clear();
    }
    void updateFrontier (Intersection*);
    void loadCompareList (std::list<Intersection*>&, std::list<std::pair<double, double> >&, std::list<double>&);
    bool moveToSelected (Intersection*);
    const AStarMapIterator getSelectedIterator () { return selected.begin(); }
    std::list<Roadway*> mergeBidirectionalPaths(BidirectionalAStar*, Intersection*);
    AStarNode* getAStarNodeFromIntersection (Intersection*);
  private:
    std::list<FrontierCost> frontierCosts ();
    A_STAR_DIRECTION direction;
    // double topDistance;
    Intersection* latest;
    std::map<Intersection*, AStarNode*> frontier;
    std::map<Intersection*, AStarNode*> selected;
  };
  void compareLists (std::list<Intersection*>&, std::list<std::pair<double, double> >&, std::list<double>&, std::list<Intersection*>&, std::list<std::pair<double, double> >&, std::list<double>&);
  void clearLists (std::list<Intersection*>&, std::list<std::pair<double, double> >&, std::list<double>&, std::list<Intersection*>&, std::list<std::pair<double, double> >&, std::list<double>&);
  double heuristic (Coordinate, double, Coordinate, double);
  double euclidianDistance (Coordinate,Coordinate);
}

#endif