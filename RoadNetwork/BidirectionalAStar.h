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
  class AStarNode;

  enum A_STAR_DIRECTION { NODIRECTION, FORWARD, BACKWARD };
  enum EXPLORE_STATE {UNSET, FRONTIER, SELECTED};
  typedef std::pair<double, double> Coordinate;
  typedef std::map<Intersection*, AStarNode*>::iterator AStarMapIterator;

  struct FrontierCost
  {
    FrontierCost(Intersection* n, Coordinate pos, double d):
      node(n),
      x(pos.first),
      y(pos.second),
      cost(d) {}
    Intersection* node;
    double x;
    double y;
    double cost;
  };

  class AStarNode
  {
  public:
    AStarNode(NodeCostTuple n): 
      node(n.node), 
      distance(n.distance), 
      cost(n.cost),
      direction(NODIRECTION) 
      {
        pathDistance = 0;
        pathCost = 0;
      }
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
    inline void setPathCost (double d) { pathCost = d; }
    inline double getPathCost () { return pathCost; }
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
    double distance, cost;
    double pathDistance, pathCost;
    A_STAR_DIRECTION direction;
    // EXPLORE_STATE state;
    std::list<Intersection*> path;
  };

  // http://stackoverflow.com/questions/25122932/pointers-as-keys-in-map-c-stl
  template<class T> struct ptr_less {
    bool operator()(T* lhs, T* rhs) {
        return *lhs < *rhs; }};

  class BidirectionalAStar
  {
  public:
    BidirectionalAStar (Intersection* s, A_STAR_DIRECTION d): 
      direction(d)
    {
      NodeCostTuple source(s,0,0);
      AStarNode* n = new AStarNode(source);
      n->setPath(s);
      frontier.insert(std::pair<Intersection*,AStarNode*>(s,n));
      moveToSelected(s);
    }
    ~BidirectionalAStar ()
    {
      for (AStarMapIterator i = selected.begin(); i != selected.end(); ++i)
      {
        delete i->second; // this should just clear the backtrace paths in the search
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
    std::pair<AStarMapIterator,AStarMapIterator> getSelectedIterator () { return std::pair<AStarMapIterator,AStarMapIterator>(selected.begin(),selected.end()); }
    std::list<Roadway*> mergeBidirectionalPaths(BidirectionalAStar*, Intersection*);
    AStarNode* getAStarNodeFromIntersection (Intersection*);
    std::list<FrontierCost> frontierCosts ();
    void printLists();
    void compareLists (std::list<Intersection*>&, std::list<std::pair<double, double> >&, std::list<double>&, std::list<Intersection*>&, std::list<std::pair<double, double> >&, std::list<double>&);
    void clearLists (std::list<Intersection*>&, std::list<std::pair<double, double> >&, std::list<double>&, std::list<Intersection*>&, std::list<std::pair<double, double> >&, std::list<double>&);
  private:
    A_STAR_DIRECTION direction;
    std::map<Intersection*, AStarNode*, ptr_less<Intersection> > frontier;
    std::map<Intersection*, AStarNode*, ptr_less<Intersection> > selected;
  };
  double heuristic (Coordinate, double, Coordinate, double);
  double euclidianDistance (Coordinate,Coordinate);
}

#endif