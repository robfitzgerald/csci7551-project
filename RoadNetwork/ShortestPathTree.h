#ifndef CSCI7551_PROJECT_SHORTESTPATHTREE_H_
#define CSCI7551_PROJECT_SHORTESTPATHTREE_H_

#include <queue>
#include <vector>
#include <utility>

#include "Intersection.h"
#include "Roadway.h"

namespace csci7551_project
{
  typedef std::pair<Intersection*, double> NodeCostTuple;
  class SPTNode
  {
  public:
    SPTNode(NodeCostTuple n): distance(n.second), node(n.first), explored(false) {}
    ~SPTNode(){}
    inline bool operator < (const SPTNode& r) const
    {
      return this->pathDistance < r.pathDistance;
    }
    double distance, pathDistance;
    Intersection* node;
    bool explored;
  };

  class ShortestPathTree
  {
  public:
    ShortestPathTree(Intersection* s): top(0)
    {
      NodeCostTuple source(s,0);
      SPTNode n(source);
      frontier.push(n);
      std::vector<NodeCostTuple> neighbors = s->getNeighbors();
      for (int i = 0; i < neighbors.size(); ++i)
      {
        SPTNode t (neighbors[i]);
        frontier.push(t);
      }
    }
    ~ShortestPathTree()
    {
      explored.clear();
      frontier = std::priority_queue<SPTNode>();
    }

    // Intersection* minCostUnexplored();
    // void relax();
    bool unexploredRemain() { return frontier.size() > 0; }
  private:
    // void explore();
    unsigned top;
    std::vector<SPTNode> explored;
    std::priority_queue<SPTNode> frontier;
  };
}

#endif