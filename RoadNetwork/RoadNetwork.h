#ifndef CSCI7551_PROJECT_ROADNETWORK_H_
#define CSCI7551_PROJECT_ROADNETWORK_H_

#include <map>
#include <vector>
#include <list>
#include <string>

#include "CostFunction.h"
#include "Intersection.h"
#include "Roadway.h"
#include "BidirectionalAStar.h"

namespace csci7551_project
{
  enum SIMULATION_TYPE { MSA, FRANK_WOLFE };  // sets simulation algorithm to run
  const unsigned int MSA_SIMULATION_ITERATIONS = 10;  // MSA algorithm step count

  /**
   * tuple which temporarily holds the origin/destination values for a trip, the path, and it's flow in a given simulation iteration
   */
  struct Path 
  {
  public:
    Path(Intersection* s, Intersection* e, double f): start(s), end(e), flow(f) {}
    ~Path(){}
    Intersection *start, *end;
    double flow;
    std::list<Roadway*> route;
  };

  /**
   * tuple which holds the origin/destination values and flow for a trip
   * @TODO: redundant: combine Path with ODPair
   */
  struct ODPair 
  {
    ODPair(Intersection *o, Intersection *d, double f): origin(o), destination(d), flow(f) {}
    ~ODPair(){}
    Intersection *origin, *destination;
    double flow;
  };

  /**
   * Graph class that contains the roadways and intersections, and performs operations on them
   */
  class RoadNetwork
  {
  public:
    RoadNetwork (COST_FUNCTION c, double alpha, double beta, SIMULATION_TYPE sim):
      simulationType(sim)
    {
      CostFunctionFactory costFunctionFactory(alpha, beta);
      this->costFunction = costFunctionFactory(c);
      simulationIterations = MSA_SIMULATION_ITERATIONS;
    }
    ~RoadNetwork()
    {
      V.erase(V.begin(), V.end());
      E.erase(E.begin(), E.end());
      // delete costFunction; <-- causes a warning, not needed tho (costFunction persists through application)
    }
    /**
     * adds an intersection with the given coordinate position and name 
     * @param double      x value (latitude)
     * @param double      y value (longitude)
     * @param std::string name
     */
    void addIntersection (double,double,std::string);
    /**
     * adds a roadway connected between the two named intersections
     * @param std::string origin intersection name
     * @param std::string destination intersection name
     * @param double      free flow time to pass through this roadway
     * @param double      steady state capacity of the roadway
     */
    void addRoadway (std::string,std::string,double,double);
    /**
     * pretty prints the current graph state
     * @return output as formatted string that is (|V| + |E| + C) lines long
     */
    std::string toString();
    /**
     * simulates the traffic assignment
     */
    inline void simulate(std::vector<ODPair>* odPairs)
    {
      if (simulationType==MSA)
        assignMSA(odPairs);
      else if (simulationType==FRANK_WOLFE)
        assignFrankWolfe(odPairs);
    }
    /**
     * set the simulation algorithm to run
     * @param s simulation type enumeration that switches the algorithm chosen
     */
    void setSimulationType (SIMULATION_TYPE s) { simulationType = s; }
    /**
     * lookup an intersection pointer by name (using in testing)
     * @param  std::string intersection name
     * @return             pointer to intersection, or zero if not found
     */
    Intersection* getIntersection (std::string);
  private:
    SIMULATION_TYPE simulationType;
    unsigned int simulationIterations;
    /**
     * runs a method of successive averages traffic assignment simulation with the provided trip matrix
     * @param std::vector<ODPair>* origin/destination pairs with flows (vehicle per time unit)
     */
    void assignMSA (std::vector<ODPair>*);
    /**
     * runs a Frank-Wolfe traffic assignment simulation with the provided trip matrix
     * @param std::vector<ODPair>* origin/destination pairs with flows (vehicle per time unit)
     */
    void assignFrankWolfe (std::vector<ODPair>*);
    void resetFlows ();
    void calculateNetworkFlows (std::vector<double>&);
    void calculateNetworkFlowCosts (std::vector<double>&);
    double calculateCurrentFlow (double, double, double);
    void loadAllOrNothing (std::vector<Path*>*);
    /**
     * convergence test for Frank Wolfe Algorithm
     * @param  currentFlows flows on each roadway at this iteration of the simulation
     * @param  aonFlows     all or nothing flow assignment at this iteration
     * @param  costs        costs on each roadway at this iteration of the simulation
     * @return              value to compare for stopping the simulation (i.e. if (relGap < 0.0001) { break; })
     */
    double relGapConvergenceTest (std::vector<double>, std::vector<double>, std::vector<double>);
    void runAllShortestPaths(std::vector<ODPair>*, std::vector<Path*>*);
    Path* shortestPath (ODPair, double, std::vector<std::list<Intersection*> >&, std::vector<std::list<std::pair<double,double> > >, std::vector<std::list<double> >, std::vector<BidirectionalAStar*>, int, std::vector<bool>, std::vector<bool>);
    bool stoppingTest (std::list<Intersection*>&, std::list<Intersection*>&, Intersection*&, Intersection*&);
    std::map<std::string,Intersection*> V;
    std::vector<Roadway*> E;
    CostFunction* costFunction;
  };

  double euclidianDistance (Intersection*,Intersection*);
  bool isLocalMaster (int);
  void printTree (Intersection*, int);
  A_STAR_DIRECTION pickSearchDirection (int);

}

#endif