#include <iostream>

#include "RoadNetwork/RoadNetwork.h"
#include "RoadNetwork/Intersection.h"
#include "RoadNetwork/Roadway.h"
#include "utility/readFile.h"


namespace proj = csci7551_project;

int main () 
{
  std::cout << "----- top -----" << std::endl;
  proj::RoadNetwork roadNetwork(proj::BPL, 0.15, 4.0);
  std::vector<proj::ODPair> odPairs;
  proj::readFile("three_road_network.graph", roadNetwork);

  std::cout << "----- file read complete -----" << std::endl;

  proj::Intersection* O = roadNetwork.getIntersection("O");
  proj::Intersection* D = roadNetwork.getIntersection("D");
  proj::Intersection* A = roadNetwork.getIntersection("A"); 
  proj::ODPair p(A,O,50);
  proj::ODPair q(A,D,100);
  odPairs.push_back(p);
  odPairs.push_back(q);

  std::cout << "----- about to toString -----" << std::endl;

  std::cout << roadNetwork.toString() << std::endl;
  roadNetwork.runAllShortestPaths(odPairs);
  return 0;
}