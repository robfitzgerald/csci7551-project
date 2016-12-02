#include <iostream>

#include "RoadNetwork/RoadNetwork.h"
#include "utility/readFile.h"

namespace proj = csci7551_project;

int main () 
{
  proj::RoadNetwork roadNetwork(proj::BPL, 0.15, 4.0);
  proj::readfile("three_road_network.graph", roadNetwork);
  std::cout << roadNetwork.toString();
  return 0;
}