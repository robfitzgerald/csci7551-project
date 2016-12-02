#include <iostream>

#include "RoadNetwork.h"
#include "CostFunction.h"

using namespace csci7551_project;

int main ()
{

  RoadNetwork* roadNetwork = new RoadNetwork(BPL, 0.15, 4.0);
  roadNetwork->addIntersection(-1,-1,"bottom left");
  roadNetwork->addIntersection(0,1,"center top");
  roadNetwork->addIntersection(1,-1,"bottom right");
  roadNetwork->addRoadway("bottom left", "center top", 25, 100);
  roadNetwork->addRoadway("bottom left", "bottom right", 45, 400);
  std::cout << roadNetwork->toString();

  return 0;
}