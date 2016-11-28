#include <iostream>

#include "Edge-Roadway.h"

using namespace csci7551_project;

int main()
{
  CostFunctionFactory costFunctionFactory(0.15, 4);
  Vertex s, d;
  CostFunction* C = costFunctionFactory(SMOCK);
  Roadway* r = new Roadway(&s, &d, C);
  r->setDistance(10)
    ->setFlow(10)
    ->setFreeFlowTime(50)
    ->setCapacity(100);
  std::cout << "cost: " << r->cost() << ", distance: " << r->weight() << std::endl;
}