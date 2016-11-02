#include "CostFunction.h"
#include <iostream>

using namespace csci7551_project;

int main()
{
  CostFunctionFactory costFunctionFactory(0.15,4);
  CostFunction* smock = costFunctionFactory(SMOCK);
  CostFunction* overgaard = costFunctionFactory(OVERGAARD);
  CostFunction* bpl = costFunctionFactory(BPL);

  std::cout << "smock: " << smock->cost(10,50,100) << std::endl;
  std::cout << "overgaard: " << overgaard->cost(10,50,100) << std::endl;
  std::cout << "bpl: " << bpl->cost(10,50,100) << std::endl;
  
  return 0;    
}  