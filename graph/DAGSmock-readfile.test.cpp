#include "DAGSmock.h"
#include "readfile/readfile.h"

#include <iostream>

int main ()
{

  csci7551_project::DAGSmock G;
  readfile("three_road_network.graph", G);
  std::cout << G.toString();

  return 0;
}