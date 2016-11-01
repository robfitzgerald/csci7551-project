#include "DAGSmock.h"

int main ()
{
  csci7551_project::DAGSmock G;
  G.addIntersection(0,0,"A");
  G.addIntersection(5,5,"B");
  G.addRoadway("A","B",5,10);
  return 0;
}