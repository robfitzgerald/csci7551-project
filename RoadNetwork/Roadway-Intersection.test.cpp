
#include <iostream>

#include "Graph.h"
#include "Roadway.h"
#include "Intersection.h"
#include "IntersectionProperty.h"
#include "CostFunction.h"

using namespace csci7551_project;

int main()
{
  CostFunctionFactory costFunctionFactory(0.15, 4);
  CostFunction* smock = costFunctionFactory(SMOCK);
  Intersection* a = new Intersection(new IntersectionProperty(-1,-1,"bottom left")), 
    *b = new Intersection(new IntersectionProperty(0,1,"center top")),
    *c = new Intersection(new IntersectionProperty(1,-1,"bottom right"));
  Edge* ab = a->connect(b,smock),
    *bc = b->connect(c,smock),
    *ca = c->connect(a,smock);
  std::cout << a->getID() << b->getID() << c->getID() << std::endl;
  IntersectionProperty* aProps = (IntersectionProperty*) a->getProps(),
    *bProps = (IntersectionProperty*) b->getProps(),
    *cProps = (IntersectionProperty*) c->getProps();
  std::cout << aProps->getName() << " " << aProps->getX() << " " << aProps->getY() << std::endl;

  return 0;
}