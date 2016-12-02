#include "Graph.h"
#include <iostream>

int main() 
{
  csci7551_project::Vertex* a = new csci7551_project::Vertex(), 
    *b = new csci7551_project::Vertex(),
    *c = new csci7551_project::Vertex();
  csci7551_project::Edge* ab = a->connect(b),
    *bc = b->connect(c),
    *ca = c->connect(a);
  std::cout << a->getID() << b->getID() << c->getID() << std::endl;

  return 0;
}