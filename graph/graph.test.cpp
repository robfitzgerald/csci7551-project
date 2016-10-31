#include "Vertex.h"
#include <iostream>
#include <vector>

int main()
{
  using namespace csci7551_project;
  Vertex d, e, f, g;

  std::cout << d.getID() << " " << e.getID() << " " << f.getID() << std::endl;
  d.connect(&e,7);
  d.connect(&f,13.4);
  g.connect(&d, new P_Smock(5,10,100));

  std::vector<Edge*> outflows = d.getOutflows();
  for (int i = 0; i < outflows.size(); ++i)
  {
    std::cout << "id: " << outflows[i]->getID() << ", weight: " << outflows[i]->getProps()->weight() << std::endl;
  }

  std::cout << std::endl;

  std::vector<Edge*> gOut = g.getOutflows();
  for (int i = 0; i < gOut.size(); ++i)
  {
    std::cout << "id: " << gOut[i]->getID() << ", weight: " << gOut[i]->getProps()->weight() << std::endl;
    for (int j = 0; j < 10; ++j)
    {
      std::cout << "cost with V = " << j << ": " << gOut[i]->getProps()->cost(j) << std::endl;
    }
  }


  std::cout << std::endl;

  return 0;
}