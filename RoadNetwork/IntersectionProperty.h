#ifndef CSCI7551_PROJECT_INTERSECTIONPROPERTY_H_
#define CSCI7551_PROJECT_INTERSECTIONPROPERTY_H_

#include <string>

#include "Graph.h"

namespace csci7551_project
{
  class IntersectionProperty : public VertexProperty 
  {
  public:
    IntersectionProperty(double lon, double lat): 
      VertexProperty(), 
      x(lon), 
      y(lat) {}
    IntersectionProperty(double lon, double lat, std::string n): 
      VertexProperty(), 
      x(lon), 
      y(lat), 
      name(n) {}
    inline double getX() { return x; }
    inline double getY() { return y; }
    inline std::string getName() { return name; }
  private:
    int x, y;
    std::string name;
  };
}

#endif