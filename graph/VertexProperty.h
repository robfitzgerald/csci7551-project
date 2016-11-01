#ifndef CSCI7551_PROJECT_GRAPH_VERTEXPROPERTY_H_
#define CSCI7551_PROJECT_GRAPH_VERTEXPROPERTY_H_

#include <string>

namespace csci7551_project
{
  class VertexProperty {};

  class RoadIntersection
  {
  public:
    RoadIntersection(int lon, int lat): x(lon), y(lat) {}
    RoadIntersection(int lon, int lat, string n): x(lon), y(lat), name(n) {}
    inline int getX() { return x; }
    inline int getY() { return y; }
    inline string getName() { return n; }
  private:
    int x, y;
    string name;
  };
}

#endif