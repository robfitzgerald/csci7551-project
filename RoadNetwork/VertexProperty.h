#ifndef CSCI7551_PROJECT_GRAPH_VERTEXPROPERTY_H_
#define CSCI7551_PROJECT_GRAPH_VERTEXPROPERTY_H_

#include <string>

namespace csci7551_project
{
  class VertexProperty
  {
  public:
    VertexProperty(int lon, int lat): x(lon), y(lat) {}
    VertexProperty(int lon, int lat, std::string n): x(lon), y(lat), name(n) {}
    inline int getX() { return x; }
    inline int getY() { return y; }
    inline std::string getName() { return name; }
  private:
    int x, y;
    std::string name;
  };
}

#endif