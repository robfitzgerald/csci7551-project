#ifndef CSCI7551_PROJECT_READFILE_H_
#define CSCI7551_PROJECT_READFILE_H_

#include <string>

#include "../RoadNetwork/RoadNetwork.h"

// ƒ read_file,p,r)
//
// precondition:  file has one integer per line
// postcondition: returns a vector made of the integers stored in the file
//

namespace csci7551_project
{
  void readFile(const char*, RoadNetwork&);
  void parseNode (std::string, double&, double&, std::string&);
  void parseEdge (std::string, std::string&, std::string&, double&, double&);
}

#endif