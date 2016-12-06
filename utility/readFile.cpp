//
//  readfile.cpp
//  csci3412_pa1
//
//  Created by Robert Fitzgerald on 2/4/15.
//
//

#include "readFile.h"

#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <exception>

#include "../RoadNetwork/RoadNetwork.h"

namespace csci7551_project
{
    void readFile(const char* fileName, RoadNetwork& G)
    {
        std::vector<int> result;
        std::ifstream file;
        file.open(fileName);
        
        if (file.good())
        {
            std::string line;
            bool nodesSection = false, edgesSection = false;

            while (getline(file,line))
            {
                if (line[0] == '#')
                {
                    // do nothing on comment lines
                } 
                else if (line.substr(0,7) == "! nodes")
                {
                    nodesSection = true;
                    edgesSection = false;
                } 
                else if (line.substr(0,7) == "! edges")
                {
                    edgesSection = true;
                    nodesSection = false;
                }
                else if (nodesSection)
                {
                    double x, y;
                    std::string name;
                    parseNode(line,x,y,name);
                    G.addIntersection(x,y,name);
                } 
                else if (edgesSection)
                {
                    std::string s, d;
                    double t, c;
                    parseEdge(line,s,d,t,c);
                    G.addRoadway(s,d,t,c);
                } 
                line.clear();
            }
        }
    }

    void parseNode (std::string line, double& x, double& y, std::string& name)
    {
        std::stringstream stream(line);
        try
        {
            stream >> x;
            stream >> y;
            stream >> name;
        } 
        catch (std::exception& e)
        {
            std::cout << "Exception: " << e.what() << std::endl;
        }
    }

    void parseEdge (std::string line, std::string& s, std::string& d, double& t, double& c)
    {
        std::stringstream stream(line);
        try
        {
            stream >> s;
            stream >> d;
            stream >> t;
            stream >> c;
        } 
        catch (std::exception& e)
        {
            std::cout << "Exception: " << e.what() << std::endl;
        }
    }
}