cmake_minimum_required (VERSION 2.6)
project (RoadNetworkApp)
add_executable(RoadNetworkApp 
  RoadNetwork/Graph.cpp
  RoadNetwork/Intersection.cpp
  RoadNetwork/RoadNetwork.cpp
  RoadNetwork/ShortestPathTree.cpp
  utility/readFile.cpp
  omp.test.cpp)