cmake_minimum_required (VERSION 2.6)
project (RoadNetworkApp)
add_executable(RoadNetworkApp 
  RoadNetwork/Graph.cpp
  RoadNetwork/Intersection.cpp
  RoadNetwork/RoadNetwork.cpp
  RoadNetwork/BidirectionalAStar.cpp
  utility/readFile.cpp
  msaSim.test.cpp)