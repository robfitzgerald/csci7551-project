cmake_minimum_required (VERSION 2.6)
project (RoadNetworkApp)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
add_executable(RoadNetworkApp 
  RoadNetwork/Graph.cpp
  RoadNetwork/Intersection.cpp
  RoadNetwork/RoadNetwork.cpp
  RoadNetwork/BidirectionalAStar.cpp
  utility/readFile.cpp
  msaSim.test.cpp)