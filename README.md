# csci7551-project
## Simulation of Static Traffic Assignment

this project is a simulation of the textbook techniques for assigning vehicle traffic to a road network for a static snapshot of the expected distribution of congestion on the system.  it is based on methods described in Chapter 10 of [Modelling Transport](http://onlinelibrary.wiley.com/book/10.1002/9781119993308) by Juan de Dios Ortúzar and Luis G. Willumsen.

### installation (CSEGrid Hydra)

1. clone repo: `$ git clone https://github.com/robfitzgerald/csci7551-project.git`
2. enter directory: `$ cd csci7551-project/`
3. run cmake: `$ cmake CMakeLists.txt`
4. run make: `$ make`
5. run the application: `bpsh <n> ./RoadNetworkApp` (n | your favorite node here)

#### sample installation

```
[robert.fitzgerald@hydra csci7551]$ git clone https://github.com/robfitzgerald/csci7551-project.git
Initialized empty Git repository in /home/robert.fitzgerald/csci7551/csci7551-project/.git/
remote: Counting objects: 270, done.
remote: Compressing objects: 100% (69/69), done.
remote: Total 270 (delta 49), reused 0 (delta 0), pack-reused 201
Receiving objects: 100% (270/270), 132.68 KiB, done.
Resolving deltas: 100% (149/149), done.
[robert.fitzgerald@hydra csci7551]$ cd csci7551-project/
[robert.fitzgerald@hydra csci7551-project]$ ls
auraria-small.osm  CMakeLists.txt  GenerateODPairs  msaSim.test.cpp  omp.test.cpp  readFile.test.cpp  RoadNetwork  three_road_network.graph  utility
[robert.fitzgerald@hydra csci7551-project]$ cmake CMakeLists.txt 
-- The C compiler identification is GNU 4.4.7
-- The CXX compiler identification is GNU 4.4.7
-- Check for working C compiler: /usr/bin/cc
-- Check for working C compiler: /usr/bin/cc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Try OpenMP C flag = [-fopenmp]
-- Performing Test OpenMP_FLAG_DETECTED
-- Performing Test OpenMP_FLAG_DETECTED - Success
-- Try OpenMP CXX flag = [-fopenmp]
-- Performing Test OpenMP_FLAG_DETECTED
-- Performing Test OpenMP_FLAG_DETECTED - Success
-- Found OpenMP: -fopenmp  
-- Configuring done
-- Generating done
-- Build files have been written to: /home/robert.fitzgerald/csci7551/csci7551-project
[robert.fitzgerald@hydra csci7551-project]$ make
Scanning dependencies of target RoadNetworkApp
[ 16%] Building CXX object CMakeFiles/RoadNetworkApp.dir/RoadNetwork/Graph.cpp.o
[ 33%] Building CXX object CMakeFiles/RoadNetworkApp.dir/RoadNetwork/Intersection.cpp.o
[ 50%] Building CXX object CMakeFiles/RoadNetworkApp.dir/RoadNetwork/RoadNetwork.cpp.o
[ 66%] Building CXX object CMakeFiles/RoadNetworkApp.dir/RoadNetwork/BidirectionalAStar.cpp.o
[ 83%] Building CXX object CMakeFiles/RoadNetworkApp.dir/utility/readFile.cpp.o
[100%] Building CXX object CMakeFiles/RoadNetworkApp.dir/msaSim.test.cpp.o
Linking CXX executable RoadNetworkApp
[100%] Built target RoadNetworkApp
[robert.fitzgerald@hydra csci7551-project]$ bpsh 9 ./RoadNetworkApp

```

### classes
```
RoadNetwork/Graph.h
  class Vertex               // base graph vertex class
  class Edge                 // base graph edge class
  class VertexProperty       // base vertex properties class
RoadNetwork/Intersection.h
  struct NodeCostTuple       // used by search to hold associate the cost of a path choice
  class Intersection         // derived Vertex class with Roadway relationships and euclidian properties
RoadNetwork/IntersectionProperty.h
  class IntersectionProperty // derived VertexProperty class that has lat/long position and a name
RoadNetwork/Roadway.h
  class Roadway              // derived Edge class with properties for determining the cost and flow of a road
RoadNetwork/BidirectionalAStar.h
  struct FrontierCost        // tuple for producing data on each frontier node for the compareCosts operations
  class AStarNode            // node class that wraps an Intersection class and stores the path to that node in the current search instance along with the cumulative cost to traverse to that node
  class BidirectionalAStar   // class that tracks the list of selected and frontier nodes of a search and provides operations to step forward the stages of the bidirectional a* search algorithm
RoadNetwork/RoadNetwork.h
  struct Path                // two intersections and a list of roadways between them
  struct ODPair              // future deprecated test class for calling a search, to be replaced with Path
  class RoadNetwork          // Graph class for Intersections and Roadways, set with a simulation algorithm, and exposing a simulate method to run simulation on the network using a set of origin/destination pairs
utility/readFile.h
  readFile()                 // parses .graph-type files into a RoadNetwork
GenerateODPairs/GenerateODPairs.scala
  GenerateODPairs            // utility application to produce random OD pairs from a list of origin nodes and destination nodes described in the .graph file (not yet described)
```

### a few choice words before you run this

this application is still incomplete. in it's current example, it loads the graph as described in the file `three_road_network.graph`, which has three different routes between the node 'A' and node 'O'.

the driver file, msaSim.test.cpp declares one trip for 50 vehicles per time unit from node "A" to node "O".

all functions are currently logging like it's going out of style. hopefully the logs can show you the iterative search process is _trying_ to happen. i sometimes find that it terminates with a child process error, and sometimes it hangs, and sometimes it terminates with a seg fault.

some terminologies in the logging:  the "selected" and "frontier" lists for the "forward" and "backward" search are exactly what they sound like. you will see a printout of both lists for both directions at the end of the search iteration. you _should_ see a single node added to the selected list each iteration for each sub-search. it _should_ be the one with minimal cost.

### example run

```
----- top -----
Intersection euclidianDistance(): A -> B | (0,5) - 5
Intersection euclidianDistance(): B -> O | (0,5) - 5
Intersection euclidianDistance(): A -> D | (5,5) - 7.07107
Intersection euclidianDistance(): D -> E | (0,5) - 5
Intersection euclidianDistance(): E -> O | (5,10) - 11.1803
Intersection euclidianDistance(): F -> G | (5,5) - 7.07107
Intersection euclidianDistance(): G -> O | (10,5) - 11.1803
Intersection euclidianDistance(): A -> F | (5,0) - 5
----- file read complete -----
----- about to toString -----
----- Graph State -----
-= Vertices =----------
[A] (0,0): 0 in, 3 out.
[B] (0,5): 1 in, 1 out.
[D] (-5,5): 1 in, 1 out.
[E] (-5,0): 1 in, 1 out.
[F] (5,0): 1 in, 1 out.
[G] (10,5): 1 in, 1 out.
[O] (0,10): 3 in, 0 out.
-= Edges =-------------
(A)-->(B) distance: 5, cost: 40
(B)-->(O) distance: 5, cost: 40
(A)-->(D) distance: 7.07107, cost: 20
(D)-->(E) distance: 5, cost: 20
(E)-->(O) distance: 11.1803, cost: 20
(F)-->(G) distance: 7.07107, cost: 10
(G)-->(O) distance: 11.1803, cost: 10
(A)-->(F) distance: 5, cost: 10

starting RoadNetwork::assignMSA
assignMSA iteration 1, phi 1
forward updateFrontier, found parent: A
forward updateFrontier, found child: B
forward updateFrontier, found child: D
forward updateFrontier, found child: F
backward updateFrontier, found parent: O
backward updateFrontier, found child: B
backward updateFrontier, found child: E
backward updateFrontier, found child: G
forward  master - critical section
meeting point: 0 == 0: not found
meetingPointFound: 0 searchSpaceExhausted: 0
forward  master - selecting best frontier node and then clearing lists
compareLists result, choose F G with heuristic 27.0711
forward  master - selecting best frontier complete
forward  master - best frontier buckets have sizes 1 and 1
I have my pointers
forward  master - chose: F and G
F
forward updateFrontier, found parent: F
forward updateFrontier, found child: G
forward  master|slave - printing current search state
----- Printing Lists in this search -----
-- Frontier Nodes --
B, distance: 5, cost: 40
D, distance: 7.07107, cost: 20
G, distance: 12.0711, cost: 20
-- Selected Nodes --
A, distance: 0, cost: 0
F, distance: 5, cost: 10
G
backward updateFrontier, found parent: G
backward updateFrontier, found child: F
backward  master|slave - printing current search state
----- Printing Lists in this search -----
-- Frontier Nodes --
B, distance: 5, cost: 40
E, distance: 11.1803, cost: 20
F, distance: 18.2514, cost: 20
-- Selected Nodes --
G, distance: 11.1803, cost: 10
O, distance: 0, cost: 0
G
backward  master|slave - printing current search state
----- Printing Lists in this search -----
-- Frontier Nodes --
B, distance: 5, cost: 40
E, distance: 11.1803, cost: 20
F, distance: 18.2514, cost: 20
-- Selected Nodes --
G, distance: 11.1803, cost: 10
O, distance: 0, cost: 0
forward  master - meeting point not found, continuing search
forward  master - clearing the buckets
forward  master - critical section
meeting point: 0 == 0: not found
meetingPointFound: 0 searchSpaceExhausted: 0
forward  master - selecting best frontier node and then clearing lists
compareLists result, choose D E with heuristic 45
forward  master - selecting best frontier complete
forward  master - best frontier buckets have sizes 1 and 1
I have my pointers
forward  master - chose: D and E
D
forward updateFrontier, found parent: D
forward updateFrontier, found child: E
forward  master|slave - printing current search state
----- Printing Lists in this search -----
-- Frontier Nodes --
B, distance: 5, cost: 40
E, distance: 12.0711, cost: 40
G, distance: 12.0711, cost: 20
-- Selected Nodes --
A, distance: 0, cost: 0
D, distance: 7.07107, cost: 20
F, distance: 5, cost: 10
E
backward updateFrontier, found parent: E
backward updateFrontier, found child: D
backward  master|slave - printing current search state
----- Printing Lists in this search -----
-- Frontier Nodes --
B, distance: 5, cost: 40
D, distance: 16.1803, cost: 40
F, distance: 18.2514, cost: 20
-- Selected Nodes --
E, distance: 11.1803, cost: 20
G, distance: 11.1803, cost: 10
O, distance: 0, cost: 0
forward  master - meeting point not found, continuing search
forward  master - clearing the buckets
forward  master - critical section
meeting point: 0 == 0: not found
meetingPointFound: 0 searchSpaceExhausted: 0
forward  master - selecting best frontier node and then clearing lists
compareLists result, choose G F with heuristic 47.0711
forward  master - selecting best frontier complete
forward  master - best frontier buckets have sizes 1 and 1
I have my pointers
forward  master - chose: G and F
G
forward updateFrontier, found parent: G
forward updateFrontier, found child: O
forward  master|slave - printing current search state
----- Printing Lists in this search -----
-- Frontier Nodes --
B, distance: 5, cost: 40
E, distance: 12.0711, cost: 40
O, distance: 23.2514, cost: 30
-- Selected Nodes --
A, distance: 0, cost: 0
D, distance: 7.07107, cost: 20
F, distance: 5, cost: 10
G, distance: 12.0711, cost: 20
F
backward updateFrontier, found parent: F
backward updateFrontier, found child: A
backward  master|slave - printing current search state
----- Printing Lists in this search -----
-- Frontier Nodes --
A, distance: 23.2514, cost: 30
B, distance: 5, cost: 40
D, distance: 16.1803, cost: 40
-- Selected Nodes --
E, distance: 11.1803, cost: 20
F, distance: 18.2514, cost: 20
G, distance: 11.1803, cost: 10
O, distance: 0, cost: 0
forward  master - meeting point not found, continuing search
forward  master - clearing the buckets
forward  master - critical section
meeting point: 0x8427d0 == 0x8427d0: F
meetingPointFound: 1 searchSpaceExhausted: 0
forward  master - stopping condition or search space exhausted
forward  master - meeting point found, merging
forward  master - merge the searches
merging two paths into one
shortest path found: G -> O -> G -> F
forward  master - merge completed
forward  master - all shared memory for this job pair was cleared
backward  master|slave - end of shortestPath
done with job 1
^C
```
