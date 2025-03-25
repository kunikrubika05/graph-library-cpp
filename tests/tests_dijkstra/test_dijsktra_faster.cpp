#include <gtest/gtest.h>

#include <vector>

#include "graph/algorithms/dijkstra/dijkstra_faster.hpp"
#include "graph/structure/directed_adjacency_list.hpp"
#include "graph/structure/directed_adjacency_matrix.hpp"

TEST(OptimizedDijkstraTest, SimpleGraphWithTargetEarlyExit) {
  graph::DirectedAdjacencyList<int, int> graph;
  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);
  graph.addEdge(1, 2, 2).addEdge(1, 4, 5);
  graph.addEdge(2, 3, 1).addEdge(2, 5, 3);
  graph.addEdge(4, 5, 1);

  int target = 5;
  graph::algorithms::OptimizedDijkstra<graph::DirectedAdjacencyList<int, int>>
      dijkstra(graph);
  dijkstra.compute(1, &target);

  EXPECT_EQ(dijkstra.getDistance(1), 0);
  EXPECT_EQ(dijkstra.getDistance(2), 2);
  EXPECT_EQ(dijkstra.getDistance(5), 5);

  std::vector<int> path = dijkstra.retrievePath(5);
  std::vector<int> expected1 = {1, 2, 5};
  std::vector<int> expected2 = {1, 4, 5};
  EXPECT_TRUE(path == expected1 || path == expected2);
}

TEST(OptimizedDijkstraTest, UnreachableVertex) {
  graph::DirectedAdjacencyMatrix<int, int> graph(3);
  graph.addVertex(1).addVertex(2).addVertex(3);
  graph.addEdge(1, 2, 1);
  graph::algorithms::OptimizedDijkstra<graph::DirectedAdjacencyMatrix<int, int>>
      dijkstra(graph);
  dijkstra.compute(1);
  EXPECT_THROW(dijkstra.getDistance(3), std::runtime_error);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
