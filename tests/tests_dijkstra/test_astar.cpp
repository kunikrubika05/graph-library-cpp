#include <gtest/gtest.h>

#include <vector>

#include "graph/algorithms/dijkstra/astar.hpp"
#include "graph/structure/directed_adjacency_list.hpp"

using namespace graph;
using namespace graph::algorithms;

TEST(AStarTest, SimpleGraphZeroHeuristic) {
  graph::DirectedAdjacencyList<int, int> graph;
  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, 1).addEdge(2, 3, 1).addEdge(1, 3, 3).addEdge(3, 4, 1);

  auto heuristic = [](int u, int target) -> int { return 0; };

  AStar<graph::DirectedAdjacencyList<int, int>> astar(graph, heuristic);
  astar.compute(1, 4);

  EXPECT_EQ(astar.getDistance(4), 3);
  std::vector<int> path = astar.retrievePath(4);
  std::vector<int> expectedPath = {1, 2, 3, 4};
  EXPECT_EQ(path, expectedPath);
}

TEST(AStarTest, NoPath) {
  graph::DirectedAdjacencyList<int, int> graph;
  graph.addVertex(1).addVertex(2).addVertex(3);
  graph.addEdge(1, 2, 1);

  auto heuristic = [](int, int) -> int { return 0; };

  AStar<graph::DirectedAdjacencyList<int, int>> astar(graph, heuristic);
  astar.compute(1, 3);
  EXPECT_THROW(astar.retrievePath(3), std::runtime_error);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
