#include <gtest/gtest.h>

#include <vector>

#include "graph/algorithms/bellman_ford/bellman_ford.hpp"
#include "graph/structure/directed_adjacency_list.hpp"
#include "graph/structure/directed_edge_list.hpp"

using namespace graph;
using namespace graph::algorithms;

TEST(BellmanFordAdvancedTest, MultipleEqualPaths) {
  DirectedAdjacencyList<int, int> graph;
  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);

  graph.addEdge(1, 2, 1);
  graph.addEdge(1, 3, 1);
  graph.addEdge(2, 4, 1);
  graph.addEdge(3, 4, 1);
  graph.addEdge(4, 5, 1);

  BellmanFord<DirectedAdjacencyList<int, int>> bf(graph);
  bf.compute(1);

  EXPECT_EQ(bf.getDistance(5), 3);

  auto path = bf.retrievePath(5);
  EXPECT_EQ(path.size(), 4);
  EXPECT_EQ(path[0], 1);
  EXPECT_EQ(path[3], 5);
}

TEST(BellmanFordAdvancedTest, ComplexNegativeWeights) {
  DirectedEdgeList<std::string, int> graph;

  graph.addVertex("A").addVertex("B").addVertex("C").addVertex("D").addVertex(
      "E");
  graph.addEdge("A", "B", 6);
  graph.addEdge("A", "C", 7);
  graph.addEdge("B", "C", 8);
  graph.addEdge("B", "D", -4);
  graph.addEdge("B", "E", 5);
  graph.addEdge("C", "B", -2);
  graph.addEdge("C", "E", 9);
  graph.addEdge("D", "C", 7);
  graph.addEdge("D", "A", 2);
  graph.addEdge("E", "D", 7);
  graph.addEdge("E", "A", -3);

  BellmanFord<DirectedEdgeList<std::string, int>> bf(graph);
  bf.compute("A");

  EXPECT_FALSE(bf.hasNegativeCycle());

  EXPECT_EQ(bf.getDistance("A"), 0);
  EXPECT_EQ(bf.getDistance("B"), 5);
  EXPECT_EQ(bf.getDistance("C"), 7);
  EXPECT_EQ(bf.getDistance("D"), 1);
  EXPECT_EQ(bf.getDistance("E"), 10);

  std::vector<std::string> path_to_D = bf.retrievePath("D");
  std::vector<std::string> expected = {"A", "C", "B", "D"};
  EXPECT_EQ(path_to_D, expected);
}

TEST(BellmanFordAdvancedTest, MultipleNegativeCycles) {
  DirectedAdjacencyList<int, int> graph;

  graph.addVertex(1)
      .addVertex(2)
      .addVertex(3)
      .addVertex(4)
      .addVertex(5)
      .addVertex(6);

  graph.addEdge(1, 2, 1);
  graph.addEdge(2, 3, 1);
  graph.addEdge(3, 1, -3);

  graph.addEdge(4, 5, 2);
  graph.addEdge(5, 6, 2);
  graph.addEdge(6, 4, -5);

  graph.addEdge(1, 4, 10);

  BellmanFord<DirectedAdjacencyList<int, int>> bf(graph);
  bf.compute(1);

  EXPECT_TRUE(bf.hasNegativeCycle());
}

TEST(BellmanFordAdvancedTest, LargeGraph) {
  DirectedAdjacencyList<int, int> graph;

  const int SIZE = 1000;

  for (int i = 1; i <= SIZE; i++) {
    graph.addVertex(i);
  }

  for (int i = 1; i < SIZE; i++) {
    graph.addEdge(i, i + 1, 1);
  }

  for (int i = 5; i <= SIZE; i += 5) {
    graph.addEdge(i, i - 4, 2);
  }

  BellmanFord<DirectedAdjacencyList<int, int>> bf(graph);
  bf.compute(1);

  EXPECT_FALSE(bf.hasNegativeCycle());
  EXPECT_EQ(bf.getDistance(SIZE), SIZE - 1);
}

TEST(BellmanFordAdvancedTest, EdgeCases) {
  DirectedAdjacencyList<int, int> graph;

  graph.addVertex(1);

  BellmanFord<DirectedAdjacencyList<int, int>> bf1(graph);
  bf1.compute(1);

  EXPECT_EQ(bf1.getDistance(1), 0);
  EXPECT_FALSE(bf1.hasNegativeCycle());

  GraphOptions options(false, true);
  DirectedAdjacencyList<int, int> graph2(options);
  graph2.addVertex(1);
  graph2.addEdge(1, 1, 0);

  BellmanFord<DirectedAdjacencyList<int, int>> bf2(graph2);
  bf2.compute(1);

  EXPECT_EQ(bf2.getDistance(1), 0);
  EXPECT_FALSE(bf2.hasNegativeCycle());

  DirectedAdjacencyList<int, int> graph3(options);
  graph3.addVertex(1);
  graph3.addEdge(1, 1, -1);

  BellmanFord<DirectedAdjacencyList<int, int>> bf3(graph3);
  bf3.compute(1);

  EXPECT_TRUE(bf3.hasNegativeCycle());

  DirectedAdjacencyList<int, int> emptyGraph;

  BellmanFord<DirectedAdjacencyList<int, int>> bf4(emptyGraph);
  bf4.compute(1);

  EXPECT_FALSE(bf4.hasNegativeCycle());
}

TEST(BellmanFordAdvancedTest, NegativeCycleWithoutSelfLoop) {
  DirectedAdjacencyList<int, int> graph;

  graph.addVertex(1).addVertex(2).addVertex(3);

  graph.addEdge(1, 2, 1);
  graph.addEdge(2, 3, 1);
  graph.addEdge(3, 1, -3);

  BellmanFord<DirectedAdjacencyList<int, int>> bf(graph);
  bf.compute(1);

  EXPECT_TRUE(bf.hasNegativeCycle());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
