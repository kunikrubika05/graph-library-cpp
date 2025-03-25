#include <gtest/gtest.h>

#include <vector>

#include "graph/algorithms/bellman_ford/bellman_ford.hpp"
#include "graph/structure/directed_adjacency_list.hpp"
#include "graph/structure/directed_adjacency_matrix.hpp"

using namespace graph;
using namespace graph::algorithms;

TEST(BellmanFordTest, SimpleGraph) {
  DirectedAdjacencyList<int, int> graph;

  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);
  graph.addEdge(1, 2, 2).addEdge(1, 4, 5);
  graph.addEdge(2, 3, 1).addEdge(2, 5, 3);
  graph.addEdge(4, 5, 1);

  BellmanFord<DirectedAdjacencyList<int, int>> bf(graph);
  bf.compute(1);

  EXPECT_EQ(bf.getDistance(1), 0);
  EXPECT_EQ(bf.getDistance(2), 2);
  EXPECT_EQ(bf.getDistance(3), 3);
  EXPECT_EQ(bf.getDistance(4), 5);
  EXPECT_EQ(bf.getDistance(5), 5);

  std::vector<int> path_to_3 = bf.retrievePath(3);
  std::vector<int> expected_path_to_3 = {1, 2, 3};
  EXPECT_EQ(path_to_3, expected_path_to_3);

  std::vector<int> path_to_5 = bf.retrievePath(5);
  std::vector<int> expected_path_to_5_1 = {1, 2, 5};
  std::vector<int> expected_path_to_5_2 = {1, 4, 5};
  EXPECT_TRUE(path_to_5 == expected_path_to_5_1 ||
              path_to_5 == expected_path_to_5_2);
}

TEST(BellmanFordTest, UnreachableVertex) {
  DirectedAdjacencyList<int, int> graph;
  graph.addVertex(1).addVertex(2).addVertex(3);
  graph.addEdge(1, 2, 1);

  BellmanFord<DirectedAdjacencyList<int, int>> bf(graph);
  bf.compute(1);

  EXPECT_THROW(bf.retrievePath(3), std::runtime_error);
  EXPECT_THROW(bf.getDistance(3), std::runtime_error);
}

TEST(BellmanFordTest, NegativeWeights) {
  DirectedAdjacencyList<int, int> graph;
  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, 4);
  graph.addEdge(1, 3, 3);
  graph.addEdge(2, 3, -2);
  graph.addEdge(3, 4, 2);

  BellmanFord<DirectedAdjacencyList<int, int>> bf(graph);
  bf.compute(1);

  EXPECT_EQ(bf.getDistance(1), 0);
  EXPECT_EQ(bf.getDistance(2), 4);
  EXPECT_EQ(bf.getDistance(3), 2);
  EXPECT_EQ(bf.getDistance(4), 4);

  std::vector<int> path_to_3 = bf.retrievePath(3);
  std::vector<int> expected = {1, 2, 3};
  EXPECT_EQ(path_to_3, expected);

  EXPECT_FALSE(bf.hasNegativeCycle());
}

TEST(BellmanFordMatrixTest, SimpleGraph) {
  DirectedAdjacencyMatrix<int, int> graph(3);
  graph.addVertex(1).addVertex(2).addVertex(3);

  graph.addEdge(1, 2, 2);
  graph.addEdge(1, 3, 5);
  graph.addEdge(2, 3, 1);

  BellmanFord<DirectedAdjacencyMatrix<int, int>> bf(graph);
  bf.compute(1);

  EXPECT_EQ(bf.getDistance(1), 0);
  EXPECT_EQ(bf.getDistance(2), 2);
  EXPECT_EQ(bf.getDistance(3), 3);
}

TEST(BellmanFordTest, NegativeCycleDetection) {
  DirectedAdjacencyList<int, int> graph;
  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, 1);
  graph.addEdge(2, 3, 2);
  graph.addEdge(3, 4, 3);
  graph.addEdge(4, 2, -7);

  BellmanFord<DirectedAdjacencyList<int, int>> bf(graph);
  bf.compute(1);

  EXPECT_TRUE(bf.hasNegativeCycle());
  EXPECT_THROW(bf.getDistance(4), std::runtime_error);
}

TEST(BellmanFordTest, NegativeCyclePathRetrieval) {
  DirectedAdjacencyList<int, int> graph;
  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, 1);
  graph.addEdge(2, 3, 2);
  graph.addEdge(3, 4, 3);
  graph.addEdge(4, 2, -7);

  BellmanFord<DirectedAdjacencyList<int, int>> bf(graph);
  bf.compute(1);

  EXPECT_THROW(bf.retrievePath(4), std::runtime_error);
}

TEST(BellmanFordTest, EmptyGraph) {
  DirectedAdjacencyList<int, int> graph;

  BellmanFord<DirectedAdjacencyList<int, int>> bf(graph);
  bf.compute(1);

  EXPECT_FALSE(bf.hasNegativeCycle());
}

TEST(BellmanFordTest, DisconnectedGraph) {
  DirectedAdjacencyList<int, int> graph;
  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, 5);

  BellmanFord<DirectedAdjacencyList<int, int>> bf(graph);
  bf.compute(1);

  EXPECT_EQ(bf.getDistance(1), 0);
  EXPECT_EQ(bf.getDistance(2), 5);
  EXPECT_THROW(bf.getDistance(3), std::runtime_error);
  EXPECT_THROW(bf.getDistance(4), std::runtime_error);
}

TEST(BellmanFordTest, ComplexNegativeGraph) {
  DirectedAdjacencyList<int, int> graph;

  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);
  graph.addEdge(1, 2, 6);
  graph.addEdge(1, 3, 7);
  graph.addEdge(2, 3, 8);
  graph.addEdge(2, 4, -4);
  graph.addEdge(2, 5, 5);
  graph.addEdge(3, 2, -2);
  graph.addEdge(3, 5, 9);
  graph.addEdge(4, 3, 7);
  graph.addEdge(4, 1, 2);
  graph.addEdge(5, 4, 7);
  graph.addEdge(5, 1, -3);

  BellmanFord<DirectedAdjacencyList<int, int>> bf(graph);
  bf.compute(1);

  EXPECT_FALSE(bf.hasNegativeCycle());

  EXPECT_EQ(bf.getDistance(1), 0);
  EXPECT_EQ(bf.getDistance(2), 5);
  EXPECT_EQ(bf.getDistance(3), 7);
  EXPECT_EQ(bf.getDistance(4), 1);
  EXPECT_EQ(bf.getDistance(5), 10);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
