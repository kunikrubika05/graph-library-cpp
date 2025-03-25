#include <gtest/gtest.h>

#include <set>
#include <vector>

#include "graph/algorithms/dfs/topological_sort.hpp"
#include "graph/structure/directed_adjacency_list.hpp"

using namespace graph;
using namespace graph::algorithms;

TEST(TopologicalSortTest, SimpleDAG) {
  DirectedAdjacencyList<int, double> graph;
  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);
  graph.addEdge(1, 2, 1.0)
      .addEdge(1, 3, 1.0)
      .addEdge(2, 4, 1.0)
      .addEdge(3, 4, 1.0)
      .addEdge(4, 5, 1.0);

  TopologicalSort<DirectedAdjacencyList<int, double>> sorter(graph);
  EXPECT_TRUE(sorter.sort());

  auto sorted = sorter.getSortedVertices();

  EXPECT_EQ(sorted.size(), 5);

  std::unordered_map<int, int> positions;
  for (size_t i = 0; i < sorted.size(); ++i) {
    positions[sorted[i]] = i;
  }

  EXPECT_LT(positions[1], positions[2]);
  EXPECT_LT(positions[1], positions[3]);

  EXPECT_LT(positions[2], positions[4]);
  EXPECT_LT(positions[3], positions[4]);

  EXPECT_LT(positions[4], positions[5]);
}

TEST(TopologicalSortTest, CyclicGraph) {
  DirectedAdjacencyList<int, double> graph;
  graph.addVertex(1).addVertex(2).addVertex(3);
  graph.addEdge(1, 2, 1.0).addEdge(2, 3, 1.0).addEdge(3, 1, 1.0);

  TopologicalSort<DirectedAdjacencyList<int, double>> sorter(graph);
  EXPECT_FALSE(sorter.sort());
}

TEST(TopologicalSortTest, EmptyGraph) {
  DirectedAdjacencyList<int, double> graph;

  TopologicalSort<DirectedAdjacencyList<int, double>> sorter(graph);
  EXPECT_TRUE(sorter.sort());
  EXPECT_TRUE(sorter.getSortedVertices().empty());
}

TEST(TopologicalSortTest, SingleVertex) {
  DirectedAdjacencyList<int, double> graph;
  graph.addVertex(1);

  TopologicalSort<DirectedAdjacencyList<int, double>> sorter(graph);
  EXPECT_TRUE(sorter.sort());

  auto sorted = sorter.getSortedVertices();
  EXPECT_EQ(sorted.size(), 1);
  EXPECT_EQ(sorted[0], 1);
}

TEST(TopologicalSortTest, DisconnectedDAG) {
  DirectedAdjacencyList<int, double> graph;
  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);

  graph.addEdge(1, 2, 1.0);
  graph.addEdge(3, 4, 1.0).addEdge(4, 5, 1.0);

  TopologicalSort<DirectedAdjacencyList<int, double>> sorter(graph);
  EXPECT_TRUE(sorter.sort());

  auto sorted = sorter.getSortedVertices();
  EXPECT_EQ(sorted.size(), 5);

  std::set<int> vertices(sorted.begin(), sorted.end());
  EXPECT_EQ(vertices.size(), 5);

  std::unordered_map<int, int> positions;
  for (size_t i = 0; i < sorted.size(); ++i) {
    positions[sorted[i]] = i;
  }

  EXPECT_LT(positions[1], positions[2]);
  EXPECT_LT(positions[3], positions[4]);
  EXPECT_LT(positions[4], positions[5]);
}
