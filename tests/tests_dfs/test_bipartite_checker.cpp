#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "graph/algorithms/dfs/bipartite_checker.hpp"
#include "graph/structure/undirected_adjacency_list.hpp"

using namespace graph;
using namespace graph::algorithms;

TEST(BipartiteCheckerTest, SimpleBipartiteGraph) {
  UndirectedAdjacencyList<int, double> graph;
  graph.addVertex(1).addVertex(2).addVertex(3);
  graph.addEdge(1, 2, 1.0).addEdge(2, 3, 1.0);

  BipartiteChecker<UndirectedAdjacencyList<int, double>> checker(graph);

  EXPECT_TRUE(checker.isBipartite());

  auto partition = checker.getPartition();
  EXPECT_EQ(partition.first.size() + partition.second.size(), 3);

  std::vector<int> part1 = partition.first;
  std::vector<int> part2 = partition.second;

  bool in_part1_1_3 =
      (std::find(part1.begin(), part1.end(), 1) != part1.end() &&
       std::find(part1.begin(), part1.end(), 3) != part1.end());
  bool in_part2_1_3 =
      (std::find(part2.begin(), part2.end(), 1) != part2.end() &&
       std::find(part2.begin(), part2.end(), 3) != part2.end());

  EXPECT_TRUE(in_part1_1_3 || in_part2_1_3);
}

TEST(BipartiteCheckerTest, NonBipartiteGraph) {
  UndirectedAdjacencyList<int, double> graph;
  graph.addVertex(1).addVertex(2).addVertex(3);
  graph.addEdge(1, 2, 1.0).addEdge(2, 3, 1.0).addEdge(3, 1, 1.0);

  BipartiteChecker<UndirectedAdjacencyList<int, double>> checker(graph);

  EXPECT_FALSE(checker.isBipartite());

  auto partition = checker.getPartition();
  EXPECT_TRUE(partition.first.empty() && partition.second.empty());
}

TEST(BipartiteCheckerTest, MultipleComponentsBipartite) {
  UndirectedAdjacencyList<int, double> graph;
  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);
  graph.addEdge(1, 2, 1.0).addEdge(2, 3, 1.0);
  graph.addEdge(4, 5, 1.0);

  BipartiteChecker<UndirectedAdjacencyList<int, double>> checker(graph);

  EXPECT_TRUE(checker.isBipartite());

  auto partition = checker.getPartition();
  EXPECT_EQ(partition.first.size() + partition.second.size(), 5);
}

TEST(BipartiteCheckerTest, MultipleComponentsNonBipartite) {
  UndirectedAdjacencyList<int, double> graph;
  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);
  graph.addEdge(1, 2, 1.0).addEdge(2, 3, 1.0).addEdge(3, 1, 1.0);
  graph.addEdge(4, 5, 1.0);

  BipartiteChecker<UndirectedAdjacencyList<int, double>> checker(graph);

  EXPECT_FALSE(checker.isBipartite());
}

TEST(BipartiteCheckerTest, EmptyGraph) {
  UndirectedAdjacencyList<int, double> graph;

  BipartiteChecker<UndirectedAdjacencyList<int, double>> checker(graph);

  EXPECT_TRUE(checker.isBipartite());

  auto partition = checker.getPartition();
  EXPECT_TRUE(partition.first.empty() && partition.second.empty());
}

TEST(BipartiteCheckerTest, SingleVertexGraph) {
  UndirectedAdjacencyList<int, double> graph;
  graph.addVertex(1);

  BipartiteChecker<UndirectedAdjacencyList<int, double>> checker(graph);

  EXPECT_TRUE(checker.isBipartite());

  auto partition = checker.getPartition();
  EXPECT_EQ(partition.first.size(), 1);
  EXPECT_TRUE(partition.second.empty());
}
