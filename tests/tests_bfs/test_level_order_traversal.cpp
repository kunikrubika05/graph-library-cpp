#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "graph/algorithms/bfs/level_order_traversal.hpp"
#include "graph/structure/undirected_adjacency_list.hpp"

using namespace graph;
using namespace graph::algorithms;

TEST(LevelOrderTraversalTest, OrderAndDistanceTest) {
  UndirectedAdjacencyList<int, int> graph;
  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, 1).addEdge(1, 3, 1).addEdge(2, 4, 1).addEdge(3, 4, 1);

  LevelOrderTraversal<UndirectedAdjacencyList<int, int>> lot(graph);
  std::vector<int> visitOrder;
  lot.traverse(
      1, [](int v) {}, [&visitOrder](int v) { visitOrder.push_back(v); });

  EXPECT_EQ(visitOrder.front(), 1);

  const auto &dist = lot.getDistance();
  EXPECT_EQ(dist.at(1), 0);

  EXPECT_TRUE(dist.at(2) == 1 || dist.at(3) == 1);

  EXPECT_EQ(dist.at(4), 2);

  const auto &order = lot.getOrder();
  EXPECT_EQ(order, visitOrder);
}

TEST(LevelOrderTraversalTest, RetrievePathTest) {
  UndirectedAdjacencyList<int, int> graph;
  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, 1).addEdge(2, 3, 1).addEdge(2, 4, 1);

  LevelOrderTraversal<UndirectedAdjacencyList<int, int>> lot(graph);
  lot.traverse(1);

  std::vector<int> path = lot.retrievePath(3);
  std::vector<int> expected1 = {1, 2, 3};
  std::vector<int> expected2 = {1, 2, 4, 3};

  EXPECT_EQ(path, expected1);

  graph.addVertex(5);
  EXPECT_THROW(lot.retrievePath(5), std::runtime_error);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
