#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "graph/algorithms/dfs/components.hpp"
#include "graph/structure/undirected_adjacency_list.hpp"

using namespace graph;
using namespace graph::algorithms;

TEST(ConnectedComponentsTest, SingleComponent) {
  UndirectedAdjacencyList<int, double> graph;
  graph.addVertex(1).addVertex(2).addVertex(3);
  graph.addEdge(1, 2, 1.0).addEdge(2, 3, 1.0);

  ConnectedComponents<UndirectedAdjacencyList<int, double>> cc(graph);
  auto component = cc.findOne(1);

  EXPECT_EQ(component.size(), 3);
  EXPECT_EQ(component[0], 1);
  EXPECT_EQ(component[1], 2);
  EXPECT_EQ(component[2], 3);
}

TEST(ConnectedComponentsTest, MultipleComponents) {
  UndirectedAdjacencyList<int, double> graph;
  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);
  graph.addEdge(1, 2, 1.0).addEdge(1, 3, 1.0);
  graph.addEdge(4, 5, 1.0);

  ConnectedComponents<UndirectedAdjacencyList<int, double>> cc(graph);
  auto components = cc.findAll();

  EXPECT_EQ(components.size(), 2);
}
