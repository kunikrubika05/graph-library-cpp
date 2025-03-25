#include <gtest/gtest.h>

#include <algorithm>
#include <set>
#include <utility>
#include <vector>

#include "graph/algorithms/dfs/bridges.hpp"
#include "graph/structure/undirected_adjacency_list.hpp"

using namespace graph;
using namespace graph::algorithms;

template <typename T>
bool containsEdge(const std::vector<std::pair<T, T>> &bridges, T v1, T v2) {
  return std::find_if(bridges.begin(), bridges.end(),
                      [v1, v2](const auto &edge) {
                        return (edge.first == v1 && edge.second == v2) ||
                               (edge.first == v2 && edge.second == v1);
                      }) != bridges.end();
}

TEST(BridgesTest, EmptyGraph) {
  UndirectedAdjacencyList<int, double> graph;

  Bridges<UndirectedAdjacencyList<int, double>> bridge_finder(graph);
  bridge_finder.findBridges();

  EXPECT_TRUE(bridge_finder.getBridges().empty());
}

TEST(BridgesTest, SingleVertex) {
  UndirectedAdjacencyList<int, double> graph;
  graph.addVertex(1);

  Bridges<UndirectedAdjacencyList<int, double>> bridge_finder(graph);
  bridge_finder.findBridges();

  EXPECT_TRUE(bridge_finder.getBridges().empty());
}

TEST(BridgesTest, SingleEdge) {
  UndirectedAdjacencyList<int, double> graph;
  graph.addVertex(1).addVertex(2);
  graph.addEdge(1, 2, 1.0);

  Bridges<UndirectedAdjacencyList<int, double>> bridge_finder(graph);
  bridge_finder.findBridges();

  auto bridges = bridge_finder.getBridges();
  EXPECT_EQ(bridges.size(), 1);
  EXPECT_TRUE(containsEdge(bridges, 1, 2));
}

TEST(BridgesTest, CycleNoBridges) {
  UndirectedAdjacencyList<int, double> graph;
  graph.addVertex(1).addVertex(2).addVertex(3);
  graph.addEdge(1, 2, 1.0).addEdge(2, 3, 1.0).addEdge(3, 1, 1.0);

  Bridges<UndirectedAdjacencyList<int, double>> bridge_finder(graph);
  bridge_finder.findBridges();

  EXPECT_TRUE(bridge_finder.getBridges().empty());
}

TEST(BridgesTest, ComplexGraphWithBridges) {
  UndirectedAdjacencyList<int, double> graph;
  for (int i = 1; i <= 7; ++i) {
    graph.addVertex(i);
  }

  graph.addEdge(1, 2, 1.0).addEdge(2, 3, 1.0).addEdge(3, 1, 1.0);

  graph.addEdge(4, 5, 1.0).addEdge(5, 6, 1.0).addEdge(6, 4, 1.0);
  graph.addEdge(5, 7, 1.0);  // Мост к листу

  graph.addEdge(2, 4, 1.0);

  Bridges<UndirectedAdjacencyList<int, double>> bridge_finder(graph);
  bridge_finder.findBridges();

  auto bridges = bridge_finder.getBridges();
  EXPECT_EQ(bridges.size(), 2);
  EXPECT_TRUE(containsEdge(bridges, 2, 4));
  EXPECT_TRUE(containsEdge(bridges, 5, 7));
}

TEST(BridgesTest, TreeAllEdgesAreBridges) {
  UndirectedAdjacencyList<int, double> graph;

  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);
  graph.addEdge(1, 2, 1.0).addEdge(1, 3, 1.0).addEdge(3, 4, 1.0).addEdge(3, 5,
                                                                         1.0);

  Bridges<UndirectedAdjacencyList<int, double>> bridge_finder(graph);
  bridge_finder.findBridges();

  auto bridges = bridge_finder.getBridges();
  EXPECT_EQ(bridges.size(), 4);
  EXPECT_TRUE(containsEdge(bridges, 1, 2));
  EXPECT_TRUE(containsEdge(bridges, 1, 3));
  EXPECT_TRUE(containsEdge(bridges, 3, 4));
  EXPECT_TRUE(containsEdge(bridges, 3, 5));
}
