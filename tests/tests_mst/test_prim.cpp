#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "graph/algorithms/mst/prim.hpp"
#include "graph/structure/undirected_adjacency_list.hpp"
#include "graph/structure/undirected_edge_list.hpp"

using namespace graph;
using namespace graph::algorithms;

TEST(PrimTest, SimpleGraph) {
  UndirectedAdjacencyList<int, int> graph;

  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, 1);
  graph.addEdge(2, 3, 2);
  graph.addEdge(3, 4, 3);
  graph.addEdge(4, 1, 4);
  graph.addEdge(2, 4, 5);

  Prim<UndirectedAdjacencyList<int, int>> prim(graph);
  prim.compute(1);

  EXPECT_TRUE(prim.isConnected());
  EXPECT_EQ(prim.getTotalWeight(), 6);

  auto mst_edges = prim.getMSTEdges();
  EXPECT_EQ(mst_edges.size(), 3);

  int total_weight = 0;
  for (const auto &edge : mst_edges) {
    total_weight += edge.weight;
  }
  EXPECT_EQ(total_weight, 6);
}

TEST(PrimTest, DisconnectedGraph) {
  UndirectedAdjacencyList<int, int> graph;

  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, 1);
  graph.addEdge(3, 4, 2);

  Prim<UndirectedAdjacencyList<int, int>> prim(graph);
  prim.compute();

  EXPECT_FALSE(prim.isConnected());
  EXPECT_THROW(prim.getTotalWeight(), std::runtime_error);
  EXPECT_THROW(prim.getMSTEdges(), std::runtime_error);
}

TEST(PrimTest, EmptyGraph) {
  UndirectedAdjacencyList<int, int> graph;

  Prim<UndirectedAdjacencyList<int, int>> prim(graph);
  prim.compute();

  EXPECT_FALSE(prim.isConnected());
}

TEST(PrimTest, SingleVertexGraph) {
  UndirectedAdjacencyList<int, int> graph;
  graph.addVertex(1);

  Prim<UndirectedAdjacencyList<int, int>> prim(graph);
  prim.compute();

  EXPECT_TRUE(prim.isConnected());
  EXPECT_EQ(prim.getTotalWeight(), 0);
  EXPECT_TRUE(prim.getMSTEdges().empty());
}

TEST(PrimTest, ComplexGraph) {
  UndirectedAdjacencyList<std::string, double> graph;

  graph.addVertex("A").addVertex("B").addVertex("C");
  graph.addVertex("D").addVertex("E").addVertex("F");

  graph.addEdge("A", "B", 7.0);
  graph.addEdge("A", "C", 9.0);
  graph.addEdge("A", "D", 14.0);
  graph.addEdge("B", "C", 10.0);
  graph.addEdge("B", "E", 15.0);
  graph.addEdge("C", "D", 11.0);
  graph.addEdge("C", "E", 2.0);
  graph.addEdge("D", "E", 6.0);
  graph.addEdge("D", "F", 9.0);
  graph.addEdge("E", "F", 5.0);

  Prim<UndirectedAdjacencyList<std::string, double>> prim(graph);
  prim.compute("A");

  EXPECT_TRUE(prim.isConnected());
  EXPECT_DOUBLE_EQ(prim.getTotalWeight(), 29.0);

  auto mst_edges = prim.getMSTEdges();
  EXPECT_EQ(mst_edges.size(), 5);
}

TEST(PrimTest, NegativeWeights) {
  UndirectedEdgeList<int, int> graph;

  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, -5);
  graph.addEdge(2, 3, -2);
  graph.addEdge(3, 4, -1);
  graph.addEdge(1, 4, -3);
  graph.addEdge(1, 3, 10);

  Prim<UndirectedEdgeList<int, int>> prim(graph);
  prim.compute();

  EXPECT_TRUE(prim.isConnected());
  EXPECT_EQ(prim.getTotalWeight(), -10);

  auto mst_edges = prim.getMSTEdges();
  EXPECT_EQ(mst_edges.size(), 3);

  std::vector<int> weights;
  for (const auto &edge : mst_edges) {
    weights.push_back(edge.weight);
  }
  std::sort(weights.begin(), weights.end());

  EXPECT_EQ(weights[0], -5);
  EXPECT_EQ(weights[1], -3);
  EXPECT_EQ(weights[2], -2);
}

TEST(PrimTest, DifferentStartVertices) {
  UndirectedAdjacencyList<int, int> graph;

  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, 10);
  graph.addEdge(1, 3, 15);
  graph.addEdge(2, 3, 5);
  graph.addEdge(2, 4, 5);
  graph.addEdge(3, 4, 10);

  Prim<UndirectedAdjacencyList<int, int>> prim1(graph);
  prim1.compute(1);

  Prim<UndirectedAdjacencyList<int, int>> prim2(graph);
  prim2.compute(4);

  EXPECT_EQ(prim1.getTotalWeight(), prim2.getTotalWeight());
  EXPECT_EQ(prim1.getTotalWeight(), 20);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
