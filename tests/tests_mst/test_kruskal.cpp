#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "graph/algorithms/mst/kruskal.hpp"
#include "graph/structure/undirected_adjacency_list.hpp"
#include "graph/structure/undirected_edge_list.hpp"

using namespace graph;
using namespace graph::algorithms;

TEST(KruskalTest, SimpleGraph) {
  UndirectedAdjacencyList<int, int> graph;

  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, 1);
  graph.addEdge(2, 3, 2);
  graph.addEdge(3, 4, 3);
  graph.addEdge(4, 1, 4);
  graph.addEdge(2, 4, 5);

  Kruskal<UndirectedAdjacencyList<int, int>> kruskal(graph);
  kruskal.compute();

  EXPECT_TRUE(kruskal.isConnected());
  EXPECT_EQ(kruskal.getTotalWeight(), 6);

  auto mst_edges = kruskal.getMSTEdges();
  EXPECT_EQ(mst_edges.size(), 3);

  std::vector<bool> edge_found(4, false);
  for (const auto &edge : mst_edges) {
    if ((edge.from == 1 && edge.to == 2) || (edge.from == 2 && edge.to == 1)) {
      edge_found[0] = true;
      EXPECT_EQ(edge.weight, 1);
    } else if ((edge.from == 2 && edge.to == 3) ||
               (edge.from == 3 && edge.to == 2)) {
      edge_found[1] = true;
      EXPECT_EQ(edge.weight, 2);
    } else if ((edge.from == 3 && edge.to == 4) ||
               (edge.from == 4 && edge.to == 3)) {
      edge_found[2] = true;
      EXPECT_EQ(edge.weight, 3);
    } else if ((edge.from == 1 && edge.to == 4) ||
               (edge.from == 4 && edge.to == 1)) {
      edge_found[3] = true;
      EXPECT_EQ(edge.weight, 4);
    }
  }

  EXPECT_TRUE(edge_found[0] && edge_found[1] && edge_found[2]);
}

TEST(KruskalTest, DisconnectedGraph) {
  UndirectedAdjacencyList<int, int> graph;

  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, 1);
  graph.addEdge(3, 4, 2);

  Kruskal<UndirectedAdjacencyList<int, int>> kruskal(graph);
  kruskal.compute();

  EXPECT_FALSE(kruskal.isConnected());
  EXPECT_THROW(kruskal.getTotalWeight(), std::runtime_error);
  EXPECT_THROW(kruskal.getMSTEdges(), std::runtime_error);
}

TEST(KruskalTest, EmptyGraph) {
  UndirectedAdjacencyList<int, int> graph;

  Kruskal<UndirectedAdjacencyList<int, int>> kruskal(graph);
  kruskal.compute();

  EXPECT_FALSE(kruskal.isConnected());
}

TEST(KruskalTest, SingleVertexGraph) {
  UndirectedAdjacencyList<int, int> graph;
  graph.addVertex(1);

  Kruskal<UndirectedAdjacencyList<int, int>> kruskal(graph);
  kruskal.compute();

  EXPECT_TRUE(kruskal.isConnected());
  EXPECT_EQ(kruskal.getTotalWeight(), 0);
  EXPECT_TRUE(kruskal.getMSTEdges().empty());
}

TEST(KruskalTest, ComplexGraph) {
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

  Kruskal<UndirectedAdjacencyList<std::string, double>> kruskal(graph);
  kruskal.compute();

  EXPECT_TRUE(kruskal.isConnected());
  EXPECT_DOUBLE_EQ(kruskal.getTotalWeight(), 29.0);

  auto mst_edges = kruskal.getMSTEdges();
  EXPECT_EQ(mst_edges.size(), 5);
}

TEST(KruskalTest, NegativeWeights) {
  UndirectedEdgeList<int, int> graph;

  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, -5);
  graph.addEdge(2, 3, -2);
  graph.addEdge(3, 4, -1);
  graph.addEdge(1, 4, -3);
  graph.addEdge(1, 3, 10);

  Kruskal<UndirectedEdgeList<int, int>> kruskal(graph);
  kruskal.compute();

  EXPECT_TRUE(kruskal.isConnected());
  EXPECT_EQ(kruskal.getTotalWeight(), -10);

  auto mst_edges = kruskal.getMSTEdges();
  EXPECT_EQ(mst_edges.size(), 3);

  std::sort(mst_edges.begin(), mst_edges.end());
  EXPECT_EQ(mst_edges[0].weight, -5);
  EXPECT_EQ(mst_edges[1].weight, -3);
  EXPECT_EQ(mst_edges[2].weight, -2);
}

TEST(KruskalTest, MultipleEqualWeights) {
  UndirectedAdjacencyList<int, int> graph;

  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, 5);
  graph.addEdge(2, 3, 5);
  graph.addEdge(3, 4, 5);
  graph.addEdge(4, 1, 5);

  Kruskal<UndirectedAdjacencyList<int, int>> kruskal(graph);
  kruskal.compute();

  EXPECT_TRUE(kruskal.isConnected());
  EXPECT_EQ(kruskal.getTotalWeight(), 15);

  auto mst_edges = kruskal.getMSTEdges();
  EXPECT_EQ(mst_edges.size(), 3);
}

TEST(KruskalTest, LargeGraph) {
  UndirectedAdjacencyList<int, int> graph;

  const int size = 20;
  const int total_vertices = size * size;

  for (int i = 0; i < total_vertices; i++) {
    graph.addVertex(i);
  }

  for (int i = 0; i < size; i++) {
    for (int j = 0; j < size - 1; j++) {
      int vertex1 = i * size + j;
      int vertex2 = i * size + j + 1;
      graph.addEdge(vertex1, vertex2, 1);
    }
  }

  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size; j++) {
      int vertex1 = i * size + j;
      int vertex2 = (i + 1) * size + j;
      graph.addEdge(vertex1, vertex2, 2);
    }
  }

  Kruskal<UndirectedAdjacencyList<int, int>> kruskal(graph);
  kruskal.compute();

  EXPECT_TRUE(kruskal.isConnected());

  auto mst_edges = kruskal.getMSTEdges();
  EXPECT_EQ(mst_edges.size(), total_vertices - 1);

  int count_weight_1 = 0;
  int count_weight_2 = 0;

  for (const auto &edge : mst_edges) {
    if (edge.weight == 1)
      count_weight_1++;
    else if (edge.weight == 2)
      count_weight_2++;
  }

  EXPECT_EQ(count_weight_1, size * (size - 1));
  EXPECT_EQ(count_weight_2, size - 1);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
