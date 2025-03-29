#include <gtest/gtest.h>

#include <chrono>

#include "graph/algorithms/flow/edmonds_karp.hpp"
#include "graph/algorithms/flow/ford_fulkerson.hpp"
#include "graph/structure/directed_adjacency_list.hpp"

using namespace graph;
using namespace graph::algorithms;
using namespace graph::algorithms::flow;

TEST(EdmondsKarpTest, SimpleGraph) {
  DirectedAdjacencyList<int, int> graph;

  graph.addVertex(0).addVertex(1).addVertex(2).addVertex(3).addVertex(4);

  graph.addEdge(0, 1, 10);
  graph.addEdge(0, 2, 5);
  graph.addEdge(1, 3, 8);
  graph.addEdge(1, 4, 5);
  graph.addEdge(2, 4, 5);
  graph.addEdge(4, 3, 8);

  EdmondsKarp<DirectedAdjacencyList<int, int>, int, int> maxflow(graph, 0, 3);
  maxflow.compute();

  EXPECT_EQ(maxflow.getMaxFlow(), 15);
}

TEST(EdmondsKarpTest, NoPath) {
  DirectedAdjacencyList<int, int> graph;

  graph.addVertex(0).addVertex(1);

  EdmondsKarp<DirectedAdjacencyList<int, int>, int, int> maxflow(graph, 0, 1);
  maxflow.compute();

  EXPECT_EQ(maxflow.getMaxFlow(), 0);
}

TEST(EdmondsKarpTest, ComplexGraph) {
  DirectedAdjacencyList<std::string, int> graph;

  graph.addVertex("S")
      .addVertex("A")
      .addVertex("B")
      .addVertex("C")
      .addVertex("D")
      .addVertex("T");

  graph.addEdge("S", "A", 10);
  graph.addEdge("S", "C", 10);
  graph.addEdge("A", "B", 4);
  graph.addEdge("A", "C", 2);
  graph.addEdge("A", "D", 8);
  graph.addEdge("B", "T", 10);
  graph.addEdge("C", "D", 9);
  graph.addEdge("D", "B", 6);
  graph.addEdge("D", "T", 10);

  EdmondsKarp<DirectedAdjacencyList<std::string, int>, std::string, int>
      maxflow(graph, "S", "T");
  maxflow.compute();

  EXPECT_EQ(maxflow.getMaxFlow(), 19);

  int sa_flow = maxflow.getFlow("S", "A");
  int sc_flow = maxflow.getFlow("S", "C");
  int dt_flow = maxflow.getFlow("D", "T");

  EXPECT_EQ(sa_flow + sc_flow, 19);
  EXPECT_GE(dt_flow, 0);
  EXPECT_LE(dt_flow, 10);
}

TEST(EdmondsKarpTest, CompareWithFordFulkerson) {
  DirectedAdjacencyList<int, int> graph;

  for (int i = 0; i < 8; i++) {
    graph.addVertex(i);
  }

  graph.addEdge(0, 1, 10);
  graph.addEdge(0, 2, 5);
  graph.addEdge(0, 3, 15);
  graph.addEdge(1, 4, 9);
  graph.addEdge(1, 5, 15);
  graph.addEdge(2, 5, 8);
  graph.addEdge(2, 3, 4);
  graph.addEdge(3, 6, 16);
  graph.addEdge(4, 5, 15);
  graph.addEdge(4, 7, 10);
  graph.addEdge(5, 7, 10);
  graph.addEdge(5, 6, 15);
  graph.addEdge(6, 2, 6);
  graph.addEdge(6, 7, 10);

  EdmondsKarp<DirectedAdjacencyList<int, int>, int, int> edmonds_karp(graph, 0,
                                                                      7);
  edmonds_karp.compute();

  FordFulkerson<DirectedAdjacencyList<int, int>, int, int> ford_fulkerson(graph,
                                                                          0, 7);
  ford_fulkerson.compute();

  EXPECT_EQ(edmonds_karp.getMaxFlow(), ford_fulkerson.getMaxFlow());
  EXPECT_EQ(edmonds_karp.getMaxFlow(), 28);
}

TEST(EdmondsKarpTest, BFSPerformance) {
  DirectedAdjacencyList<int, int> graph;

  const int n = 100;

  for (int i = 0; i <= 201; i++) {
    graph.addVertex(i);
  }

  for (int i = 1; i <= n; i++) {
    graph.addEdge(0, i, 1);
  }

  for (int i = 1; i <= n; i++) {
    for (int j = 1; j <= std::min(3, n); j++) {
      int target = 100 + ((i + j) % n) + 1;
      graph.addEdge(i, target, 1);
    }
  }

  for (int i = 101; i <= 100 + n; i++) {
    graph.addEdge(i, 201, 1);
  }

  auto start_ek = std::chrono::high_resolution_clock::now();
  EdmondsKarp<DirectedAdjacencyList<int, int>, int, int> edmonds_karp(graph, 0,
                                                                      201);
  edmonds_karp.compute();
  auto end_ek = std::chrono::high_resolution_clock::now();
  auto duration_ek =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_ek - start_ek)
          .count();

  auto start_ff = std::chrono::high_resolution_clock::now();
  FordFulkerson<DirectedAdjacencyList<int, int>, int, int> ford_fulkerson(
      graph, 0, 201);
  ford_fulkerson.compute();
  auto end_ff = std::chrono::high_resolution_clock::now();
  auto duration_ff =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_ff - start_ff)
          .count();

  EXPECT_EQ(edmonds_karp.getMaxFlow(), ford_fulkerson.getMaxFlow());

  std::cout << "Edmonds-Karp time: " << duration_ek << " ms\n";
  std::cout << "Ford-Fulkerson time: " << duration_ff << " ms\n";
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
