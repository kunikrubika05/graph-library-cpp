#include <gtest/gtest.h>

#include "graph/algorithms/flow/ford_fulkerson.hpp"
#include "graph/structure/directed_adjacency_list.hpp"

using namespace graph;
using namespace graph::algorithms;
using namespace graph::algorithms::flow;

TEST(FordFulkersonTest, SimpleGraph) {
  DirectedAdjacencyList<int, int> graph;

  graph.addVertex(0).addVertex(1).addVertex(2).addVertex(3).addVertex(4);

  graph.addEdge(0, 1, 10);
  graph.addEdge(0, 2, 5);
  graph.addEdge(1, 3, 8);
  graph.addEdge(1, 4, 5);
  graph.addEdge(2, 4, 5);
  graph.addEdge(4, 3, 8);

  FordFulkerson<DirectedAdjacencyList<int, int>, int, int> maxflow(graph, 0, 3);
  maxflow.compute();

  EXPECT_EQ(maxflow.getMaxFlow(), 15);
}

TEST(FordFulkersonTest, NoPath) {
  DirectedAdjacencyList<int, int> graph;

  graph.addVertex(0).addVertex(1);

  FordFulkerson<DirectedAdjacencyList<int, int>, int, int> maxflow(graph, 0, 1);
  maxflow.compute();

  EXPECT_EQ(maxflow.getMaxFlow(), 0);
}

TEST(FordFulkersonTest, ComplexGraph) {
  DirectedAdjacencyList<std::string, int> graph;

  graph.addVertex("S").addVertex("A").addVertex("B").addVertex("C")
      .addVertex("D").addVertex("T");

  graph.addEdge("S", "A", 10);
  graph.addEdge("S", "C", 10);
  graph.addEdge("A", "B", 4);
  graph.addEdge("A", "C", 2);
  graph.addEdge("A", "D", 8);
  graph.addEdge("B", "T", 10);
  graph.addEdge("C", "D", 9);
  graph.addEdge("D", "B", 6);
  graph.addEdge("D", "T", 10);

  FordFulkerson<DirectedAdjacencyList<std::string, int>, std::string, int>
      maxflow(graph, "S", "T");
  maxflow.compute();

  EXPECT_EQ(maxflow.getMaxFlow(), 19);

  EXPECT_EQ(maxflow.getFlow("S", "A"), 10);
  EXPECT_EQ(maxflow.getFlow("S", "C"), 9);
  EXPECT_EQ(maxflow.getFlow("A", "D"), 6);
  EXPECT_EQ(maxflow.getFlow("D", "T"), 10);
}

TEST(FordFulkersonTest, BottleneckGraph) {
  DirectedAdjacencyList<int, int> graph;

  graph.addVertex(0).addVertex(1).addVertex(2).addVertex(3);

  graph.addEdge(0, 1, 1000);
  graph.addEdge(1, 2, 1);
  graph.addEdge(2, 3, 1000);

  FordFulkerson<DirectedAdjacencyList<int, int>, int, int> maxflow(graph, 0, 3);
  maxflow.compute();

  EXPECT_EQ(maxflow.getMaxFlow(), 1);
}

TEST(FordFulkersonTest, ParallelEdges) {
  DirectedAdjacencyList<int, int> graph;

  graph.addVertex(0).addVertex(1);

  graph.addEdge(0, 1, 5);

  FordFulkerson<DirectedAdjacencyList<int, int>, int, int> maxflow(graph, 0, 1);
  maxflow.compute();

  EXPECT_EQ(maxflow.getMaxFlow(), 5);
}

TEST(FordFulkersonTest, LargerNetwork) {
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

  FordFulkerson<DirectedAdjacencyList<int, int>, int, int> maxflow(graph, 0, 7);
  maxflow.compute();

  EXPECT_EQ(maxflow.getMaxFlow(), 28);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
