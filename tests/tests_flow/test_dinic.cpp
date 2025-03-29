#include <gtest/gtest.h>
#include "graph/structure/directed_adjacency_list.hpp"
#include "graph/algorithms/flow/dinic.hpp"

TEST(DinicTest, SimpleMaxFlow) {
  graph::DirectedAdjacencyList<int, int> g;
  g.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  g.addEdge(1, 2, 3);
  g.addEdge(1, 3, 2);
  g.addEdge(2, 3, 1);
  g.addEdge(2, 4, 2);
  g.addEdge(3, 4, 3);

  graph::algorithms::flow::Dinic<graph::DirectedAdjacencyList<int, int>, int, int> flow(g, 1, 4);
  flow.compute();
  EXPECT_EQ(flow.getMaxFlow(), 5);
}

TEST(DinicTest, CheckFlowConservation) {
  graph::DirectedAdjacencyList<int, int> g;
  g.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  g.addEdge(1, 2, 3);
  g.addEdge(1, 3, 2);
  g.addEdge(2, 3, 1);
  g.addEdge(2, 4, 2);
  g.addEdge(3, 4, 3);

  graph::algorithms::flow::Dinic<graph::DirectedAdjacencyList<int, int>, int, int> flow(g, 1, 4);
  flow.compute();

  int in_flow_3 = flow.getFlow(1, 3) + flow.getFlow(2, 3);
  int out_flow_3 = flow.getFlow(3, 4);
  EXPECT_EQ(in_flow_3, out_flow_3);
}

TEST(DinicTest, EmptyGraph) {
  graph::DirectedAdjacencyList<int, int> empty_g;
  empty_g.addVertex(1).addVertex(2);

  graph::algorithms::flow::Dinic<graph::DirectedAdjacencyList<int, int>, int, int> flow(empty_g, 1, 2);
  flow.compute();
  EXPECT_EQ(flow.getMaxFlow(), 0);
}

TEST(DinicTest, LargeCapacities) {
  graph::DirectedAdjacencyList<int, int> large_g;
  large_g.addVertex(1).addVertex(2).addVertex(3);
  large_g.addEdge(1, 2, 100000);
  large_g.addEdge(2, 3, 100000);

  graph::algorithms::flow::Dinic<graph::DirectedAdjacencyList<int, int>, int, int> flow(large_g, 1, 3);
  flow.compute();
  EXPECT_EQ(flow.getMaxFlow(), 100000);
}

TEST(DinicTest, BlockingFlow) {
  graph::DirectedAdjacencyList<int, int> blocking_g;
  blocking_g.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);

  blocking_g.addEdge(1, 2, 10);
  blocking_g.addEdge(1, 3, 10);
  blocking_g.addEdge(2, 4, 1);
  blocking_g.addEdge(3, 4, 10);
  blocking_g.addEdge(4, 5, 11);

  graph::algorithms::flow::Dinic<graph::DirectedAdjacencyList<int, int>, int, int> flow(blocking_g, 1, 5);
  flow.compute();
  EXPECT_EQ(flow.getMaxFlow(), 11);
}
