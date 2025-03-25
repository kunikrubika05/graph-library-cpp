#include <gtest/gtest.h>

#include <vector>

#include "graph/algorithms/bfs/components.hpp"
#include "graph/structure/undirected_adjacency_list.hpp"

using namespace graph;
using namespace graph::algorithms;

TEST(ConnectedComponentsTest, SingleComponent) {
  UndirectedAdjacencyList<int, int> graph;
  graph.addVertex(1).addVertex(2).addVertex(3);
  graph.addEdge(1, 2, 1).addEdge(2, 3, 1);

  ConnectedComponents<UndirectedAdjacencyList<int, int>> cc(graph);
  auto components = cc.computeComponents();

  EXPECT_EQ(components.size(), 1);
  EXPECT_EQ(components[0].size(), 3);
}

TEST(ConnectedComponentsTest, MultipleComponents) {
  UndirectedAdjacencyList<int, int> graph;
  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);
  graph.addEdge(1, 2, 1);
  graph.addEdge(3, 4, 1);

  ConnectedComponents<UndirectedAdjacencyList<int, int>> cc(graph);
  auto components = cc.computeComponents();

  EXPECT_EQ(components.size(), 3);
  bool found2 = false, found2_2 = false, found1 = false;
  for (const auto &comp : components) {
    if (comp.size() == 2) {
      if (!found2)
        found2 = true;
      else
        found2_2 = true;
    } else if (comp.size() == 1)
      found1 = true;
  }
  EXPECT_TRUE(found2 && found2_2 && found1);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
