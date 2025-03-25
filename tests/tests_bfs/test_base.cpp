#include <gtest/gtest.h>

#include <vector>

#include "graph/algorithms/bfs/breadth_first_search.hpp"
#include "graph/structure/directed_adjacency_list.hpp"
#include "graph/structure/directed_adjacency_matrix.hpp"
#include "graph/structure/undirected_adjacency_list.hpp"
#include "graph/structure/undirected_adjacency_matrix.hpp"

using namespace graph;
using namespace graph::algorithms;

TEST(BFSTest, EmptyGraph) {
  UndirectedAdjacencyList<int, int> graph;
  BreadthFirstSearch<UndirectedAdjacencyList<int, int>> bfs(graph);

  bfs.traverseAll();
  EXPECT_TRUE(bfs.getVisited().empty());
}

TEST(BFSTest, DisconnectedGraph) {
  UndirectedAdjacencyList<int, int> graph;
  graph.addVertex(1)
      .addVertex(2)
      .addVertex(3)
      .addVertex(4)
      .addVertex(5)
      .addEdge(1, 2, 1)
      .addEdge(2, 3, 1)
      .addEdge(4, 5, 1);

  BreadthFirstSearch<UndirectedAdjacencyList<int, int>> bfs(graph);

  std::vector<int> order;
  bfs.traverseAll([](int) {}, [&order](int v) { order.push_back(v); });

  EXPECT_EQ(5, order.size());
  EXPECT_EQ(5, bfs.getVisited().size());

  for (int v = 1; v <= 5; ++v) {
    EXPECT_TRUE(bfs.getVisited().at(v));
  }
}

TEST(BFSTest, OrderTest) {
  UndirectedAdjacencyList<int, int> graph;
  graph.addVertex(1)
      .addVertex(2)
      .addVertex(3)
      .addVertex(4)
      .addVertex(5)
      .addEdge(1, 2, 1)
      .addEdge(1, 3, 1)
      .addEdge(2, 4, 1)
      .addEdge(2, 5, 1)
      .addEdge(3, 5, 1);

  BreadthFirstSearch<UndirectedAdjacencyList<int, int>> bfs(graph);

  std::vector<int> order;
  bfs.traverse(1, [](int) {}, [&order](int v) { order.push_back(v); });

  EXPECT_EQ(5, order.size());

  EXPECT_EQ(1, order[0]);

  auto pos2 = std::find(order.begin(), order.end(), 2) - order.begin();
  auto pos3 = std::find(order.begin(), order.end(), 3) - order.begin();
  auto pos4 = std::find(order.begin(), order.end(), 4) - order.begin();
  auto pos5 = std::find(order.begin(), order.end(), 5) - order.begin();

  EXPECT_LT(pos2, pos4);
  EXPECT_LT(pos2, pos5);
  EXPECT_LT(pos3, pos5);
}

TEST(BFSTest, DistanceTest) {
  DirectedAdjacencyList<int, int> graph;
  graph.addVertex(1)
      .addVertex(2)
      .addVertex(3)
      .addVertex(4)
      .addVertex(5)
      .addEdge(1, 2, 1)
      .addEdge(2, 3, 1)
      .addEdge(3, 4, 1)
      .addEdge(4, 5, 1);

  BreadthFirstSearch<DirectedAdjacencyList<int, int>> bfs(graph);

  bfs.traverse(1);

  EXPECT_EQ(0, bfs.getDistance().at(1));
  EXPECT_EQ(1, bfs.getDistance().at(2));
  EXPECT_EQ(2, bfs.getDistance().at(3));
  EXPECT_EQ(3, bfs.getDistance().at(4));
  EXPECT_EQ(4, bfs.getDistance().at(5));
}

TEST(BFSTest, ParentTest) {
  UndirectedAdjacencyList<int, int> graph;
  graph.addVertex(1)
      .addVertex(2)
      .addVertex(3)
      .addVertex(4)
      .addVertex(5)
      .addEdge(1, 2, 1)
      .addEdge(1, 3, 1)
      .addEdge(2, 4, 1)
      .addEdge(3, 5, 1);

  BreadthFirstSearch<UndirectedAdjacencyList<int, int>> bfs(graph);

  bfs.traverse(1);

  const auto &parent = bfs.getParent();

  EXPECT_EQ(0, parent.count(1));

  EXPECT_EQ(1, parent.at(2));
  EXPECT_EQ(1, parent.at(3));

  EXPECT_EQ(2, parent.at(4));

  EXPECT_EQ(3, parent.at(5));
}

TEST(BFSMatrixTest, EmptyGraph) {
  UndirectedAdjacencyMatrix<int, int> graph(10);
  BreadthFirstSearch<UndirectedAdjacencyMatrix<int, int>> bfs(graph);

  bfs.traverseAll();
  EXPECT_TRUE(bfs.getVisited().empty());
}

TEST(BFSMatrixTest, DisconnectedGraph) {
  UndirectedAdjacencyMatrix<int, int> graph(10);
  graph.addVertex(1)
      .addVertex(2)
      .addVertex(3)
      .addVertex(4)
      .addVertex(5)
      .addEdge(1, 2, 1)
      .addEdge(2, 3, 1)
      .addEdge(4, 5, 1);

  BreadthFirstSearch<UndirectedAdjacencyMatrix<int, int>> bfs(graph);

  std::vector<int> order;
  bfs.traverseAll([](int) {}, [&order](int v) { order.push_back(v); });

  EXPECT_EQ(5, order.size());
  EXPECT_EQ(5, bfs.getVisited().size());

  for (int v = 1; v <= 5; ++v) {
    EXPECT_TRUE(bfs.getVisited().at(v));
  }
}

TEST(BFSMatrixTest, OrderTest) {
  UndirectedAdjacencyMatrix<int, int> graph(10);
  graph.addVertex(1)
      .addVertex(2)
      .addVertex(3)
      .addVertex(4)
      .addVertex(5)
      .addEdge(1, 2, 1)
      .addEdge(1, 3, 1)
      .addEdge(2, 4, 1)
      .addEdge(2, 5, 1)
      .addEdge(3, 5, 1);

  BreadthFirstSearch<UndirectedAdjacencyMatrix<int, int>> bfs(graph);

  std::vector<int> order;
  bfs.traverse(1, [](int) {}, [&order](int v) { order.push_back(v); });

  EXPECT_EQ(5, order.size());
  EXPECT_EQ(1, order[0]);

  auto pos2 = std::find(order.begin(), order.end(), 2) - order.begin();
  auto pos3 = std::find(order.begin(), order.end(), 3) - order.begin();
  auto pos4 = std::find(order.begin(), order.end(), 4) - order.begin();
  auto pos5 = std::find(order.begin(), order.end(), 5) - order.begin();

  EXPECT_LT(pos2, pos4);
  EXPECT_LT(pos2, pos5);
  EXPECT_LT(pos3, pos5);
}

TEST(BFSMatrixTest, DistanceTest) {
  DirectedAdjacencyMatrix<int, int> graph(10);
  graph.addVertex(1)
      .addVertex(2)
      .addVertex(3)
      .addVertex(4)
      .addVertex(5)
      .addEdge(1, 2, 1)
      .addEdge(2, 3, 1)
      .addEdge(3, 4, 1)
      .addEdge(4, 5, 1);

  BreadthFirstSearch<DirectedAdjacencyMatrix<int, int>> bfs(graph);

  bfs.traverse(1);

  EXPECT_EQ(0, bfs.getDistance().at(1));
  EXPECT_EQ(1, bfs.getDistance().at(2));
  EXPECT_EQ(2, bfs.getDistance().at(3));
  EXPECT_EQ(3, bfs.getDistance().at(4));
  EXPECT_EQ(4, bfs.getDistance().at(5));
}

TEST(BFSMatrixTest, ParentTest) {
  UndirectedAdjacencyMatrix<int, int> graph(10);
  graph.addVertex(1)
      .addVertex(2)
      .addVertex(3)
      .addVertex(4)
      .addVertex(5)
      .addEdge(1, 2, 1)
      .addEdge(1, 3, 1)
      .addEdge(2, 4, 1)
      .addEdge(3, 5, 1);

  BreadthFirstSearch<UndirectedAdjacencyMatrix<int, int>> bfs(graph);

  bfs.traverse(1);

  const auto &parent = bfs.getParent();

  EXPECT_EQ(0, parent.count(1));
  EXPECT_EQ(1, parent.at(2));
  EXPECT_EQ(1, parent.at(3));
  EXPECT_EQ(2, parent.at(4));
  EXPECT_EQ(3, parent.at(5));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
