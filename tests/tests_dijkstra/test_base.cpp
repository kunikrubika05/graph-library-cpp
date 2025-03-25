#include <gtest/gtest.h>

#include <vector>

#include "graph/algorithms/dijkstra/dijkstra.hpp"
#include "graph/structure/directed_adjacency_list.hpp"
#include "graph/structure/directed_adjacency_matrix.hpp"

using namespace graph;
using namespace graph::algorithms;

TEST(DijkstraTest, SimpleGraph) {
  DirectedAdjacencyList<int, int> graph;

  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);
  graph.addEdge(1, 2, 2).addEdge(1, 4, 5);
  graph.addEdge(2, 3, 1).addEdge(2, 5, 3);
  graph.addEdge(4, 5, 1);

  Dijkstra<DirectedAdjacencyList<int, int>> dijkstra(graph);
  dijkstra.compute(1);

  EXPECT_EQ(dijkstra.getDistance(1), 0);
  EXPECT_EQ(dijkstra.getDistance(2), 2);
  EXPECT_EQ(dijkstra.getDistance(3), 3);
  EXPECT_EQ(dijkstra.getDistance(4), 5);
  EXPECT_EQ(dijkstra.getDistance(5), 5);

  std::vector<int> path_to_3 = dijkstra.retrievePath(3);
  std::vector<int> expected_path_to_3 = {1, 2, 3};
  EXPECT_EQ(path_to_3, expected_path_to_3);

  std::vector<int> path_to_5 = dijkstra.retrievePath(5);
  std::vector<int> expected_path_to_5_1 = {1, 2, 5};
  std::vector<int> expected_path_to_5_2 = {1, 4, 5};
  EXPECT_TRUE(path_to_5 == expected_path_to_5_1 ||
              path_to_5 == expected_path_to_5_2);
}

TEST(DijkstraTest, UnreachableVertex) {
  DirectedAdjacencyList<int, int> graph;
  graph.addVertex(1).addVertex(2).addVertex(3);
  graph.addEdge(1, 2, 1);

  Dijkstra<DirectedAdjacencyList<int, int>> dijkstra(graph);
  dijkstra.compute(1);

  EXPECT_THROW(dijkstra.retrievePath(3), std::runtime_error);
}

TEST(DijkstraMatrixTest, SimpleGraph) {
  DirectedAdjacencyMatrix<int, int> graph(3);
  graph.addVertex(1).addVertex(2).addVertex(3);

  graph.addEdge(1, 2, 2);
  graph.addEdge(1, 3, 5);
  graph.addEdge(2, 3, 1);

  Dijkstra<DirectedAdjacencyMatrix<int, int>> dijkstra(graph);
  dijkstra.compute(1);

  EXPECT_EQ(dijkstra.getDistance(1), 0);
  EXPECT_EQ(dijkstra.getDistance(2), 2);
  EXPECT_EQ(dijkstra.getDistance(3), 3);
}

TEST(DijkstraMatrixTest, UnreachableVertex) {
  DirectedAdjacencyMatrix<int, int> graph(3);
  graph.addVertex(1).addVertex(2).addVertex(3);

  graph.addEdge(2, 3, 2);
  graph.addEdge(3, 1, 4);

  Dijkstra<DirectedAdjacencyMatrix<int, int>> dijkstra(graph);
  dijkstra.compute(1);

  EXPECT_THROW(dijkstra.getDistance(2), std::runtime_error);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
