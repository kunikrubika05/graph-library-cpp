#include <gtest/gtest.h>

#include <chrono>
#include <iostream>

#include "graph/structure/directed_adjacency_list.hpp"

using namespace graph;

TEST(DirectedAdjacencyListTest, AddVertexSilent) {
  DirectedAdjacencyList<int, double> graph;
  graph.addVertex(1).addVertex(2);
  EXPECT_EQ(graph.vertexCount(), 2);
  graph.addVertex(1);
  EXPECT_EQ(graph.vertexCount(), 2);
}

TEST(DirectedAdjacencyListTest, AddEdgeSilent) {
  DirectedAdjacencyList<int, double> graph;
  graph.addVertex(1).addVertex(2);
  graph.addEdge(1, 2, 1.5);
  EXPECT_TRUE(graph.hasEdge(1, 2));
  EXPECT_DOUBLE_EQ(graph.getEdgeWeight(1, 2), 1.5);
}

TEST(DirectedAdjacencyListTest, ErrorThrowingOnDuplicateVertex) {
  DirectedAdjacencyList<int, double> graph;
  graph.setErrorPolicy(ErrorPolicy::THROW_EXCEPTIONS);
  graph.addVertex(1);
  EXPECT_THROW(graph.addVertex(1), std::exception);
}

TEST(DirectedAdjacencyListTest, ErrorThrowingOnSelfLoop) {
  DirectedAdjacencyList<int, double> graph;
  graph.setErrorPolicy(ErrorPolicy::THROW_EXCEPTIONS);
  graph.addVertex(1);
  EXPECT_THROW(graph.addEdge(1, 1, 1.0), std::exception);
}

TEST(DirectedAdjacencyListTest, RemoveEdgeError) {
  DirectedAdjacencyList<int, double> graph;
  graph.setErrorPolicy(ErrorPolicy::THROW_EXCEPTIONS);
  graph.addVertex(1).addVertex(2);
  EXPECT_THROW(graph.removeEdge(1, 2), std::exception);
}

TEST(DirectedAdjacencyListTest, RemoveVertexError) {
  DirectedAdjacencyList<int, double> graph;
  graph.setErrorPolicy(ErrorPolicy::THROW_EXCEPTIONS);
  graph.addVertex(1);
  EXPECT_THROW(graph.removeVertex(2), std::exception);
}

TEST(DirectedAdjacencyListTest, NeighborsError) {
  DirectedAdjacencyList<int, double> graph;
  graph.setErrorPolicy(ErrorPolicy::THROW_EXCEPTIONS);
  graph.addVertex(1);
  EXPECT_THROW(graph.getNeighbors(2), std::exception);
}

TEST(DirectedAdjacencyListTest, ComplexGraphOperations) {
  DirectedAdjacencyList<int, double> graph;
  graph.setErrorPolicy(ErrorPolicy::THROW_EXCEPTIONS);
  for (int i = 1; i <= 5; ++i) {
    graph.addVertex(i);
  }
  graph.addEdge(1, 2, 1.1);
  graph.addEdge(1, 3, 1.2);
  graph.addEdge(2, 4, 1.3);
  graph.addEdge(3, 5, 1.4);
  EXPECT_TRUE(graph.hasEdge(1, 2));
  EXPECT_TRUE(graph.hasEdge(1, 3));
  EXPECT_FALSE(graph.hasEdge(2, 3));

  graph.removeEdge(1, 2);
  EXPECT_THROW(graph.removeEdge(1, 2), std::exception);
}

TEST(DirectedAdjacencyListTest, VertexPerformanceTest) {
  DirectedAdjacencyList<int, double> graph;
  const int count = 10000000;

  auto start = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < count; ++i) {
    graph.addVertex(i);
  }

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;

  std::cout << "Time to add " << count << " vertices: " << elapsed.count()
            << " sec\n";

  EXPECT_EQ(graph.vertexCount(), count);
}

TEST(DirectedAdjacencyListTest, EdgePerformanceTest) {
  DirectedAdjacencyList<int, double> graph;
  const int vertexCount = 10000000;

  for (int i = 0; i < vertexCount; ++i) {
    graph.addVertex(i);
  }

  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < vertexCount - 1; ++i) {
    graph.addEdge(i, i + 1, 1.0);
  }
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "Time to add " << vertexCount - 1
            << " edges: " << elapsed.count() << " sec\n";

  EXPECT_EQ(graph.edgeCount(), vertexCount - 1);
}

TEST(DirectedAdjacencyListTest, RandomEdgeInsertionTest) {
  DirectedAdjacencyList<int, double> graph;
  const int count = 5000;
  for (int i = 0; i < count; ++i) {
    graph.addVertex(i);
  }
  for (int i = 0; i < count; ++i) {
    int j = (i + 500) % count;
    graph.addEdge(i, j, static_cast<double>(i) / 10);
    EXPECT_TRUE(graph.hasEdge(i, j));
    EXPECT_DOUBLE_EQ(graph.getEdgeWeight(i, j), static_cast<double>(i) / 10);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
