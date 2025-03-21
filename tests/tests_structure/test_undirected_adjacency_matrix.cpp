#include "graph/structure/undirected_adjacency_matrix.hpp"

#include <algorithm>
#include <chrono>
#include <gtest/gtest.h>
#include <iostream>

using namespace graph;

TEST(UndirectedAdjacencyMatrixTest, AddVertexEdgeBasic) {
    UndirectedAdjacencyMatrix<int, double> graph(10);
    graph.addVertex(1).addVertex(2);
    EXPECT_EQ(graph.vertexCount(), 2);
    graph.addEdge(1, 2, 1.5);
    EXPECT_TRUE(graph.hasEdge(1, 2));
    EXPECT_TRUE(graph.hasEdge(2, 1));
    EXPECT_DOUBLE_EQ(graph.getEdgeWeight(1, 2), 1.5);
}

TEST(UndirectedAdjacencyMatrixTest, RemoveVertexEdge) {
    UndirectedAdjacencyMatrix<int, double> graph(10);
    graph.addVertex(1).addVertex(2);
    graph.addEdge(1, 2, 2.5);
    graph.removeEdge(1, 2);
    EXPECT_FALSE(graph.hasEdge(1, 2));
    graph.removeVertex(1);
    EXPECT_FALSE(graph.hasVertex(1));
}

TEST(UndirectedAdjacencyMatrixTest, DuplicateVertexError) {
    UndirectedAdjacencyMatrix<int, double> graph(10);
    graph.setErrorPolicy(ErrorPolicy::THROW_EXCEPTIONS);
    graph.addVertex(1);
    EXPECT_THROW(graph.addVertex(1), std::exception);
}

TEST(UndirectedAdjacencyMatrixTest, SelfLoopError) {
    UndirectedAdjacencyMatrix<int, double> graph(10);
    graph.setErrorPolicy(ErrorPolicy::THROW_EXCEPTIONS);
    graph.addVertex(1);
    EXPECT_THROW(graph.addEdge(1, 1, 1.0), std::exception);
}

TEST(UndirectedAdjacencyMatrixTest, RemoveEdgeError) {
    UndirectedAdjacencyMatrix<int, double> graph(10);
    graph.setErrorPolicy(ErrorPolicy::THROW_EXCEPTIONS);
    graph.addVertex(1).addVertex(2);
    EXPECT_THROW(graph.removeEdge(1, 2), std::exception);
}

TEST(UndirectedAdjacencyMatrixTest, NeighborsTest) {
    UndirectedAdjacencyMatrix<int, double> graph(10);
    graph.addVertex(1).addVertex(2).addVertex(3);
    graph.addEdge(1, 2, 1.0);
    graph.addEdge(1, 3, 1.2);
    auto neighbors = graph.getNeighbors(1);
    EXPECT_EQ(neighbors.size(), 2);
    EXPECT_TRUE(std::find(neighbors.begin(), neighbors.end(), 2) != neighbors.end());
    EXPECT_TRUE(std::find(neighbors.begin(), neighbors.end(), 3) != neighbors.end());
}

TEST(UndirectedAdjacencyMatrixTest, VertexPerformanceTest) {
    const int count = 100000;
    UndirectedAdjacencyMatrix<int, double> graph(count);

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < count; ++i) {
        graph.addVertex(i);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Time to add " << count << " vertices: " << elapsed.count() << " sec\n";
    EXPECT_EQ(graph.vertexCount(), count);
}

TEST(UndirectedAdjacencyMatrixTest, EdgePerformanceTest) {
    const int count = 100000;
    UndirectedAdjacencyMatrix<int, double> graph(count);
    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < count; ++i) {
        graph.addVertex(i);
    }
    for (int i = 0; i < count - 1; ++i) {
        graph.addEdge(i, i + 1, 1.0);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Time to add " << count - 1 << " edges: " << elapsed.count() << " sec\n";
    EXPECT_EQ(graph.edgeCount(), count - 1);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
