#include "graph/structure/directed_adjacency_matrix.hpp"

#include <chrono>
#include <gtest/gtest.h>
#include <iostream>

using namespace graph;

TEST(DirectedAdjacencyMatrixNoVectorTest, AddVertexEdgeO1) {
    DirectedAdjacencyMatrix<int, double> graph(10);
    graph.addVertex(1).addVertex(2);
    EXPECT_EQ(graph.vertexCount(), 2);
    graph.addEdge(1, 2, 1.5);
    EXPECT_TRUE(graph.hasEdge(1, 2));
    EXPECT_DOUBLE_EQ(graph.getEdgeWeight(1, 2), 1.5);
}

TEST(DirectedAdjacencyMatrixNoVectorTest, RemoveVertexEdgeO1) {
    DirectedAdjacencyMatrix<int, double> graph(10);
    graph.addVertex(1).addVertex(2);
    graph.addEdge(1, 2, 2.5);
    graph.removeEdge(1, 2);
    EXPECT_FALSE(graph.hasEdge(1, 2));
    graph.removeVertex(1);
    EXPECT_FALSE(graph.hasVertex(1));
}

TEST(DirectedAdjacencyMatrixNoVectorTest, ErrorOnDuplicateVertex) {
    DirectedAdjacencyMatrix<int, double> graph(10);
    graph.setErrorPolicy(ErrorPolicy::THROW_EXCEPTIONS);
    graph.addVertex(1);
    EXPECT_THROW(graph.addVertex(1), std::exception);
}

TEST(DirectedAdjacencyMatrixNoVectorTest, SelfLoopError) {
    DirectedAdjacencyMatrix<int, double> graph(10, GraphOptions(), ErrorPolicy::THROW_EXCEPTIONS);
    graph.addVertex(1);
    EXPECT_THROW(graph.addEdge(1, 1, 1.0), std::exception);
}

TEST(DirectedAdjacencyMatrixNoVectorTest, EdgeNotFoundError) {
    DirectedAdjacencyMatrix<int, double> graph(10);
    graph.setErrorPolicy(ErrorPolicy::THROW_EXCEPTIONS);
    graph.addVertex(1).addVertex(2);
    EXPECT_THROW(graph.getEdgeWeight(1, 2), std::exception);
}

TEST(DirectedAdjacencyMatrixNoVectorTest, VertexPerformanceTest) {
    const std::size_t count = 100000;
    DirectedAdjacencyMatrix<int, double> graph(count);

    auto start = std::chrono::high_resolution_clock::now();
    for (std::size_t i = 0; i < count; ++i) {
        graph.addVertex(static_cast<int>(i));
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Time to add " << count << " vertices: " << elapsed.count() << " sec\n";

    EXPECT_EQ(graph.vertexCount(), count);
}

TEST(DirectedAdjacencyMatrixNoVectorTest, EdgePerformanceTest) {
    const std::size_t count = 100000;
    DirectedAdjacencyMatrix<int, double> graph(count);
    auto start = std::chrono::high_resolution_clock::now();

    for (std::size_t i = 0; i < count; ++i) {
        graph.addVertex(static_cast<int>(i));
    }
    for (std::size_t i = 0; i < count - 1; ++i) {
        graph.addEdge(static_cast<int>(i), static_cast<int>(i + 1), 1.0);
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
