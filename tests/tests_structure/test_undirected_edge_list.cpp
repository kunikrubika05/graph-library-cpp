#include "graph/structure/undirected_edge_list.hpp"

#include <algorithm>
#include <chrono>
#include <gtest/gtest.h>
#include <iostream>
#include <string>

using namespace graph;

TEST(UndirectedEdgeListTest, AddVertexEdgeBasic) {
    UndirectedEdgeList<std::string, double> graph;
    graph.addVertex("Alice").addVertex("Bob");
    EXPECT_EQ(graph.vertexCount(), 2);
    graph.addEdge("Alice", "Bob", 3.14);
    EXPECT_TRUE(graph.hasEdge("Alice", "Bob"));
    EXPECT_DOUBLE_EQ(graph.getEdgeWeight("Alice", "Bob"), 3.14);
}

TEST(UndirectedEdgeListTest, DuplicateVertexError) {
    UndirectedEdgeList<std::string, double> graph;
    graph.setErrorPolicy(ErrorPolicy::THROW_EXCEPTIONS);
    graph.addVertex("Alice");
    EXPECT_THROW(graph.addVertex("Alice"), std::exception);
}

TEST(UndirectedEdgeListTest, VertexNotFoundError) {
    UndirectedEdgeList<std::string, double> graph;
    graph.setErrorPolicy(ErrorPolicy::THROW_EXCEPTIONS);
    EXPECT_THROW(graph.addEdge("Alice", "Bob", 1.0), std::exception);
}

TEST(UndirectedEdgeListTest, SelfLoopError) {
    GraphOptions options(false, false);
    UndirectedEdgeList<std::string, double> graph(options, ErrorPolicy::THROW_EXCEPTIONS);
    graph.addVertex("Alice");
    EXPECT_THROW(graph.addEdge("Alice", "Alice", 2.0), std::exception);
}

TEST(UndirectedEdgeListTest, RemoveEdgeError) {
    UndirectedEdgeList<std::string, double> graph;
    graph.setErrorPolicy(ErrorPolicy::THROW_EXCEPTIONS);
    graph.addVertex("Alice").addVertex("Bob");
    EXPECT_THROW(graph.removeEdge("Alice", "Bob"), std::exception);
}

TEST(UndirectedEdgeListTest, RemoveVertexAlsoRemovesEdges) {
    UndirectedEdgeList<std::string, double> graph;
    graph.addVertex("Alice").addVertex("Bob").addVertex("Charlie");
    graph.addEdge("Alice", "Bob", 1.0);
    graph.addEdge("Bob", "Charlie", 1.5);
    graph.removeVertex("Bob");
    EXPECT_FALSE(graph.hasVertex("Bob"));
    EXPECT_FALSE(graph.hasEdge("Alice", "Bob"));
    EXPECT_FALSE(graph.hasEdge("Bob", "Charlie"));
}

TEST(UndirectedEdgeListTest, GetNeighborsTest) {
    UndirectedEdgeList<std::string, double> graph;
    graph.addVertex("Alice").addVertex("Bob").addVertex("Charlie").addVertex("Dave");
    graph.addEdge("Alice", "Bob", 1.0);
    graph.addEdge("Alice", "Charlie", 1.1);
    auto neighbors = graph.getNeighbors("Alice");
    EXPECT_EQ(neighbors.size(), 2);
    EXPECT_TRUE(std::find(neighbors.begin(), neighbors.end(), "Bob") != neighbors.end());
    EXPECT_TRUE(std::find(neighbors.begin(), neighbors.end(), "Charlie") != neighbors.end());
}

TEST(UndirectedEdgeListTest, PerformanceTest) {
    const int count = 50000;
    UndirectedEdgeList<int, double> graph;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < count; ++i) {
        graph.addVertex(i);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Time to add " << count << " vertices: " << elapsed.count() << " sec\n";
    EXPECT_EQ(graph.vertexCount(), count);
}

TEST(UndirectedEdgeListTest, DuplicateEdgeCheck) {
    GraphOptions options(true, true);
    UndirectedEdgeList<std::string, double> graph(options, ErrorPolicy::THROW_EXCEPTIONS);
    graph.addVertex("Alice").addVertex("Bob");
    graph.addEdge("Alice", "Bob", 1.0);
    EXPECT_THROW(graph.addEdge("Bob", "Alice", 2.0), std::exception);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
