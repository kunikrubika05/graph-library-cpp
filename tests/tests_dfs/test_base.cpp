#include "graph/algorithms/dfs/depth_first_search.hpp"
#include "graph/structure/undirected_adjacency_list.hpp"

#include <gtest/gtest.h>
#include <vector>
#include <string>

using namespace graph;
using namespace graph::algorithms;

TEST(DepthFirstSearchTest, SingleSourceTraversal) {
    UndirectedAdjacencyList<int, double> graph;
    graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
    graph.addEdge(1, 2, 1.0).addEdge(2, 3, 1.0).addEdge(3, 4, 1.0);

    DepthFirstSearch<UndirectedAdjacencyList<int, double>> dfs(graph);
    dfs.run(1);

    auto visited = dfs.getVisited();
    EXPECT_TRUE(visited[1]);
    EXPECT_TRUE(visited[2]);
    EXPECT_TRUE(visited[3]);
    EXPECT_TRUE(visited[4]);
}

TEST(DepthFirstSearchTest, VertexHandler) {
    UndirectedAdjacencyList<int, double> graph;
    graph.addVertex(1).addVertex(2).addVertex(3);
    graph.addEdge(1, 2, 1.0).addEdge(1, 3, 1.0);

    std::vector<int> visited_vertices;
    DepthFirstSearch<UndirectedAdjacencyList<int, double>> dfs(graph);
    dfs.setVertexHandler([&visited_vertices](const int& vertex) {
        visited_vertices.push_back(vertex);
    });

    dfs.run(1);

    EXPECT_EQ(visited_vertices.size(), 3);
    EXPECT_EQ(visited_vertices[0], 1);
}

TEST(DepthFirstSearchTest, DisconnectedGraph) {
    UndirectedAdjacencyList<int, double> graph;
    graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);
    graph.addEdge(1, 2, 1.0).addEdge(4, 5, 1.0);

    DepthFirstSearch<UndirectedAdjacencyList<int, double>> dfs(graph);
    dfs.runOnAll();

    auto visited = dfs.getVisited();
    EXPECT_TRUE(visited[1]);
    EXPECT_TRUE(visited[2]);
    EXPECT_TRUE(visited[3]);
    EXPECT_TRUE(visited[4]);
    EXPECT_TRUE(visited[5]);
}
