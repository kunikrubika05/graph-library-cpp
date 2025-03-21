#include "graph/algorithms/floyd/floyd.hpp"
#include "graph/structure/directed_adjacency_list.hpp"
#include "graph/structure/directed_adjacency_matrix.hpp"

#include <gtest/gtest.h>
#include <vector>

TEST(FloydWarshallTest, SimpleDirectedGraph) {
    graph::DirectedAdjacencyList<int, int> graph;
    graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
    graph.addEdge(1, 2, 5).addEdge(1, 3, 10);
    graph.addEdge(2, 3, 2).addEdge(2, 4, 1);
    graph.addEdge(3, 4, 4);

    FloydWarshall<graph::DirectedAdjacencyList<int, int>> floyd(graph);
    floyd.compute();

    EXPECT_EQ(floyd.getDistance(1, 2), 5);
    EXPECT_EQ(floyd.getDistance(1, 3), 7);
    EXPECT_EQ(floyd.getDistance(1, 4), 6);

    std::vector<int> path = floyd.retrievePath(1, 4);
    std::vector<int> expected = {1, 2, 4};
    EXPECT_EQ(path, expected);
}

TEST(FloydWarshallTest, NegativeEdges) {
    graph::DirectedAdjacencyList<int, int> graph;
    graph.addVertex(1).addVertex(2).addVertex(3);
    graph.addEdge(1, 2, 3).addEdge(2, 3, 2);
    graph.addEdge(1, 3, 10).addEdge(3, 1, -4);

    FloydWarshall<graph::DirectedAdjacencyList<int, int>> floyd(graph);
    floyd.compute();

    EXPECT_FALSE(floyd.hasNegativeCycle());
    EXPECT_EQ(floyd.getDistance(1, 3), 5);
}

TEST(FloydWarshallTest, NegativeCycle) {
    graph::DirectedAdjacencyList<int, int> graph;
    graph.addVertex(1).addVertex(2).addVertex(3);
    graph.addEdge(1, 2, 1).addEdge(2, 3, 1);
    graph.addEdge(3, 1, -3);

    FloydWarshall<graph::DirectedAdjacencyList<int, int>> floyd(graph);
    floyd.compute();

    EXPECT_TRUE(floyd.hasNegativeCycle());
    EXPECT_THROW(floyd.getDistance(1, 3), std::runtime_error);
}

TEST(FloydWarshallTest, UnreachableVertex) {
    graph::DirectedAdjacencyList<int, int> graph;
    graph.addVertex(1).addVertex(2).addVertex(3);
    graph.addEdge(1, 2, 1);

    FloydWarshall<graph::DirectedAdjacencyList<int, int>> floyd(graph);
    floyd.compute();

    EXPECT_THROW(floyd.getDistance(1, 3), std::runtime_error);
    EXPECT_THROW(floyd.retrievePath(1, 3), std::runtime_error);
}

TEST(FloydWarshallTest, AdjacencyMatrixTest) {
    graph::DirectedAdjacencyMatrix<int, int> graph(10);
    graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
    graph.addEdge(1, 2, 5).addEdge(1, 3, 10);
    graph.addEdge(2, 3, 2).addEdge(2, 4, 1);
    graph.addEdge(3, 4, 4);

    FloydWarshall<graph::DirectedAdjacencyMatrix<int, int>> floyd(graph);
    floyd.compute();

    EXPECT_EQ(floyd.getDistance(1, 2), 5);
    EXPECT_EQ(floyd.getDistance(1, 3), 7);
    EXPECT_EQ(floyd.getDistance(1, 4), 6);

    std::vector<int> path = floyd.retrievePath(1, 4);
    std::vector<int> expected = {1, 2, 4};
    EXPECT_EQ(path, expected);
}

TEST(FloydWarshallTest, GetDistancesTest) {
    graph::DirectedAdjacencyList<int, int> graph;
    graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
    graph.addEdge(1, 2, 5).addEdge(1, 3, 10);
    graph.addEdge(2, 3, 2).addEdge(2, 4, 1);
    graph.addEdge(3, 4, 4);

    FloydWarshall<graph::DirectedAdjacencyList<int, int>> floyd(graph);
    floyd.compute();

    const auto &distances = floyd.getDistances();

    EXPECT_EQ(distances.at(1).at(2), 5);
    EXPECT_EQ(distances.at(1).at(3), 7);
    EXPECT_EQ(distances.at(1).at(4), 6);
    EXPECT_EQ(distances.at(2).at(3), 2);
    EXPECT_EQ(distances.at(2).at(4), 1);
    EXPECT_EQ(distances.at(3).at(4), 4);

    EXPECT_EQ(distances.at(1).at(1), 0);
    EXPECT_EQ(distances.at(2).at(2), 0);

    EXPECT_EQ(distances.at(4).at(1), std::numeric_limits<int>::max());
}

TEST(FloydWarshallTest, GetDistancesWithNegativeEdges) {
    graph::DirectedAdjacencyList<int, int> graph;
    graph.addVertex(1).addVertex(2).addVertex(3);
    graph.addEdge(1, 2, 3).addEdge(2, 3, 2);
    graph.addEdge(1, 3, 10).addEdge(3, 1, -4);

    FloydWarshall<graph::DirectedAdjacencyList<int, int>> floyd(graph);
    floyd.compute();

    const auto &distances = floyd.getDistances();

    EXPECT_EQ(distances.at(1).at(3), 5);
    EXPECT_EQ(distances.at(3).at(2), -1);// Путь 3->1->2
}

TEST(FloydWarshallTest, GetDistancesAdjacencyMatrix) {
    graph::DirectedAdjacencyMatrix<int, int> graph(10);
    graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
    graph.addEdge(1, 2, 5).addEdge(1, 3, 10);
    graph.addEdge(2, 3, 2).addEdge(2, 4, 1);
    graph.addEdge(3, 4, 4);

    FloydWarshall<graph::DirectedAdjacencyMatrix<int, int>> floyd(graph);
    floyd.compute();

    const auto &distances = floyd.getDistances();

    EXPECT_EQ(distances.at(1).at(4), 6);
    EXPECT_EQ(distances.at(3).at(4), 4);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
