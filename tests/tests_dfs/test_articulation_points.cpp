#include "graph/algorithms/dfs/articulation_points.hpp"
#include "graph/structure/undirected_adjacency_list.hpp"

#include <gtest/gtest.h>
#include <algorithm>
#include <vector>

using namespace graph;
using namespace graph::algorithms;

TEST(ArticulationPointsTest, EmptyGraph) {
    UndirectedAdjacencyList<int, double> graph;
    ArticulationPoints<UndirectedAdjacencyList<int, double>> ap_finder(graph);
    ap_finder.findArticulationPoints();
    EXPECT_TRUE(ap_finder.getArticulationPoints().empty());
}

TEST(ArticulationPointsTest, SingleVertex) {
    UndirectedAdjacencyList<int, double> graph;
    graph.addVertex(1);
    ArticulationPoints<UndirectedAdjacencyList<int, double>> ap_finder(graph);
    ap_finder.findArticulationPoints();
    EXPECT_TRUE(ap_finder.getArticulationPoints().empty());
}

TEST(ArticulationPointsTest, TwoVerticesNoArticulation) {
    UndirectedAdjacencyList<int, double> graph;
    graph.addVertex(1).addVertex(2);
    graph.addEdge(1, 2, 1.0);
    ArticulationPoints<UndirectedAdjacencyList<int, double>> ap_finder(graph);
    ap_finder.findArticulationPoints();
    EXPECT_TRUE(ap_finder.getArticulationPoints().empty());
}

TEST(ArticulationPointsTest, SimpleChain) {
    UndirectedAdjacencyList<int, double> graph;
    graph.addVertex(1).addVertex(2).addVertex(3);
    graph.addEdge(1, 2, 1.0).addEdge(2, 3, 1.0);
    ArticulationPoints<UndirectedAdjacencyList<int, double>> ap_finder(graph);
    ap_finder.findArticulationPoints();
    auto result = ap_finder.getArticulationPoints();
    std::sort(result.begin(), result.end());
    EXPECT_EQ(result.size(), 1);
    EXPECT_EQ(result[0], 2);
}

TEST(ArticulationPointsTest, CycleNoArticulation) {
    UndirectedAdjacencyList<int, double> graph;
    graph.addVertex(1).addVertex(2).addVertex(3);
    graph.addEdge(1, 2, 1.0).addEdge(2, 3, 1.0).addEdge(3, 1, 1.0);
    ArticulationPoints<UndirectedAdjacencyList<int, double>> ap_finder(graph);
    ap_finder.findArticulationPoints();
    EXPECT_TRUE(ap_finder.getArticulationPoints().empty());
}

TEST(ArticulationPointsTest, ComplexGraph) {
    UndirectedAdjacencyList<int, double> graph;
    for (int i = 1; i <= 6; ++i) {
        graph.addVertex(i);
    }
    graph.addEdge(1, 2, 1.0).addEdge(1, 3, 1.0);
    graph.addEdge(2, 4, 1.0);
    graph.addEdge(3, 5, 1.0);
    graph.addEdge(4, 5, 1.0);
    graph.addEdge(4, 6, 1.0);

    ArticulationPoints<UndirectedAdjacencyList<int, double>> ap_finder(graph);
    ap_finder.findArticulationPoints();
    auto result = ap_finder.getArticulationPoints();
    std::sort(result.begin(), result.end());
    std::vector<int> expected = {4};
    EXPECT_EQ(result, expected);
}

TEST(ArticulationPointsTest, RootIsArticulationPoint) {
    UndirectedAdjacencyList<int, double> graph;
    for (int i = 1; i <= 5; ++i) {
        graph.addVertex(i);
    }
    graph.addEdge(1, 2, 1.0).addEdge(1, 3, 1.0);
    graph.addEdge(2, 4, 1.0).addEdge(2, 5, 1.0);

    ArticulationPoints<UndirectedAdjacencyList<int, double>> ap_finder(graph);
    ap_finder.findArticulationPoints();
    auto result = ap_finder.getArticulationPoints();
    std::sort(result.begin(), result.end());
    std::vector<int> expected = {1, 2};
    EXPECT_EQ(result, expected);
}

TEST(ArticulationPointsTest, MultipleArticulationPoints) {
    UndirectedAdjacencyList<int, double> graph;
    for (int i = 1; i <= 8; ++i) {
        graph.addVertex(i);
    }
    graph.addEdge(1, 2, 1.0).addEdge(1, 3, 1.0);
    graph.addEdge(2, 4, 1.0);
    graph.addEdge(3, 5, 1.0);
    graph.addEdge(4, 5, 1.0);
    graph.addEdge(4, 6, 1.0);
    graph.addEdge(5, 7, 1.0);
    graph.addEdge(7, 8, 1.0);

    ArticulationPoints<UndirectedAdjacencyList<int, double>> ap_finder(graph);
    ap_finder.findArticulationPoints();
    auto result = ap_finder.getArticulationPoints();
    std::sort(result.begin(), result.end());
    std::vector<int> expected = {4, 5, 7};
    EXPECT_EQ(result, expected);
}
