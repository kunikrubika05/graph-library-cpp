#include "graph/algorithms/dfs/cycle_detection.hpp"
#include "graph/structure/undirected_adjacency_list.hpp"

#include <gtest/gtest.h>
#include <set>
#include <vector>

using namespace graph;
using namespace graph::algorithms;

TEST(CycleDetectionTest, NoCycle) {
    UndirectedAdjacencyList<int, double> graph;
    graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
    graph.addEdge(1, 2, 1.0).addEdge(2, 3, 1.0).addEdge(3, 4, 1.0);

    CycleDetection<UndirectedAdjacencyList<int, double>> detector(graph);
    EXPECT_FALSE(detector.hasCycle());
    EXPECT_TRUE(detector.getCycle().empty());
}

TEST(CycleDetectionTest, SimpleCycle) {
    UndirectedAdjacencyList<int, double> graph;
    graph.addVertex(1).addVertex(2).addVertex(3);
    graph.addEdge(1, 2, 1.0).addEdge(2, 3, 1.0).addEdge(3, 1, 1.0);

    CycleDetection<UndirectedAdjacencyList<int, double>> detector(graph);
    EXPECT_TRUE(detector.hasCycle());

    auto cycle = detector.getCycle();
    EXPECT_FALSE(cycle.empty());

    EXPECT_EQ(cycle.front(), cycle.back());
}

TEST(CycleDetectionTest, MultipleComponents) {
    UndirectedAdjacencyList<int, double> graph;
    graph.addVertex(1).addVertex(2).addVertex(3);
    graph.addVertex(4).addVertex(5);

    graph.addEdge(1, 2, 1.0).addEdge(2, 3, 1.0).addEdge(3, 1, 1.0);
    graph.addEdge(4, 5, 1.0);

    CycleDetection<UndirectedAdjacencyList<int, double>> detector(graph);
    EXPECT_TRUE(detector.hasCycle());
}

TEST(CycleDetectionTest, ComplexCycle) {
    UndirectedAdjacencyList<int, double> graph;
    graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);
    graph.addEdge(1, 2, 1.0).addEdge(2, 3, 1.0).addEdge(3, 4, 1.0).addEdge(4, 5, 1.0).addEdge(5, 2, 1.0);

    CycleDetection<UndirectedAdjacencyList<int, double>> detector(graph);
    EXPECT_TRUE(detector.hasCycle());

    auto cycle = detector.getCycle();

    EXPECT_FALSE(cycle.empty());
    EXPECT_EQ(cycle.front(), cycle.back());

    std::set<int> cycle_vertices(cycle.begin(), cycle.end() - 1);

    EXPECT_TRUE(cycle_vertices.count(2) > 0);
    EXPECT_TRUE(cycle_vertices.count(3) > 0);
    EXPECT_TRUE(cycle_vertices.count(4) > 0);
    EXPECT_TRUE(cycle_vertices.count(5) > 0);
}

TEST(CycleDetectionTest, EmptyGraph) {
    UndirectedAdjacencyList<int, double> graph;

    CycleDetection<UndirectedAdjacencyList<int, double>> detector(graph);
    EXPECT_FALSE(detector.hasCycle());
    EXPECT_TRUE(detector.getCycle().empty());
}

TEST(CycleDetectionTest, SingleVertex) {
    UndirectedAdjacencyList<int, double> graph;
    graph.addVertex(1);

    CycleDetection<UndirectedAdjacencyList<int, double>> detector(graph);
    EXPECT_FALSE(detector.hasCycle());
    EXPECT_TRUE(detector.getCycle().empty());
}
