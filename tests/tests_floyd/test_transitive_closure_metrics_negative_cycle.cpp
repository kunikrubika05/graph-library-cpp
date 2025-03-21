#include "graph/algorithms/floyd/graph_metrics.hpp"
#include "graph/algorithms/floyd/negative_cycle_detector.hpp"
#include "graph/algorithms/floyd/transitive_closure.hpp"
#include "graph/structure/directed_adjacency_list.hpp"

#include <gtest/gtest.h>
#include <unordered_set>

TEST(TransitiveClosureTest, SimpleGraph) {
    graph::DirectedAdjacencyList<int, int> graph;
    graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
    graph.addEdge(1, 2, 1).addEdge(2, 3, 1).addEdge(3, 4, 1);

    TransitiveClosure<graph::DirectedAdjacencyList<int, int>> tc(graph);
    tc.compute();

    EXPECT_TRUE(tc.isReachable(1, 2));
    EXPECT_TRUE(tc.isReachable(1, 3));
    EXPECT_TRUE(tc.isReachable(1, 4));
    EXPECT_TRUE(tc.isReachable(2, 3));
    EXPECT_TRUE(tc.isReachable(2, 4));
    EXPECT_TRUE(tc.isReachable(3, 4));

    EXPECT_FALSE(tc.isReachable(4, 1));
    EXPECT_FALSE(tc.isReachable(3, 1));
    EXPECT_FALSE(tc.isReachable(2, 1));
}

TEST(GraphMetricsTest, DiameterAndRadius) {
    graph::DirectedAdjacencyList<int, int> graph;
    graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);
    graph.addEdge(1, 2, 1).addEdge(2, 3, 1);
    graph.addEdge(3, 4, 1).addEdge(4, 5, 1);
    graph.addEdge(5, 1, 1);

    GraphMetrics<graph::DirectedAdjacencyList<int, int>> metrics(graph);

    EXPECT_EQ(metrics.diameter(), 4);
    EXPECT_EQ(metrics.radius(), 4);

    auto center = metrics.center();
    EXPECT_EQ(center.size(), 5);

    for (int i = 1; i <= 5; i++) {
        EXPECT_TRUE(std::find(center.begin(), center.end(), i) != center.end());
    }
}

TEST(NegativeCycleDetectorTest, DetectAndFindCycle) {
    graph::DirectedAdjacencyList<int, int> graph;
    graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
    graph.addEdge(1, 2, 2).addEdge(2, 3, 2);
    graph.addEdge(3, 4, 2).addEdge(4, 1, -8);

    NegativeCycleDetector<graph::DirectedAdjacencyList<int, int>> detector(graph);

    EXPECT_TRUE(detector.hasNegativeCycle());

    auto cycle = detector.findNegativeCycle();
    EXPECT_FALSE(cycle.empty());

    std::unordered_set<int> cycle_vertices(cycle.begin(), cycle.end());
    EXPECT_EQ(cycle_vertices.size(), 4);
}
