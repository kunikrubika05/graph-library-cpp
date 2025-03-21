#include "graph/algorithms/bfs/bipartite_checker.hpp"
#include "graph/structure/undirected_adjacency_list.hpp"

#include <gtest/gtest.h>

using namespace graph;
using namespace graph::algorithms;

TEST(BipartiteCheckerTest, BipartiteGraph) {
    UndirectedAdjacencyList<int, int> graph;
    graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
    graph.addEdge(1, 2, 1).addEdge(1, 3, 1).addEdge(2, 4, 1).addEdge(3, 4, 1);

    BipartiteChecker<UndirectedAdjacencyList<int, int>> checker;
    EXPECT_TRUE(checker.isBipartite(graph));
}

TEST(BipartiteCheckerTest, NonBipartiteGraph) {
    UndirectedAdjacencyList<int, int> graph;
    graph.addVertex(1).addVertex(2).addVertex(3);
    graph.addEdge(1, 2, 1).addEdge(2, 3, 1).addEdge(3, 1, 1);

    BipartiteChecker<UndirectedAdjacencyList<int, int>> checker;
    EXPECT_FALSE(checker.isBipartite(graph));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
