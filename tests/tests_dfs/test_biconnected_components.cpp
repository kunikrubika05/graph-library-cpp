#include "graph/algorithms/dfs/biconnected_components.hpp"
#include "graph/structure/undirected_adjacency_list.hpp"

#include <gtest/gtest.h>

using namespace graph;
using namespace graph::algorithms;

template<typename VertexType>
bool edgeInComponent(const std::pair<VertexType, VertexType> &edge,
                     const std::vector<std::pair<VertexType, VertexType>> &component) {
    for (const auto &e: component) {
        if ((e.first == edge.first && e.second == edge.second) ||
            (e.first == edge.second && e.second == edge.first)) {
            return true;
        }
    }
    return false;
}

TEST(BiconnectedComponentsTest, EmptyGraph) {
    UndirectedAdjacencyList<int, int> graph;
    BiconnectedComponents<UndirectedAdjacencyList<int, int>> bc(graph);

    bc.findComponents();

    EXPECT_TRUE(bc.getComponents().empty());
}

TEST(BiconnectedComponentsTest, SingleEdge) {
    UndirectedAdjacencyList<int, int> graph;
    graph.addVertex(1).addVertex(2).addEdge(1, 2, 1);

    BiconnectedComponents<UndirectedAdjacencyList<int, int>> bc(graph);
    bc.findComponents();

    EXPECT_EQ(1, bc.getComponents().size());
    EXPECT_EQ(1, bc.getComponents()[0].size());
    EXPECT_TRUE(edgeInComponent(std::make_pair(1, 2), bc.getComponents()[0]));
}

TEST(BiconnectedComponentsTest, Triangle) {
    UndirectedAdjacencyList<int, int> graph;
    graph.addVertex(1).addVertex(2).addVertex(3).addEdge(1, 2, 1).addEdge(2, 3, 1).addEdge(3, 1, 1);

    BiconnectedComponents<UndirectedAdjacencyList<int, int>> bc(graph);
    bc.findComponents();

    EXPECT_EQ(1, bc.getComponents().size());
    EXPECT_EQ(3, bc.getComponents()[0].size());
}

TEST(BiconnectedComponentsTest, TwoComponents) {
    UndirectedAdjacencyList<int, int> graph;
    graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5).addEdge(1, 2, 1).addEdge(2, 3, 1).addEdge(3, 1, 1)

            .addEdge(3, 4, 1)

            .addEdge(4, 5, 1)
            .addEdge(5, 4, 1);

    BiconnectedComponents<UndirectedAdjacencyList<int, int>> bc(graph);
    bc.findComponents();

    EXPECT_EQ(3, bc.getComponents().size());

    bool found_triangle = false;
    bool found_bridge = false;
    bool found_edge = false;

    for (const auto &component: bc.getComponents()) {
        if (component.size() == 3) {
            found_triangle = true;
            EXPECT_TRUE(edgeInComponent(std::make_pair(1, 2), component));
            EXPECT_TRUE(edgeInComponent(std::make_pair(2, 3), component));
            EXPECT_TRUE(edgeInComponent(std::make_pair(3, 1), component));
        } else if (component.size() == 1) {
            if (edgeInComponent(std::make_pair(3, 4), component)) {
                found_bridge = true;
            } else if (edgeInComponent(std::make_pair(4, 5), component)) {
                found_edge = true;
            }
        }
    }

    EXPECT_TRUE(found_triangle);
    EXPECT_TRUE(found_bridge);
    EXPECT_TRUE(found_edge);
}

TEST(BiconnectedComponentsTest, Necklace) {
    UndirectedAdjacencyList<int, int> graph;
    graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5).addVertex(6).addEdge(1, 2, 1).addEdge(2, 3, 1).addEdge(3, 1, 1)

            .addEdge(3, 4, 1)

            .addEdge(4, 5, 1)
            .addEdge(5, 6, 1)
            .addEdge(6, 4, 1);

    BiconnectedComponents<UndirectedAdjacencyList<int, int>> bc(graph);
    bc.findComponents();

    EXPECT_EQ(3, bc.getComponents().size());

    int cycle_count = 0;
    int bridge_count = 0;

    for (const auto &component: bc.getComponents()) {
        if (component.size() == 3) {
            cycle_count++;
        } else if (component.size() == 1) {
            bridge_count++;
            EXPECT_TRUE(edgeInComponent(std::make_pair(3, 4), component));
        }
    }

    EXPECT_EQ(2, cycle_count);
    EXPECT_EQ(1, bridge_count);
}

TEST(BiconnectedComponentsTest, DisconnectedGraph) {
    UndirectedAdjacencyList<int, int> graph;
    graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5).addVertex(6).addEdge(1, 2, 1).addEdge(2, 3, 1).addEdge(3, 1, 1)

            .addEdge(4, 5, 1)
            .addEdge(5, 6, 1)
            .addEdge(6, 4, 1);

    BiconnectedComponents<UndirectedAdjacencyList<int, int>> bc(graph);
    bc.findComponents();

    EXPECT_EQ(2, bc.getComponents().size());
    EXPECT_EQ(3, bc.getComponents()[0].size());
    EXPECT_EQ(3, bc.getComponents()[1].size());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
