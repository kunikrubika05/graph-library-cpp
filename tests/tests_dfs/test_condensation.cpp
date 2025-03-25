#include <gtest/gtest.h>

#include <algorithm>
#include <unordered_set>
#include <vector>

#include "graph/algorithms/dfs/graph_condensation.hpp"
#include "graph/structure/directed_adjacency_list.hpp"

using namespace graph;
using namespace graph::algorithms;

bool componentsEqual(const std::vector<std::vector<int>> &actual,
                     const std::vector<std::vector<int>> &expected) {
  if (actual.size() != expected.size()) return false;

  std::vector<std::unordered_set<int>> actual_sets;
  for (const auto &comp : actual) {
    actual_sets.push_back(std::unordered_set<int>(comp.begin(), comp.end()));
  }

  for (const auto &exp_comp : expected) {
    std::unordered_set<int> exp_set(exp_comp.begin(), exp_comp.end());
    bool found = false;
    for (const auto &act_set : actual_sets) {
      if (act_set == exp_set) {
        found = true;
        break;
      }
    }
    if (!found) return false;
  }

  return true;
}

TEST(GraphCondensationTest, EmptyGraph) {
  DirectedAdjacencyList<int, double> graph;
  GraphCondensation<DirectedAdjacencyList<int, double>> condensation(graph);
  condensation.buildCondensation();
  EXPECT_TRUE(condensation.getCondensedVertices().empty());
  EXPECT_TRUE(condensation.getCondensedEdges().empty());
}

TEST(GraphCondensationTest, SingleVertex) {
  DirectedAdjacencyList<int, double> graph;
  graph.addVertex(1);

  GraphCondensation<DirectedAdjacencyList<int, double>> condensation(graph);
  condensation.buildCondensation();

  EXPECT_EQ(condensation.getCondensedVertices().size(), 1);
  EXPECT_EQ(condensation.getCondensedVertices()[0].size(), 1);
  EXPECT_EQ(condensation.getCondensedVertices()[0][0], 1);
  EXPECT_TRUE(condensation.getCondensedEdges().empty());
}

TEST(GraphCondensationTest, TwoSeparateVertices) {
  DirectedAdjacencyList<int, double> graph;
  graph.addVertex(1).addVertex(2);

  GraphCondensation<DirectedAdjacencyList<int, double>> condensation(graph);
  condensation.buildCondensation();

  EXPECT_EQ(condensation.getCondensedVertices().size(), 2);
  EXPECT_TRUE(componentsEqual(condensation.getCondensedVertices(), {{1}, {2}}));
  EXPECT_TRUE(condensation.getCondensedEdges().empty());
}

TEST(GraphCondensationTest, SimpleDirectedEdge) {
  DirectedAdjacencyList<int, double> graph;
  graph.addVertex(1).addVertex(2);
  graph.addEdge(1, 2, 1.0);

  GraphCondensation<DirectedAdjacencyList<int, double>> condensation(graph);
  condensation.buildCondensation();

  EXPECT_EQ(condensation.getCondensedVertices().size(), 2);
  EXPECT_TRUE(componentsEqual(condensation.getCondensedVertices(), {{1}, {2}}));

  EXPECT_EQ(condensation.getCondensedEdges().size(), 1);
  EXPECT_EQ(condensation.getComponentId(1),
            condensation.getCondensedEdges()[0].first);
  EXPECT_EQ(condensation.getComponentId(2),
            condensation.getCondensedEdges()[0].second);
}

TEST(GraphCondensationTest, SimpleCycle) {
  DirectedAdjacencyList<int, double> graph;
  graph.addVertex(1).addVertex(2).addVertex(3);
  graph.addEdge(1, 2, 1.0).addEdge(2, 3, 1.0).addEdge(3, 1, 1.0);

  GraphCondensation<DirectedAdjacencyList<int, double>> condensation(graph);
  condensation.buildCondensation();

  EXPECT_EQ(condensation.getCondensedVertices().size(), 1);
  EXPECT_TRUE(
      componentsEqual(condensation.getCondensedVertices(), {{1, 2, 3}}));
  EXPECT_TRUE(condensation.getCondensedEdges().empty());
}

TEST(GraphCondensationTest, ComplexGraph) {
  DirectedAdjacencyList<int, double> graph;
  for (int i = 1; i <= 6; ++i) {
    graph.addVertex(i);
  }
  graph.addEdge(1, 2, 1.0);
  graph.addEdge(2, 3, 1.0);
  graph.addEdge(3, 4, 1.0);
  graph.addEdge(3, 5, 1.0);
  graph.addEdge(5, 6, 1.0);
  graph.addEdge(6, 2, 1.0);

  GraphCondensation<DirectedAdjacencyList<int, double>> condensation(graph);
  condensation.buildCondensation();

  EXPECT_EQ(condensation.getCondensedVertices().size(), 3);
  EXPECT_TRUE(componentsEqual(condensation.getCondensedVertices(),
                              {{1}, {2, 3, 5, 6}, {4}}));

  EXPECT_EQ(condensation.getCondensedEdges().size(), 2);
}

TEST(GraphCondensationTest, MultipleComponents) {
  DirectedAdjacencyList<int, double> graph;
  for (int i = 1; i <= 10; ++i) {
    graph.addVertex(i);
  }

  graph.addEdge(1, 2, 1.0);
  graph.addEdge(2, 3, 1.0);
  graph.addEdge(2, 5, 1.0);
  graph.addEdge(5, 6, 1.0);
  graph.addEdge(6, 1, 1.0);

  graph.addEdge(5, 4, 1.0);

  graph.addEdge(7, 8, 1.0);
  graph.addEdge(8, 9, 1.0);
  graph.addEdge(9, 10, 1.0);
  graph.addEdge(10, 7, 1.0);

  GraphCondensation<DirectedAdjacencyList<int, double>> condensation(graph);
  condensation.buildCondensation();

  EXPECT_EQ(condensation.getCondensedVertices().size(), 4);
  EXPECT_TRUE(componentsEqual(condensation.getCondensedVertices(),
                              {{1, 2, 5, 6}, {3}, {4}, {7, 8, 9, 10}}));

  EXPECT_EQ(condensation.getCondensedEdges().size(), 2);
}
