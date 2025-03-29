#include <gtest/gtest.h>

#include "graph/algorithms/matching/hopcroft_karp_matching.hpp"
#include "graph/structure/undirected_adjacency_list.hpp"

TEST(HopcroftKarpMatchingTest, SimpleTest) {
  graph::UndirectedAdjacencyList<int, int> g;

  g.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);
  g.addEdge(1, 4, 1).addEdge(1, 5, 1);
  g.addEdge(2, 4, 1);
  g.addEdge(3, 5, 1);

  std::vector<int> left_part = {1, 2, 3};

  graph::algorithms::matching::HopcroftKarpMatching<
      graph::UndirectedAdjacencyList<int, int>, int>
      matching(g, left_part);

  matching.compute();

  EXPECT_EQ(matching.getMatchingSize(), 2);

  auto result = matching.getMatching();
  EXPECT_EQ(result.size(), 2);

  std::unordered_set<int> matched_left;
  std::unordered_set<int> matched_right;

  for (const auto& [left, right] : result) {
    matched_left.insert(left);
    matched_right.insert(right);
    EXPECT_TRUE(g.hasEdge(left, right));
  }

  EXPECT_EQ(matched_left.size(), 2);

  EXPECT_EQ(matched_right.size(), 2);
  EXPECT_TRUE(matched_right.find(4) != matched_right.end());
  EXPECT_TRUE(matched_right.find(5) != matched_right.end());
}

TEST(HopcroftKarpMatchingTest, AutoBipartiteTest) {
  graph::UndirectedAdjacencyList<int, int> g;

  g.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);
  g.addEdge(1, 4, 1).addEdge(1, 5, 1);
  g.addEdge(2, 4, 1);
  g.addEdge(3, 5, 1);

  graph::algorithms::matching::HopcroftKarpMatching<
      graph::UndirectedAdjacencyList<int, int>, int>
      matching(g);

  matching.compute();

  EXPECT_EQ(matching.getMatchingSize(), 2);

  auto result = matching.getMatching();
  EXPECT_EQ(result.size(), 2);

  for (const auto& [left, right] : result) {
    EXPECT_TRUE(g.hasEdge(left, right));
  }
}

TEST(HopcroftKarpMatchingTest, FullMatchingTest) {
  graph::UndirectedAdjacencyList<int, int> g;

  g.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5).addVertex(
      6);
  g.addEdge(1, 4, 1);
  g.addEdge(2, 5, 1);
  g.addEdge(3, 6, 1);

  std::vector<int> left_part = {1, 2, 3};

  graph::algorithms::matching::HopcroftKarpMatching<
      graph::UndirectedAdjacencyList<int, int>, int>
      matching(g, left_part);

  matching.compute();

  EXPECT_EQ(matching.getMatchingSize(), 3);

  auto result = matching.getMatching();
  EXPECT_EQ(result.size(), 3);

  std::unordered_set<int> matched_left;
  std::unordered_set<int> matched_right;

  for (const auto& [left, right] : result) {
    matched_left.insert(left);
    matched_right.insert(right);
    EXPECT_TRUE(g.hasEdge(left, right));
  }

  EXPECT_EQ(matched_left.size(), 3);
  EXPECT_TRUE(matching.isVertexMatched(1));
  EXPECT_TRUE(matching.isVertexMatched(2));
  EXPECT_TRUE(matching.isVertexMatched(3));
}

TEST(HopcroftKarpMatchingTest, LargeGraphTest) {
  graph::UndirectedAdjacencyList<int, int> g;

  const int N = 50;
  const int M = 50;

  for (int i = 1; i <= N + M; ++i) {
    g.addVertex(i);
  }

  for (int i = 1; i <= N; ++i) {
    for (int j = N + 1; j <= N + M; ++j) {
      if ((i + j) % 3 == 0) {
        g.addEdge(i, j, 1);
      }
    }
  }

  std::vector<int> left_part;
  for (int i = 1; i <= N; ++i) {
    left_part.push_back(i);
  }

  graph::algorithms::matching::HopcroftKarpMatching<
      graph::UndirectedAdjacencyList<int, int>, int>
      matching(g, left_part);

  matching.compute();

  EXPECT_TRUE(matching.getMatchingSize() > 0);
}
