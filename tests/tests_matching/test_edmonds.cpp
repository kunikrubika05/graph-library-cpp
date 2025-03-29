#include <gtest/gtest.h>

#include "graph/algorithms/matching/edmonds_matching.hpp"
#include "graph/structure/undirected_adjacency_list.hpp"

TEST(EdmondsMatchingTest, TinyTest) {
  graph::UndirectedAdjacencyList<int, int> g;

  g.addVertex(1).addVertex(2).addVertex(3);
  g.addEdge(1, 2, 1).addEdge(2, 3, 1);

  graph::algorithms::matching::EdmondsMatching<
      graph::UndirectedAdjacencyList<int, int>, int>
      matching(g);

  matching.compute();

  EXPECT_EQ(matching.getMatchingSize(), 1);

  auto result = matching.getMatching();
  EXPECT_EQ(result.size(), 1);

  for (const auto& [v1, v2] : result) {
    EXPECT_TRUE(g.hasEdge(v1, v2));
  }
}

TEST(EdmondsMatchingTest, SimpleTest) {
  graph::UndirectedAdjacencyList<int, int> g;

  g.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);
  g.addEdge(1, 2, 1).addEdge(2, 3, 1);
  g.addEdge(3, 4, 1).addEdge(4, 5, 1);
  g.addEdge(5, 1, 1);

  graph::algorithms::matching::EdmondsMatching<
      graph::UndirectedAdjacencyList<int, int>, int>
      matching(g);

  matching.compute();

  EXPECT_EQ(matching.getMatchingSize(), 2);

  auto result = matching.getMatching();
  EXPECT_EQ(result.size(), 2);

  std::unordered_set<int> matched_vertices;
  for (const auto& [v1, v2] : result) {
    EXPECT_TRUE(g.hasEdge(v1, v2));
    matched_vertices.insert(v1);
    matched_vertices.insert(v2);
  }

  EXPECT_EQ(matched_vertices.size(), 4);
}

TEST(EdmondsMatchingTest, BlossomTest) {
  graph::UndirectedAdjacencyList<int, int> g;

  g.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);
  g.addEdge(1, 2, 1).addEdge(2, 3, 1);
  g.addEdge(3, 4, 1).addEdge(4, 5, 1);
  g.addEdge(5, 1, 1);
  g.addEdge(1, 3, 1);

  graph::algorithms::matching::EdmondsMatching<
      graph::UndirectedAdjacencyList<int, int>, int>
      matching(g);

  matching.compute();

  EXPECT_EQ(matching.getMatchingSize(), 2);

  auto result = matching.getMatching();
  EXPECT_EQ(result.size(), 2);

  std::unordered_set<int> matched_vertices;
  for (const auto& [v1, v2] : result) {
    EXPECT_TRUE(g.hasEdge(v1, v2));
    matched_vertices.insert(v1);
    matched_vertices.insert(v2);
  }

  EXPECT_EQ(matched_vertices.size(), 4);
}

TEST(EdmondsMatchingTest, CompleteGraphTest) {
  graph::UndirectedAdjacencyList<int, int> g;

  int n = 6;
  for (int i = 1; i <= n; ++i) {
    g.addVertex(i);
  }

  for (int i = 1; i <= n; ++i) {
    for (int j = i + 1; j <= n; ++j) {
      g.addEdge(i, j, 1);
    }
  }

  graph::algorithms::matching::EdmondsMatching<
      graph::UndirectedAdjacencyList<int, int>, int>
      matching(g);

  matching.compute();

  EXPECT_EQ(matching.getMatchingSize(), n / 2);

  auto result = matching.getMatching();
  EXPECT_EQ(result.size(), n / 2);

  std::unordered_set<int> matched_vertices;
  for (const auto& [v1, v2] : result) {
    EXPECT_TRUE(g.hasEdge(v1, v2));
    matched_vertices.insert(v1);
    matched_vertices.insert(v2);
  }

  EXPECT_EQ(matched_vertices.size(), n);
}

TEST(EdmondsMatchingTest, PathGraphTest) {
  graph::UndirectedAdjacencyList<int, int> g;

  int n = 7;
  for (int i = 1; i <= n; ++i) {
    g.addVertex(i);
  }

  for (int i = 1; i < n; ++i) {
    g.addEdge(i, i + 1, 1);
  }

  graph::algorithms::matching::EdmondsMatching<
      graph::UndirectedAdjacencyList<int, int>, int>
      matching(g);

  matching.compute();

  int expected = n / 2;

  EXPECT_EQ(matching.getMatchingSize(), expected);

  auto result = matching.getMatching();
  EXPECT_EQ(result.size(), expected);

  std::unordered_set<int> matched_vertices;
  for (const auto& [v1, v2] : result) {
    EXPECT_TRUE(g.hasEdge(v1, v2));
    matched_vertices.insert(v1);
    matched_vertices.insert(v2);
  }

  EXPECT_EQ(matched_vertices.size(), 2 * expected);
}

TEST(EdmondsMatchingTest, DisconnectedGraphTest) {
  graph::UndirectedAdjacencyList<int, int> g;

  // Создаем две отдельные компоненты
  g.addVertex(1).addVertex(2).addVertex(3);
  g.addVertex(4).addVertex(5).addVertex(6);

  g.addEdge(1, 2, 1).addEdge(2, 3, 1);
  g.addEdge(4, 5, 1).addEdge(5, 6, 1);

  graph::algorithms::matching::EdmondsMatching<
      graph::UndirectedAdjacencyList<int, int>, int>
      matching(g);

  matching.compute();

  EXPECT_EQ(matching.getMatchingSize(), 2);

  auto result = matching.getMatching();
  EXPECT_EQ(result.size(), 2);

  std::unordered_set<int> matched_vertices;
  for (const auto& [v1, v2] : result) {
    EXPECT_TRUE(g.hasEdge(v1, v2));
    matched_vertices.insert(v1);
    matched_vertices.insert(v2);
  }

  EXPECT_EQ(matched_vertices.size(), 4);
}
