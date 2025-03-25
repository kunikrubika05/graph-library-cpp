#include <gtest/gtest.h>

#include <utility>

#include "graph/algorithms/mst/kruskal.hpp"
#include "graph/structure/undirected_adjacency_list.hpp"
#include "graph/structure/undirected_adjacency_matrix.hpp"

using namespace graph;
using namespace graph::algorithms;

TEST(KruskalAdvancedTest, MultipleValidMST) {
  UndirectedAdjacencyList<int, int> graph;

  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, 1);
  graph.addEdge(2, 3, 1);
  graph.addEdge(3, 4, 1);
  graph.addEdge(4, 1, 1);

  Kruskal<UndirectedAdjacencyList<int, int>> kruskal(graph);
  kruskal.compute();

  EXPECT_TRUE(kruskal.isConnected());
  EXPECT_EQ(kruskal.getTotalWeight(), 3);

  auto mst_edges = kruskal.getMSTEdges();
  EXPECT_EQ(mst_edges.size(), 3);

  for (const auto &edge : mst_edges) {
    EXPECT_EQ(edge.weight, 1);
  }
}

TEST(KruskalAdvancedTest, StringVertices) {
  UndirectedAdjacencyList<std::string, int> graph;

  graph.addVertex("Moscow")
      .addVertex("Saint Petersburg")
      .addVertex("Novosibirsk");
  graph.addVertex("Yekaterinburg").addVertex("Kazan");

  graph.addEdge("Moscow", "Saint Petersburg", 634);
  graph.addEdge("Moscow", "Novosibirsk", 2878);
  graph.addEdge("Moscow", "Yekaterinburg", 1418);
  graph.addEdge("Moscow", "Kazan", 815);
  graph.addEdge("Saint Petersburg", "Novosibirsk", 3467);
  graph.addEdge("Novosibirsk", "Yekaterinburg", 1512);
  graph.addEdge("Yekaterinburg", "Kazan", 925);

  Kruskal<UndirectedAdjacencyList<std::string, int>> kruskal(graph);
  kruskal.compute();

  EXPECT_TRUE(kruskal.isConnected());

  auto mst = kruskal.getMSTEdges();
  EXPECT_EQ(mst.size(), 4);

  int expected_weight = 634 + 815 + 925 + 1512;
  EXPECT_EQ(kruskal.getTotalWeight(), expected_weight);
}

TEST(KruskalAdvancedTest, AdjacencyMatrix) {
  UndirectedAdjacencyMatrix<int, int> graph(10);

  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, 5);
  graph.addEdge(1, 3, 10);
  graph.addEdge(1, 4, 3);
  graph.addEdge(2, 3, 2);
  graph.addEdge(2, 4, 7);
  graph.addEdge(3, 4, 8);

  Kruskal<UndirectedAdjacencyMatrix<int, int>> kruskal(graph);
  kruskal.compute();

  EXPECT_TRUE(kruskal.isConnected());
  EXPECT_EQ(kruskal.getTotalWeight(), 10);

  auto mst = kruskal.getMSTEdges();
  EXPECT_EQ(mst.size(), 3);

  std::sort(mst.begin(), mst.end());
  EXPECT_EQ(mst[0].weight, 2);
  EXPECT_EQ(mst[1].weight, 3);
  EXPECT_EQ(mst[2].weight, 5);
}

struct City {
  std::string name;
  int population;

  bool operator==(const City &other) const { return name == other.name; }

  bool operator<(const City &other) const { return name < other.name; }
};

namespace std {
template <>
struct hash<City> {
  std::size_t operator()(const City &city) const {
    return std::hash<std::string>()(city.name);
  }
};
}  // namespace std

TEST(KruskalAdvancedTest, CustomVertexType) {
  UndirectedAdjacencyList<City, double> graph;

  City moscow{"Moscow", 12000000};
  City spb{"Saint Petersburg", 5000000};
  City novosibirsk{"Novosibirsk", 1500000};
  City yekaterinburg{"Yekaterinburg", 1400000};

  graph.addVertex(moscow)
      .addVertex(spb)
      .addVertex(novosibirsk)
      .addVertex(yekaterinburg);

  graph.addEdge(moscow, spb, 634.0);
  graph.addEdge(moscow, novosibirsk, 2878.0);
  graph.addEdge(moscow, yekaterinburg, 1418.0);
  graph.addEdge(spb, novosibirsk, 3467.0);
  graph.addEdge(novosibirsk, yekaterinburg, 1512.0);

  Kruskal<UndirectedAdjacencyList<City, double>> kruskal(graph);
  kruskal.compute();

  EXPECT_TRUE(kruskal.isConnected());
  EXPECT_EQ(kruskal.getMSTEdges().size(), 3);

  double expected_weight = 634.0 + 1418.0 + 1512.0;
  EXPECT_DOUBLE_EQ(kruskal.getTotalWeight(), expected_weight);
}

TEST(KruskalAdvancedTest, DisconnectedComponents) {
  UndirectedAdjacencyList<int, int> graph;

  for (int i = 1; i <= 10; i++) {
    graph.addVertex(i);
  }

  graph.addEdge(1, 2, 1);
  graph.addEdge(2, 3, 2);
  graph.addEdge(3, 4, 3);
  graph.addEdge(4, 5, 4);

  graph.addEdge(6, 7, 5);
  graph.addEdge(7, 8, 6);
  graph.addEdge(8, 9, 7);
  graph.addEdge(9, 10, 8);

  Kruskal<UndirectedAdjacencyList<int, int>> kruskal(graph);
  kruskal.compute();

  EXPECT_FALSE(kruskal.isConnected());
  EXPECT_THROW(kruskal.getTotalWeight(), std::runtime_error);
  EXPECT_THROW(kruskal.getMSTEdges(), std::runtime_error);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
