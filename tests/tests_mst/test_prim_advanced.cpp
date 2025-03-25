#include <gtest/gtest.h>

#include <utility>

#include "graph/algorithms/mst/kruskal.hpp"
#include "graph/algorithms/mst/prim.hpp"
#include "graph/structure/undirected_adjacency_list.hpp"
#include "graph/structure/undirected_adjacency_matrix.hpp"

using namespace graph;
using namespace graph::algorithms;

TEST(PrimAdvancedTest, CompareWithKruskal) {
  UndirectedAdjacencyList<int, int> graph;

  graph.addVertex(1)
      .addVertex(2)
      .addVertex(3)
      .addVertex(4)
      .addVertex(5)
      .addVertex(6);

  graph.addEdge(1, 2, 6);
  graph.addEdge(1, 3, 1);
  graph.addEdge(1, 4, 5);
  graph.addEdge(2, 3, 5);
  graph.addEdge(2, 5, 3);
  graph.addEdge(3, 4, 5);
  graph.addEdge(3, 5, 6);
  graph.addEdge(3, 6, 4);
  graph.addEdge(4, 6, 2);
  graph.addEdge(5, 6, 6);

  Prim<UndirectedAdjacencyList<int, int>> prim(graph);
  prim.compute();

  Kruskal<UndirectedAdjacencyList<int, int>> kruskal(graph);
  kruskal.compute();

  EXPECT_TRUE(prim.isConnected());
  EXPECT_TRUE(kruskal.isConnected());
  EXPECT_EQ(prim.getTotalWeight(), kruskal.getTotalWeight());
}

TEST(PrimAdvancedTest, MultipleEqualWeights) {
  UndirectedAdjacencyList<int, int> graph;

  for (int i = 1; i <= 6; ++i) {
    graph.addVertex(i);
  }

  graph.addEdge(1, 2, 1);
  graph.addEdge(1, 3, 1);
  graph.addEdge(2, 3, 1);
  graph.addEdge(2, 4, 1);
  graph.addEdge(3, 4, 1);
  graph.addEdge(3, 5, 1);
  graph.addEdge(4, 5, 1);
  graph.addEdge(4, 6, 1);
  graph.addEdge(5, 6, 1);

  Prim<UndirectedAdjacencyList<int, int>> prim(graph);
  prim.compute();

  EXPECT_TRUE(prim.isConnected());
  EXPECT_EQ(prim.getTotalWeight(), 5);
  EXPECT_EQ(prim.getMSTEdges().size(), 5);
}

TEST(PrimAdvancedTest, AdjacencyMatrix) {
  UndirectedAdjacencyMatrix<int, int> graph(10);

  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, 5);
  graph.addEdge(1, 3, 10);
  graph.addEdge(1, 4, 3);
  graph.addEdge(2, 3, 2);
  graph.addEdge(2, 4, 7);
  graph.addEdge(3, 4, 8);

  Prim<UndirectedAdjacencyMatrix<int, int>> prim(graph);
  prim.compute();

  EXPECT_TRUE(prim.isConnected());
  EXPECT_EQ(prim.getTotalWeight(), 10);
  EXPECT_EQ(prim.getMSTEdges().size(), 3);
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

TEST(PrimAdvancedTest, CustomVertexType) {
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

  Prim<UndirectedAdjacencyList<City, double>> prim(graph);
  prim.compute(moscow);

  EXPECT_TRUE(prim.isConnected());
  EXPECT_EQ(prim.getMSTEdges().size(), 3);

  double expected_weight = 634.0 + 1418.0 + 1512.0;
  EXPECT_DOUBLE_EQ(prim.getTotalWeight(), expected_weight);
}

TEST(PrimAdvancedTest, LargeGraph) {
  UndirectedAdjacencyList<int, int> graph;

  const int size = 100;

  for (int i = 0; i < size * size; ++i) {
    graph.addVertex(i);
  }

  for (int i = 0; i < size; ++i) {
    for (int j = 0; j < size - 1; ++j) {
      int vertex1 = i * size + j;
      int vertex2 = i * size + j + 1;
      graph.addEdge(vertex1, vertex2, 1);
    }
  }

  for (int i = 0; i < size - 1; ++i) {
    for (int j = 0; j < size; ++j) {
      int vertex1 = i * size + j;
      int vertex2 = (i + 1) * size + j;
      graph.addEdge(vertex1, vertex2, 2);
    }
  }

  Prim<UndirectedAdjacencyList<int, int>> prim(graph);
  prim.compute(0);

  EXPECT_TRUE(prim.isConnected());
  EXPECT_EQ(prim.getMSTEdges().size(), size * size - 1);

  int expected_weight = size * (size - 1) + (size - 1) * 2;
  EXPECT_EQ(prim.getTotalWeight(), expected_weight);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
