#include <gtest/gtest.h>

#include <string>
#include <utility>

#include "graph/structure/graph_converter.hpp"

using namespace graph;

class City {
 public:
  City() : name_(""), population_(0) {}

  explicit City(std::string name, int population)
      : name_(std::move(name)), population_(population) {}

  const std::string& getName() const { return name_; }
  int getPopulation() const { return population_; }

  bool operator==(const City& other) const { return name_ == other.name_; }

  bool operator<(const City& other) const { return name_ < other.name_; }

 private:
  std::string name_;
  int population_;
};

namespace std {
template <>
struct hash<City> {
  size_t operator()(const City& city) const {
    return hash<string>()(city.getName());
  }
};

template <>
struct hash<pair<int, int>> {
  size_t operator()(const pair<int, int>& p) const {
    return hash<int>()(p.first) ^ (hash<int>()(p.second) << 1);
  }
};
}  // namespace std

TEST(GraphConverterTest, DirectedEdgeListToAdjacencyList) {
  DirectedEdgeList<std::string, double> edgeList;
  edgeList.addVertex("Alice").addVertex("Bob").addVertex("Charlie");
  edgeList.addEdge("Alice", "Bob", 1.0);
  edgeList.addEdge("Bob", "Charlie", 2.0);

  auto adjList =
      GraphConverter<std::string, double>::toDirectedAdjacencyList(edgeList);

  EXPECT_EQ(adjList.vertexCount(), 3);
  EXPECT_EQ(adjList.edgeCount(), 2);
  EXPECT_TRUE(adjList.hasEdge("Alice", "Bob"));
  EXPECT_TRUE(adjList.hasEdge("Bob", "Charlie"));
  EXPECT_FALSE(adjList.hasEdge("Alice", "Charlie"));
  EXPECT_DOUBLE_EQ(adjList.getEdgeWeight("Alice", "Bob"), 1.0);
  EXPECT_DOUBLE_EQ(adjList.getEdgeWeight("Bob", "Charlie"), 2.0);
}

TEST(GraphConverterTest, DirectedAdjacencyListToMatrix) {
  DirectedAdjacencyList<std::string, double> adjList;
  adjList.addVertex("Alice").addVertex("Bob").addVertex("Charlie");
  adjList.addEdge("Alice", "Bob", 1.5);
  adjList.addEdge("Bob", "Charlie", 2.5);

  auto adjMatrix =
      GraphConverter<std::string, double>::toDirectedAdjacencyMatrix(adjList,
                                                                     10);

  EXPECT_EQ(adjMatrix.vertexCount(), 3);
  EXPECT_EQ(adjMatrix.edgeCount(), 2);
  EXPECT_TRUE(adjMatrix.hasEdge("Alice", "Bob"));
  EXPECT_TRUE(adjMatrix.hasEdge("Bob", "Charlie"));
  EXPECT_FALSE(adjMatrix.hasEdge("Charlie", "Alice"));
  EXPECT_DOUBLE_EQ(adjMatrix.getEdgeWeight("Alice", "Bob"), 1.5);
  EXPECT_DOUBLE_EQ(adjMatrix.getEdgeWeight("Bob", "Charlie"), 2.5);
}

TEST(GraphConverterTest, UndirectedToDirected) {
  UndirectedEdgeList<std::string, int> undirected;
  undirected.addVertex("Alice").addVertex("Bob").addVertex("Charlie");
  undirected.addEdge("Alice", "Bob", 10);
  undirected.addEdge("Bob", "Charlie", 20);

  auto directed =
      GraphConverter<std::string, int>::toDirectedEdgeList(undirected);

  EXPECT_EQ(directed.vertexCount(), 3);
  EXPECT_EQ(directed.edgeCount(), 4);
  EXPECT_TRUE(directed.hasEdge("Alice", "Bob"));
  EXPECT_TRUE(directed.hasEdge("Bob", "Alice"));
  EXPECT_TRUE(directed.hasEdge("Bob", "Charlie"));
  EXPECT_TRUE(directed.hasEdge("Charlie", "Bob"));
  EXPECT_EQ(directed.getEdgeWeight("Alice", "Bob"), 10);
  EXPECT_EQ(directed.getEdgeWeight("Bob", "Charlie"), 20);
}

TEST(GraphConverterTest, DirectedToUndirected) {
  DirectedEdgeList<std::string, int> directed;
  directed.addVertex("Alice").addVertex("Bob").addVertex("Charlie");
  directed.addEdge("Alice", "Bob", 10);
  directed.addEdge("Bob", "Alice", 15);
  directed.addEdge("Bob", "Charlie", 20);

  auto undirected =
      GraphConverter<std::string, int>::toUndirectedEdgeList(directed);

  EXPECT_EQ(undirected.vertexCount(), 3);
  EXPECT_EQ(undirected.edgeCount(), 2);
  EXPECT_TRUE(undirected.hasEdge("Alice", "Bob"));
  EXPECT_TRUE(undirected.hasEdge("Bob", "Charlie"));
}

TEST(GraphConverterTest, ComplexGraphConversion) {
  DirectedAdjacencyList<std::string, double> original;
  original.addVertex("Москва")
      .addVertex("Санкт-Петербург")
      .addVertex("Новосибирск")
      .addVertex("Екатеринбург")
      .addVertex("Казань");

  original.addEdge("Москва", "Санкт-Петербург", 634.0);
  original.addEdge("Москва", "Новосибирск", 2878.0);
  original.addEdge("Санкт-Петербург", "Москва", 634.0);
  original.addEdge("Новосибирск", "Екатеринбург", 1418.0);
  original.addEdge("Екатеринбург", "Казань", 925.0);

  auto undirectedMatrix =
      GraphConverter<std::string, double>::toUndirectedAdjacencyMatrix(original,
                                                                       10);

  EXPECT_EQ(undirectedMatrix.vertexCount(), 5);
  EXPECT_EQ(undirectedMatrix.edgeCount(), 4);

  EXPECT_TRUE(undirectedMatrix.hasEdge("Москва", "Санкт-Петербург"));
  EXPECT_TRUE(undirectedMatrix.hasEdge("Санкт-Петербург", "Москва"));
  EXPECT_TRUE(undirectedMatrix.hasEdge("Москва", "Новосибирск"));
  EXPECT_TRUE(undirectedMatrix.hasEdge("Новосибирск", "Екатеринбург"));
  EXPECT_TRUE(undirectedMatrix.hasEdge("Екатеринбург", "Казань"));

  auto directedList =
      GraphConverter<std::string, double>::toDirectedEdgeList(undirectedMatrix);

  EXPECT_EQ(directedList.vertexCount(), 5);
  EXPECT_EQ(directedList.edgeCount(), 8);
}

TEST(GraphConverterTest, GraphWithCycles) {
  DirectedAdjacencyList<std::string, int> cyclic;
  cyclic.addVertex("A").addVertex("B").addVertex("C").addVertex("D");
  cyclic.addEdge("A", "B", 1);
  cyclic.addEdge("B", "C", 2);
  cyclic.addEdge("C", "D", 3);
  cyclic.addEdge("D", "A", 4);

  auto undirected =
      GraphConverter<std::string, int>::toUndirectedEdgeList(cyclic);
  EXPECT_EQ(undirected.vertexCount(), 4);
  EXPECT_EQ(undirected.edgeCount(), 4);

  auto directed = GraphConverter<std::string, int>::toDirectedAdjacencyMatrix(
      undirected, 5);
  EXPECT_EQ(directed.vertexCount(), 4);
  EXPECT_EQ(directed.edgeCount(), 8);

  EXPECT_TRUE(directed.hasEdge("A", "B"));
  EXPECT_TRUE(directed.hasEdge("B", "C"));
  EXPECT_TRUE(directed.hasEdge("C", "D"));
  EXPECT_TRUE(directed.hasEdge("D", "A"));
}

TEST(GraphConverterTest, CustomVertexType) {
  City moscow("Moscow", 12000000);
  City spb("Saint Petersburg", 5000000);
  City novosibirsk("Novosibirsk", 1500000);

  DirectedEdgeList<City, double> cityGraph;
  cityGraph.addVertex(moscow).addVertex(spb).addVertex(novosibirsk);
  cityGraph.addEdge(moscow, spb, 650.0);
  cityGraph.addEdge(moscow, novosibirsk, 2800.0);

  auto matrixGraph =
      GraphConverter<City, double>::toDirectedAdjacencyMatrix(cityGraph, 5);
  EXPECT_EQ(matrixGraph.vertexCount(), 3);
  EXPECT_EQ(matrixGraph.edgeCount(), 2);
  EXPECT_TRUE(matrixGraph.hasEdge(moscow, spb));
  EXPECT_DOUBLE_EQ(matrixGraph.getEdgeWeight(moscow, spb), 650.0);

  auto undirectedGraph =
      GraphConverter<City, double>::toUndirectedAdjacencyList(matrixGraph);
  EXPECT_EQ(undirectedGraph.vertexCount(), 3);
  EXPECT_EQ(undirectedGraph.edgeCount(), 2);
}

TEST(GraphConverterTest, SequentialConversions) {
  DirectedEdgeList<int, int> original;
  for (int i = 1; i <= 10; i++) {
    original.addVertex(i);
  }

  for (int i = 1; i <= 10; i++) {
    for (int j = 1; j <= 10; j++) {
      if (i != j) {
        original.addEdge(i, j, i + j);
      }
    }
  }

  auto list = GraphConverter<int, int>::toDirectedAdjacencyList(original);
  auto matrix = GraphConverter<int, int>::toDirectedAdjacencyMatrix(list, 15);
  auto undirectedList =
      GraphConverter<int, int>::toUndirectedAdjacencyList(matrix);
  auto undirectedEdgeList =
      GraphConverter<int, int>::toUndirectedEdgeList(undirectedList);
  auto directedAgain =
      GraphConverter<int, int>::toDirectedEdgeList(undirectedEdgeList);

  EXPECT_EQ(directedAgain.vertexCount(), 10);

  EXPECT_TRUE(directedAgain.hasEdge(1, 2));
  EXPECT_TRUE(directedAgain.hasEdge(5, 10));
  EXPECT_EQ(directedAgain.getEdgeWeight(1, 2), 3);
  EXPECT_EQ(directedAgain.getEdgeWeight(5, 10), 15);
}

TEST(GraphConverterTest, LargeGridGraph) {
  UndirectedAdjacencyList<std::pair<int, int>, int> grid;

  for (int i = 0; i < 20; i++) {
    for (int j = 0; j < 20; j++) {
      grid.addVertex({i, j});
    }
  }

  for (int i = 0; i < 20; i++) {
    for (int j = 0; j < 20; j++) {
      if (i < 19) grid.addEdge({i, j}, {i + 1, j}, 1);
      if (j < 19) grid.addEdge({i, j}, {i, j + 1}, 1);
    }
  }

  auto matrix =
      GraphConverter<std::pair<int, int>, int>::toUndirectedAdjacencyMatrix(
          grid, 400);
  auto gridAgain =
      GraphConverter<std::pair<int, int>, int>::toUndirectedAdjacencyList(
          matrix);

  EXPECT_EQ(gridAgain.vertexCount(), 400);
  EXPECT_EQ(gridAgain.edgeCount(), 2 * 20 * 19);

  EXPECT_TRUE(gridAgain.hasEdge({0, 0}, {1, 0}));
  EXPECT_TRUE(gridAgain.hasEdge({0, 0}, {0, 1}));
  EXPECT_TRUE(gridAgain.hasEdge({19, 19}, {18, 19}));
  EXPECT_TRUE(gridAgain.hasEdge({19, 19}, {19, 18}));
}

TEST(GraphConverterTest, ParallelEdges) {
  DirectedAdjacencyList<std::string, int> multiGraph;
  multiGraph.addVertex("A").addVertex("B");

  multiGraph.addEdge("A", "B", 1);
  multiGraph.addEdge("A", "B", 2);

  auto edgeList =
      GraphConverter<std::string, int>::toDirectedEdgeList(multiGraph);

  EXPECT_EQ(edgeList.edgeCount(), 1);
  EXPECT_EQ(edgeList.getEdgeWeight("A", "B"), 2);
}

TEST(GraphConverterTest, EmptyGraph) {
  DirectedAdjacencyList<std::string, double> empty;

  auto converted =
      GraphConverter<std::string, double>::toUndirectedAdjacencyList(empty);

  EXPECT_EQ(converted.vertexCount(), 0);
  EXPECT_EQ(converted.edgeCount(), 0);
}

TEST(GraphConverterTest, IsolatedVertices) {
  DirectedEdgeList<std::string, int> graph;
  graph.addVertex("A").addVertex("B").addVertex("C");
  graph.addEdge("A", "B", 1);

  auto matrix =
      GraphConverter<std::string, int>::toDirectedAdjacencyMatrix(graph, 5);

  EXPECT_EQ(matrix.vertexCount(), 3);
  EXPECT_EQ(matrix.edgeCount(), 1);
  EXPECT_TRUE(matrix.hasVertex("C"));
  EXPECT_FALSE(matrix.hasEdge("B", "C"));
  EXPECT_FALSE(matrix.hasEdge("C", "A"));
}

TEST(GraphConverterTest, NegativeWeights) {
  DirectedAdjacencyList<int, int> graph;
  graph.addVertex(1).addVertex(2).addVertex(3);
  graph.addEdge(1, 2, -5);
  graph.addEdge(2, 3, -10);

  auto undirected =
      GraphConverter<int, int>::toUndirectedAdjacencyMatrix(graph, 5);

  EXPECT_EQ(undirected.vertexCount(), 3);
  EXPECT_EQ(undirected.edgeCount(), 2);
  EXPECT_EQ(undirected.getEdgeWeight(1, 2), -5);
  EXPECT_EQ(undirected.getEdgeWeight(2, 3), -10);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
