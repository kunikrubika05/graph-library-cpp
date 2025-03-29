#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "graph/structure/undirected_adjacency_list.hpp"
#include "graph/visualization/graph_exporter.hpp"

using namespace graph;
using namespace graph::visualization;

TEST(GraphExportTest, ExportToDOT) {
  UndirectedAdjacencyList<int, int> graph;

  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4);
  graph.addEdge(1, 2, 5);
  graph.addEdge(2, 3, 3);
  graph.addEdge(3, 4, 2);
  graph.addEdge(4, 1, 4);

  std::string dot =
      GraphExporter<UndirectedAdjacencyList<int, int>>::exportToDOT(graph);

  std::cout << "DOT output:\n" << dot << "\n";

  EXPECT_NE(dot.find("\"1\""), std::string::npos);
  EXPECT_NE(dot.find("\"2\""), std::string::npos);
  EXPECT_NE(dot.find("\"3\""), std::string::npos);
  EXPECT_NE(dot.find("\"4\""), std::string::npos);

  EXPECT_NE(dot.find("\"1\" -- \"2\""), std::string::npos);
  EXPECT_NE(dot.find("\"2\" -- \"3\""), std::string::npos);
  EXPECT_NE(dot.find("\"3\" -- \"4\""), std::string::npos);
  EXPECT_NE(dot.find("\"1\" -- \"4\""), std::string::npos);

  EXPECT_NE(dot.find("label=\"5\""), std::string::npos);
  EXPECT_NE(dot.find("label=\"3\""), std::string::npos);
  EXPECT_NE(dot.find("label=\"2\""), std::string::npos);
  EXPECT_NE(dot.find("label=\"4\""), std::string::npos);
}

TEST(GraphExportTest, SaveToDOTFile) {
  UndirectedAdjacencyList<int, int> graph;

  graph.addVertex(1).addVertex(2).addVertex(3);
  graph.addEdge(1, 2, 5);
  graph.addEdge(2, 3, 3);
  graph.addEdge(3, 1, 4);

  std::string dot =
      GraphExporter<UndirectedAdjacencyList<int, int>>::exportToDOT(graph);

  std::string filename = "test_graph_export.dot";

  GraphExporter<UndirectedAdjacencyList<int, int>>::saveToDOTFile(dot,
                                                                  filename);

  std::ifstream file(filename);
  EXPECT_TRUE(file.is_open());

  std::string content((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
  file.close();

  EXPECT_EQ(content, dot);

  std::remove(filename.c_str());
}

TEST(GraphExportTest, RenderDOTToImage) {
  UndirectedAdjacencyList<int, int> graph;

  graph.addVertex(1).addVertex(2).addVertex(3);
  graph.addEdge(1, 2, 5);
  graph.addEdge(2, 3, 3);
  graph.addEdge(3, 1, 4);

  std::string dot =
      GraphExporter<UndirectedAdjacencyList<int, int>>::exportToDOT(graph);

  std::string dotFilename = "test_render.dot";
  std::string pngFilename = "test_render.png";

  GraphExporter<UndirectedAdjacencyList<int, int>>::saveToDOTFile(dot,
                                                                  dotFilename);

  bool success =
      GraphExporter<UndirectedAdjacencyList<int, int>>::renderDOTToImage(
          dotFilename, pngFilename);

  if (success) {
    std::ifstream file(pngFilename);
    EXPECT_TRUE(file.is_open());
    file.close();

    std::remove(pngFilename.c_str());
  } else {
    std::cout
        << "Тест рендеринга пропущен: Graphviz не найден или не установлен"
        << "\n";
  }

  std::remove(dotFilename.c_str());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
