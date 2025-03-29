#include <iostream>
#include <string>
#include <filesystem>
#include <cstdlib>

#include "graph/structure/undirected_adjacency_list.hpp"
#include "graph/visualization/graph_exporter.hpp"

using namespace graph;
using namespace graph::visualization;
namespace fs = std::filesystem;

bool openFileWithDefaultApp(const std::string& filePath) {
#if defined(_WIN32) || defined(_WIN64)
  std::string command = "start \"\" \"" + filePath + "\"";
#elif defined(__APPLE__) || defined(__MACH__)
  std::string command = "open \"" + filePath + "\"";
#else
  std::string command = "xdg-open \"" + filePath + "\"";
#endif

  return std::system(command.c_str()) == 0;
}

bool createDirectoryIfNotExists(const std::string& dirPath) {
  try {
    if (!fs::exists(dirPath)) {
      return fs::create_directories(dirPath);
    }
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Ошибка при создании директории: " << e.what() << "\n";
    return false;
  }
}

int main() {
  std::string projectDir = fs::absolute("../..").string();
  std::string outputDir = "pictures";

  if (!createDirectoryIfNotExists(outputDir)) {
    std::cerr << "Не удалось создать директорию. Сохраняем в текущую директорию." << "\n";
    outputDir = ".";
  }


  UndirectedAdjacencyList<int, int> graph;

  graph.addVertex(1).addVertex(2).addVertex(3).addVertex(4).addVertex(5);

  graph.addEdge(1, 2, 7);
  graph.addEdge(1, 3, 9);
  graph.addEdge(1, 4, 14);
  graph.addEdge(2, 3, 10);
  graph.addEdge(2, 5, 69);
  graph.addEdge(4, 5, 69);
  graph.addEdge(3, 4, 69);

  graph.addEdge(3, 4, 69);
  graph.addEdge(3, 4, 69);
  graph.addEdge(3, 4, 69);

//  std::string dot =
//      GraphExporter<UndirectedAdjacencyList<int, int>>::exportToDOT(graph);
//  std::string dotFile = "graph_69.dot";
//  GraphExporter<UndirectedAdjacencyList<int, int>>::saveToDOTFile(dot, dotFile);

  std::string dotFile = outputDir + "/graph_69.dot";
  std::string imageFile = outputDir + "/graph_69.png";

  std::string dot = GraphExporter<UndirectedAdjacencyList<int, int>>::exportToDOT(graph);
  GraphExporter<UndirectedAdjacencyList<int, int>>::saveToDOTFile(dot, dotFile);

  // std::string imageFile = "graph_69.png";

  bool success =
      GraphExporter<UndirectedAdjacencyList<int, int>>::renderDOTToImage(
          dotFile, imageFile);

  if (success) {
    std::cout << "Картинку сохранили в " << imageFile << "\n";

    if (openFileWithDefaultApp(imageFile)) {
      std::cout << "Изображение открыто в приложении по умолчанию" << "\n";
    } else {
      std::cout << "Не удалось открыть изображение автоматически" << "\n";
    }

  } else {
    std::cout << "Не фартануло..." << "\n";
  }

  return 0;
}
