#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include "graph/algorithms/mst/kruskal.hpp"
#include "graph/algorithms/mst/prim.hpp"
#include "graph/structure/undirected_adjacency_list.hpp"
#include "graph/visualization/graph_exporter.hpp"

using namespace graph;
using namespace graph::visualization;
using namespace graph::algorithms;

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

std::string exportWithTwoMSTs(
    const UndirectedAdjacencyList<int, int>& graph,
    const std::vector<std::pair<int, int>>& kruskalEdges,
    const std::vector<std::pair<int, int>>& primEdges) {
  std::stringstream ss;
  ss << "graph G {\n";

  ss << "  // Graph settings\n";
  ss << "  graph [fontname=\"Arial\", rankdir=LR, splines=true];\n";
  ss << "  node [fontname=\"Arial\", shape=circle, style=filled, "
        "fillcolor=lightblue];\n";
  ss << "  edge [fontname=\"Arial\"];\n\n";

  ss << "  // Legend\n";
  ss << "  subgraph cluster_legend {\n";
  ss << "    label=\"Легенда\";\n";
  ss << "    style=filled;\n";
  ss << "    color=lightgrey;\n";
  ss << "    fontsize=14;\n";
  ss << "    \"Обычное ребро\" [shape=plaintext, label=\"Обычное ребро\", "
        "fillcolor=white];\n";
  ss << "    \"MST Крускала\" [shape=plaintext, label=\"MST Крускала\", "
        "fillcolor=white];\n";
  ss << "    \"MST Прима\" [shape=plaintext, label=\"MST Прима\", "
        "fillcolor=white];\n";
  ss << "    \"Общие рёбра\" [shape=plaintext, label=\"Общие рёбра MST\", "
        "fillcolor=white];\n";

  ss << "    \"Обычное ребро\" -- \"MST Крускала\" [style=dashed, "
        "color=grey];\n";
  ss << "    \"MST Крускала\" -- \"MST Прима\" [color=red, penwidth=2.0];\n";
  ss << "    \"MST Прима\" -- \"Общие рёбра\" [color=blue, penwidth=2.0];\n";
  ss << "    \"Общие рёбра\" -- \"Обычное ребро\" [color=purple, "
        "penwidth=3.0];\n";
  ss << "  }\n\n";

  auto vertices = graph.getVertices();
  std::sort(vertices.begin(), vertices.end());

  ss << "  // Vertices\n";
  for (const auto& vertex : vertices) {
    ss << "  " << vertex << ";\n";
  }
  ss << "\n";

  ss << "  // Edges\n";

  struct Edge {
    int from;
    int to;
    int weight;
    bool inKruskal;
    bool inPrim;

    bool operator<(const Edge& other) const {
      if (from != other.from) return from < other.from;
      return to < other.to;
    }
  };

  auto isEdgeInMST = [](const int& from, const int& to,
                        const std::vector<std::pair<int, int>>& mstEdges) {
    for (const auto& edge : mstEdges) {
      if ((edge.first == from && edge.second == to) ||
          (edge.first == to && edge.second == from)) {
        return true;
      }
    }
    return false;
  };

  std::vector<Edge> edges;
  std::unordered_set<std::string> processedEdges;

  for (const auto& vertex : vertices) {
    auto neighbors = graph.getNeighbors(vertex);
    std::sort(neighbors.begin(), neighbors.end());

    for (const auto& neighbor : neighbors) {
      std::stringstream edge_id_stream;
      if (vertex < neighbor) {
        edge_id_stream << vertex << "_" << neighbor;
      } else {
        edge_id_stream << neighbor << "_" << vertex;
      }
      std::string edgeId = edge_id_stream.str();

      if (processedEdges.count(edgeId) > 0) {
        continue;
      }

      processedEdges.insert(edgeId);

      int from = vertex;
      int to = neighbor;

      if (from > to) {
        std::swap(from, to);
      }

      bool inKruskal = isEdgeInMST(from, to, kruskalEdges);
      bool inPrim = isEdgeInMST(from, to, primEdges);

      edges.push_back(
          {from, to, graph.getEdgeWeight(vertex, neighbor), inKruskal, inPrim});
    }
  }

  std::sort(edges.begin(), edges.end());

  for (const auto& edge : edges) {
    ss << "  " << edge.from << " -- " << edge.to << " [label=\"" << edge.weight
       << "\"";

    if (edge.inKruskal && edge.inPrim) {
      ss << ", color=purple, penwidth=3.0";
    } else if (edge.inKruskal) {
      ss << ", color=red, penwidth=2.0";
    } else if (edge.inPrim) {
      ss << ", color=blue, penwidth=2.0";
    } else {
      ss << ", style=dashed, color=grey";
    }

    ss << "];\n";
  }

  ss << "}\n";
  return ss.str();
}

int main() {
  UndirectedAdjacencyList<int, int> graph;

  for (int i = 1; i <= 8; i++) {
    graph.addVertex(i);
  }

  graph.addEdge(1, 2, 1);
  graph.addEdge(2, 3, 1);
  graph.addEdge(3, 4, 1);
  graph.addEdge(4, 5, 1);
  graph.addEdge(5, 6, 1);
  graph.addEdge(6, 7, 1);
  graph.addEdge(7, 8, 1);
  graph.addEdge(8, 1, 1);

  graph.addEdge(1, 5, 2);
  graph.addEdge(2, 6, 2);
  graph.addEdge(3, 7, 2);
  graph.addEdge(4, 8, 2);

  graph.addEdge(1, 3, 5);
  graph.addEdge(2, 4, 5);
  graph.addEdge(5, 7, 5);
  graph.addEdge(6, 8, 5);

  std::cout << "Визуализация сравнения алгоритмов MST с разными результатами..."
            << "\n";

  std::string outputDir = "../mst_visualization";
  std::system(("mkdir -p " + outputDir).c_str());

  std::string originalDotFile = outputDir + "/different_mst_original.dot";
  std::string originalImageFile = outputDir + "/different_mst_original.png";

  std::string originalDot =
      GraphExporter<UndirectedAdjacencyList<int, int>>::exportToDOT(graph);
  GraphExporter<UndirectedAdjacencyList<int, int>>::saveToDOTFile(
      originalDot, originalDotFile);

  if (GraphExporter<UndirectedAdjacencyList<int, int>>::renderDOTToImage(
          originalDotFile, originalImageFile)) {
    std::cout << "Исходный граф сохранен и открывается..." << "\n";
    openFileWithDefaultApp(originalImageFile);
  }

  std::this_thread::sleep_for(std::chrono::seconds(2));

  Kruskal<UndirectedAdjacencyList<int, int>> kruskal(graph);
  kruskal.compute();

  if (!kruskal.isConnected()) {
    std::cerr << "Граф не связный!" << "\n";
    return 1;
  }

  auto kruskalMSTEdges = kruskal.getMSTEdges();
  int kruskalWeight = kruskal.getTotalWeight();

  std::vector<std::pair<int, int>> kruskalEdges;
  for (const auto& edge : kruskalMSTEdges) {
    kruskalEdges.push_back({edge.from, edge.to});
  }

  std::cout << "\nMST по алгоритму Крускала: " << kruskalWeight << " единиц"
            << "\n";
  std::cout << "Рёбра, входящие в MST Крускала:" << "\n";
  for (const auto& edge : kruskalMSTEdges) {
    std::cout << "  " << edge.from << " -- " << edge.to
              << " (вес: " << edge.weight << ")" << "\n";
  }

  Prim<UndirectedAdjacencyList<int, int>> prim(graph);
  prim.compute(1);

  auto primMSTEdges = prim.getMSTEdges();
  int primWeight = prim.getTotalWeight();

  std::vector<std::pair<int, int>> primEdges;
  for (const auto& edge : primMSTEdges) {
    primEdges.push_back({edge.from, edge.to});
  }

  std::cout << "\nMST по алгоритму Прима: " << primWeight << " единиц"
            << "\n";
  std::cout << "Рёбра, входящие в MST Прима:" << "\n";
  for (const auto& edge : primMSTEdges) {
    std::cout << "  " << edge.from << " -- " << edge.to
              << " (вес: " << edge.weight << ")" << "\n";
  }

  int commonEdges = 0;
  std::cout << "\nРёбра, которые выбрали оба алгоритма:" << "\n";
  for (const auto& ke : kruskalEdges) {
    for (const auto& pe : primEdges) {
      if ((ke.first == pe.first && ke.second == pe.second) ||
          (ke.first == pe.second && ke.second == pe.first)) {
        commonEdges++;
        std::cout << "  " << ke.first << " -- " << ke.second << "\n";
        break;
      }
    }
  }

  std::cout << "\nАнализ результатов:" << "\n";
  std::cout << "Общих рёбер в обоих MST: " << commonEdges << " из "
            << kruskalEdges.size() << "\n";

  std::cout << "Рёбра, выбранные только алгоритмом Крускала:" << "\n";
  bool foundDifferentKruskal = false;
  for (const auto& ke : kruskalEdges) {
    bool found = false;
    for (const auto& pe : primEdges) {
      if ((ke.first == pe.first && ke.second == pe.second) ||
          (ke.first == pe.second && ke.second == pe.first)) {
        found = true;
        break;
      }
    }
    if (!found) {
      foundDifferentKruskal = true;
      std::cout << "  " << ke.first << " -- " << ke.second << "\n";
    }
  }
  if (!foundDifferentKruskal) {
    std::cout << "  Нет таких рёбер" << "\n";
  }

  std::cout << "Рёбра, выбранные только алгоритмом Прима:" << "\n";
  bool foundDifferentPrim = false;
  for (const auto& pe : primEdges) {
    bool found = false;
    for (const auto& ke : kruskalEdges) {
      if ((pe.first == ke.first && pe.second == ke.second) ||
          (pe.first == ke.second && pe.second == ke.first)) {
        found = true;
        break;
      }
    }
    if (!found) {
      foundDifferentPrim = true;
      std::cout << "  " << pe.first << " -- " << pe.second << "\n";
    }
  }
  if (!foundDifferentPrim) {
    std::cout << "  Нет таких рёбер" << "\n";
  }

  if (kruskalWeight == primWeight) {
    std::cout << "\nОба алгоритма дали одинаковый вес MST: " << kruskalWeight
              << " единиц, но разные наборы рёбер!" << "\n";
    std::cout << "Это доказывает, что для графа может существовать несколько "
                 "различных MST, "
              << "если рёбра имеют одинаковые веса." << "\n";
  } else {
    std::cout << "\nРазница в весе MST: "
              << std::abs(kruskalWeight - primWeight) << " единиц" << "\n";
    if (kruskalWeight < primWeight) {
      std::cout << "Алгоритм Крускала дал лучшее решение, что странно, так как "
                   "оба алгоритма должны находить MST."
                << "\n";
      std::cout << "Возможно, есть ошибка в реализации или настройках."
                << "\n";
    } else {
      std::cout << "Алгоритм Прима дал лучшее решение, что странно, так как "
                   "оба алгоритма должны находить MST."
                << "\n";
      std::cout << "Возможно, есть ошибка в реализации или настройках."
                << "\n";
    }
  }

  std::string comparisonDotFile = outputDir + "/different_mst_comparison.dot";
  std::string comparisonImageFile = outputDir + "/different_mst_comparison.png";

  std::string comparisonDot = exportWithTwoMSTs(graph, kruskalEdges, primEdges);
  GraphExporter<UndirectedAdjacencyList<int, int>>::saveToDOTFile(
      comparisonDot, comparisonDotFile);

  if (GraphExporter<UndirectedAdjacencyList<int, int>>::renderDOTToImage(
          comparisonDotFile, comparisonImageFile)) {
    std::cout << "\nСравнение различных MST сохранено и открывается..."
              << "\n";
    openFileWithDefaultApp(comparisonImageFile);
  }

  return 0;
}
