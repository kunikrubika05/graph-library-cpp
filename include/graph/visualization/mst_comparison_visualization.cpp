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
    const UndirectedAdjacencyList<std::string, int>& graph,
    const std::vector<std::pair<std::string, std::string>>& kruskalEdges,
    const std::vector<std::pair<std::string, std::string>>& primEdges) {
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
    ss << "  \"" << vertex << "\";\n";
  }
  ss << "\n";

  ss << "  // Edges\n";

  struct Edge {
    std::string from;
    std::string to;
    int weight;
    bool inKruskal;
    bool inPrim;

    bool operator<(const Edge& other) const {
      if (from != other.from) return from < other.from;
      return to < other.to;
    }
  };

  auto isEdgeInMST =
      [](const std::string& from, const std::string& to,
         const std::vector<std::pair<std::string, std::string>>& mstEdges) {
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

      std::string from = vertex;
      std::string to = neighbor;

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
    ss << "  \"" << edge.from << "\" -- \"" << edge.to << "\" [label=\""
       << edge.weight << "\"";

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
  UndirectedAdjacencyList<std::string, int> graph;

  graph.addVertex("Москва")
      .addVertex("Санкт-Петербург")
      .addVertex("Казань")
      .addVertex("Новосибирск")
      .addVertex("Екатеринбург")
      .addVertex("Владивосток")
      .addVertex("Сочи")
      .addVertex("Красноярск")
      .addVertex("Калининград");

  graph.addEdge("Москва", "Санкт-Петербург", 700);
  graph.addEdge("Москва", "Казань", 800);
  graph.addEdge("Москва", "Екатеринбург", 1800);
  graph.addEdge("Москва", "Сочи", 1600);
  graph.addEdge("Москва", "Калининград", 1200);
  graph.addEdge("Санкт-Петербург", "Казань", 1500);
  graph.addEdge("Санкт-Петербург", "Калининград", 800);
  graph.addEdge("Казань", "Екатеринбург", 1000);
  graph.addEdge("Екатеринбург", "Новосибирск", 1500);
  graph.addEdge("Екатеринбург", "Красноярск", 2000);
  graph.addEdge("Новосибирск", "Красноярск", 800);
  graph.addEdge("Новосибирск", "Владивосток", 5800);
  graph.addEdge("Красноярск", "Владивосток", 4500);
  graph.addEdge("Владивосток", "Сочи", 9000);
  graph.addEdge("Сочи", "Санкт-Петербург", 2300);
  graph.addEdge("Новосибирск", "Сочи", 4000);
  graph.addEdge("Екатеринбург", "Сочи", 2500);
  graph.addEdge("Казань", "Сочи", 2100);
  graph.addEdge("Казань", "Калининград", 1600);

  std::cout << "Визуализация сравнения алгоритмов MST Крускала и Прима..."
            << "\n";

  std::string outputDir = "../mst_visualization";
  std::system(("mkdir -p " + outputDir).c_str());

  std::string originalDotFile = outputDir + "/complete_road_network.dot";
  std::string originalImageFile = outputDir + "/complete_road_network.png";

  std::string originalDot =
      GraphExporter<UndirectedAdjacencyList<std::string, int>>::exportToDOT(
          graph);
  GraphExporter<UndirectedAdjacencyList<std::string, int>>::saveToDOTFile(
      originalDot, originalDotFile);

  if (GraphExporter<UndirectedAdjacencyList<std::string, int>>::
          renderDOTToImage(originalDotFile, originalImageFile)) {
    std::cout << "Исходный граф дорожной сети сохранен и открывается..."
              << "\n";
    openFileWithDefaultApp(originalImageFile);
  }

  std::this_thread::sleep_for(std::chrono::seconds(2));

  Kruskal<UndirectedAdjacencyList<std::string, int>> kruskal(graph);
  kruskal.compute();

  if (!kruskal.isConnected()) {
    std::cerr << "Граф не связный!" << "\n";
    return 1;
  }

  auto kruskalMSTEdges = kruskal.getMSTEdges();
  int kruskalWeight = kruskal.getTotalWeight();

  std::vector<std::pair<std::string, std::string>> kruskalEdges;
  for (const auto& edge : kruskalMSTEdges) {
    kruskalEdges.push_back({edge.from, edge.to});
  }

  std::cout << "MST по алгоритму Крускала: " << kruskalWeight << " км"
            << std::endl;
  std::cout << "Рёбра, входящие в MST Крускала:" << "\n";
  for (const auto& edge : kruskalMSTEdges) {
    std::cout << "  " << edge.from << " -- " << edge.to << " (" << edge.weight
              << " км)" << "\n";
  }

  Prim<UndirectedAdjacencyList<std::string, int>> prim(graph);
  prim.compute("Москва");

  auto primMSTEdges = prim.getMSTEdges();
  int primWeight = prim.getTotalWeight();

  std::vector<std::pair<std::string, std::string>> primEdges;
  for (const auto& edge : primMSTEdges) {
    primEdges.push_back({edge.from, edge.to});
  }

  std::cout << "\nMST по алгоритму Прима: " << primWeight << " км" << "\n";
  std::cout << "Рёбра, входящие в MST Прима:" << "\n";
  for (const auto& edge : primMSTEdges) {
    std::cout << "  " << edge.from << " -- " << edge.to << " (" << edge.weight
              << " км)" << "\n";
  }

  int commonEdges = 0;
  for (const auto& ke : kruskalEdges) {
    for (const auto& pe : primEdges) {
      if ((ke.first == pe.first && ke.second == pe.second) ||
          (ke.first == pe.second && ke.second == pe.first)) {
        ++commonEdges;
        break;
      }
    }
  }

  std::cout << "\nАнализ результатов:" << "\n";
  std::cout << "Общих рёбер в обоих MST: " << commonEdges << " из "
            << kruskalEdges.size() << "\n";

  if (kruskalWeight == primWeight) {
    std::cout << "Оба алгоритма дали одинаковый вес MST: " << kruskalWeight
              << " км" << "\n";
  } else {
    std::cout << "Разница в весе MST: " << std::abs(kruskalWeight - primWeight)
              << " км" << "\n";
    if (kruskalWeight < primWeight) {
      std::cout << "Алгоритм Крускала дал лучшее решение" << "\n";
    } else {
      std::cout << "Алгоритм Прима дал лучшее решение" << "\n";
    }
  }

  std::string comparisonDotFile = outputDir + "/mst_comparison.dot";
  std::string comparisonImageFile = outputDir + "/mst_comparison.png";

  std::string comparisonDot = exportWithTwoMSTs(graph, kruskalEdges, primEdges);
  GraphExporter<UndirectedAdjacencyList<std::string, int>>::saveToDOTFile(
      comparisonDot, comparisonDotFile);

  if (GraphExporter<UndirectedAdjacencyList<std::string, int>>::
          renderDOTToImage(comparisonDotFile, comparisonImageFile)) {
    std::cout << "\nСравнение алгоритмов MST сохранено и открывается..."
              << "\n";
    openFileWithDefaultApp(comparisonImageFile);
  }

  return 0;
}
