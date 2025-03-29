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

template <typename VertexType, typename WeightType>
std::string exportWithHighlightedEdges(
    const UndirectedAdjacencyList<VertexType, WeightType>& graph,
    const std::vector<std::pair<VertexType, VertexType>>& highlightedEdges,
    const std::string& color = "red") {
  std::stringstream ss;
  ss << "graph G {\n";

  ss << "  // Graph settings\n";
  ss << "  graph [fontname=\"Arial\", rankdir=LR];\n";
  ss << "  node [fontname=\"Arial\", shape=circle, style=filled, "
        "fillcolor=lightblue];\n";
  ss << "  edge [fontname=\"Arial\"];\n\n";

  auto vertices = graph.getVertices();
  std::sort(vertices.begin(), vertices.end());

  ss << "  // Vertices\n";
  for (const auto& vertex : vertices) {
    ss << "  \"" << vertex << "\";\n";
  }
  ss << "\n";

  ss << "  // Edges\n";

  struct Edge {
    VertexType from;
    VertexType to;
    WeightType weight;
    bool highlighted;

    bool operator<(const Edge& other) const {
      if (from != other.from) return from < other.from;
      return to < other.to;
    }
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

      VertexType from = vertex;
      VertexType to = neighbor;

      if (from > to) {
        std::swap(from, to);
      }

      bool isHighlighted = false;
      for (const auto& highlightedEdge : highlightedEdges) {
        VertexType h_from = highlightedEdge.first;
        VertexType h_to = highlightedEdge.second;

        if (h_from > h_to) {
          std::swap(h_from, h_to);
        }

        if (from == h_from && to == h_to) {
          isHighlighted = true;
          break;
        }
      }

      edges.push_back(
          {from, to, graph.getEdgeWeight(vertex, neighbor), isHighlighted});
    }
  }

  std::sort(edges.begin(), edges.end());

  for (const auto& edge : edges) {
    ss << "  \"" << edge.from << "\" -- \"" << edge.to << "\" [label=\""
       << edge.weight << "\"";

    if (edge.highlighted) {
      ss << ", color=" << color << ", penwidth=3.0";
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
      .addVertex("Сочи");

  graph.addEdge("Москва", "Санкт-Петербург", 700);
  graph.addEdge("Москва", "Казань", 800);
  graph.addEdge("Москва", "Екатеринбург", 1800);
  graph.addEdge("Москва", "Сочи", 1600);
  graph.addEdge("Санкт-Петербург", "Казань", 1500);
  graph.addEdge("Казань", "Екатеринбург", 1000);
  graph.addEdge("Екатеринбург", "Новосибирск", 1500);
  graph.addEdge("Новосибирск", "Владивосток", 5800);
  graph.addEdge("Владивосток", "Сочи", 9000);
  graph.addEdge("Сочи", "Санкт-Петербург", 2300);
  graph.addEdge("Новосибирск", "Сочи", 4000);
  graph.addEdge("Екатеринбург", "Сочи", 2500);

  std::cout << "Визуализация графа дорожной сети России и построение MST..."
            << "\n";

  std::string outputDir = "../mst_visualization";
  std::system(("mkdir -p " + outputDir).c_str());

  std::string originalDotFile = outputDir + "/road_network.dot";
  std::string originalImageFile = outputDir + "/road_network.png";

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

  auto mstEdges = kruskal.getMSTEdges();
  int totalWeight = kruskal.getTotalWeight();

  std::cout << "\nМинимальное остовное дерево построено!" << "\n";
  std::cout << "Общая протяженность сети: " << totalWeight << " км"
            << "\n";
  std::cout << "Рёбра, входящие в MST:" << "\n";

  std::vector<std::pair<std::string, std::string>> highlightedEdges;

  for (const auto& edge : mstEdges) {
    std::cout << "  " << edge.from << " -- " << edge.to << " (" << edge.weight
              << " км)" << "\n";
    highlightedEdges.push_back({edge.from, edge.to});
  }

  std::string mstDotFile = outputDir + "/road_network_mst.dot";
  std::string mstImageFile = outputDir + "/road_network_mst.png";

  std::string mstDot =
      exportWithHighlightedEdges(graph, highlightedEdges, "red");
  GraphExporter<UndirectedAdjacencyList<std::string, int>>::saveToDOTFile(
      mstDot, mstDotFile);

  if (GraphExporter<UndirectedAdjacencyList<std::string, int>>::
          renderDOTToImage(mstDotFile, mstImageFile)) {
    std::cout << "\nГраф с выделенным MST сохранен и открывается..."
              << "\n";
    openFileWithDefaultApp(mstImageFile);
  }

  std::cout << "\nПостроение MST с использованием алгоритма Прима..."
            << "\n";

  Prim<UndirectedAdjacencyList<std::string, int>> prim(graph);
  prim.compute("Москва");

  auto primMSTEdges = prim.getMSTEdges();
  int primTotalWeight = prim.getTotalWeight();

  std::cout << "Общая протяженность сети по алгоритму Прима: "
            << primTotalWeight << " км" << "\n";

  if (primTotalWeight == totalWeight) {
    std::cout << "Алгоритмы Крускала и Прима дали одинаковый суммарный вес MST!"
              << "\n";
  } else {
    std::cout << "Внимание: алгоритмы дали разные суммарные веса. Проверьте "
                 "граф на уникальность весов."
              << "\n";
  }

  std::string primDotFile = outputDir + "/road_network_prim.dot";
  std::string primImageFile = outputDir + "/road_network_prim.png";

  std::vector<std::pair<std::string, std::string>> primHighlightedEdges;
  for (const auto& edge : primMSTEdges) {
    primHighlightedEdges.push_back({edge.from, edge.to});
  }

  std::string primDot =
      exportWithHighlightedEdges(graph, primHighlightedEdges, "blue");
  GraphExporter<UndirectedAdjacencyList<std::string, int>>::saveToDOTFile(
      primDot, primDotFile);

  if (GraphExporter<UndirectedAdjacencyList<std::string, int>>::
          renderDOTToImage(primDotFile, primImageFile)) {
    std::cout << "Граф с MST алгоритма Прима сохранен и открывается..."
              << "\n";
    openFileWithDefaultApp(primImageFile);
  }

  return 0;
}
