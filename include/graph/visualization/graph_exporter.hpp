#pragma once

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace graph {
namespace visualization {

template <typename Graph>
class GraphExporter {
 public:
  using vertex_type = typename std::remove_reference_t<
      decltype(*std::declval<Graph>().getVertices().begin())>;
  using weight_type =
      std::remove_reference_t<decltype(std::declval<Graph>().getEdgeWeight(
          *std::declval<Graph>().getVertices().begin(),
          *std::declval<Graph>()
               .getNeighbors(*std::declval<Graph>().getVertices().begin())
               .begin()))>;

  static std::string exportToDOT(const Graph& graph) {
    std::stringstream ss;

    bool isDirected = false;

    if (isDirected) {
      ss << "digraph G {\n";
    } else {
      ss << "graph G {\n";
    }

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
      vertex_type from;
      vertex_type to;
      weight_type weight;

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
        if (!isDirected) {
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
        }

        vertex_type from = vertex;
        vertex_type to = neighbor;

        if (!isDirected && from > to) {
          std::swap(from, to);
        }

        edges.push_back({from, to, graph.getEdgeWeight(vertex, neighbor)});
      }
    }

    std::sort(edges.begin(), edges.end());

    for (const auto& edge : edges) {
      ss << "  \"" << edge.from << "\" ";
      if (isDirected) {
        ss << "-> ";
      } else {
        ss << "-- ";
      }
      ss << "\"" << edge.to << "\" [label=\"" << edge.weight << "\"];\n";
    }

    ss << "}\n";
    return ss.str();
  }

  static void saveToDOTFile(const std::string& dot,
                            const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
      file << dot;
      file.close();
    } else {
      throw std::runtime_error("Не удалось открыть файл для записи: " +
                               filename);
    }
  }

  static bool renderDOTToImage(const std::string& dotFile,
                               const std::string& outputFile,
                               const std::string& format = "png") {
    std::string command =
        "dot -T" + format + " \"" + dotFile + "\" -o \"" + outputFile + "\"";
    int result = std::system(command.c_str());
    return result == 0;
  }
};

}  // namespace visualization
}  // namespace graph
