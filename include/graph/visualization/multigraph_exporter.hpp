#pragma once

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <type_traits>

namespace graph {
namespace visualization {

template <typename V, typename W>
class MultiGraphExporter {
 public:
  static std::string exportToDOT(const MultiGraph<V, W>& graph) {
    std::stringstream ss;

    ss << "graph G {\n";

    ss << "  // Graph settings\n";
    ss << "  graph [fontname=\"Arial\", splines=true, overlap=false, "
          "nodesep=0.4];\n";
    ss << "  node [fontname=\"Arial\", shape=circle, style=filled, "
          "fillcolor=lightblue, "
       << "fontsize=10, width=0.6, height=0.6, fixedsize=true];\n";
    ss << "  edge [fontname=\"Arial\", fontsize=8, len=1.2];\n\n";

    auto vertices = graph.getVertices();
    std::sort(vertices.begin(), vertices.end());

    ss << "  // Vertices\n";
    for (const auto& vertex : vertices) {
      ss << "  \"" << vertex << "\";\n";
    }
    ss << "\n";

    ss << "  // Edges\n";

    auto allEdges = graph.getAllUniquedEdges();

    std::sort(allEdges.begin(), allEdges.end(),
              [](const auto& a, const auto& b) {
                if (std::get<0>(a) != std::get<0>(b))
                  return std::get<0>(a) < std::get<0>(b);
                if (std::get<1>(a) != std::get<1>(b))
                  return std::get<1>(a) < std::get<1>(b);
                return std::get<3>(a) < std::get<3>(b);
              });

    std::unordered_map<std::string, int> edgeCountMap;

    for (const auto& [from, to, weight, edgeId] : allEdges) {
      std::string edgeKey;
      if (from < to) {
        std::stringstream keyStream;
        keyStream << from << "_" << to;
        edgeKey = keyStream.str();
      } else {
        std::stringstream keyStream;
        keyStream << to << "_" << from;
        edgeKey = keyStream.str();
      }

      if (edgeCountMap.find(edgeKey) == edgeCountMap.end()) {
        edgeCountMap[edgeKey] = 0;
      }
      int currentEdgeIndex = edgeCountMap[edgeKey]++;
      int totalEdgesForPair = graph.getAllEdges(from, to).size();

      ss << "  \"" << from << "\" -- \"" << to << "\" [";

      if (totalEdgesForPair > 1) {
        double curvature = 0.2 + (currentEdgeIndex * 0.2);

        ss << "label=\"" << weight << "\", penwidth=1.2, ";

        ss << "lbldistance=2.0, ";

        ss << "weight=1.0, ";

        ss << "constraint=false, ";

        ss << "headclip=true, tailclip=true, ";

        if (currentEdgeIndex % 2 == 0) {
          ss << "dir=forward, pos=\"e," << curvature << "\", ";
        } else {
          ss << "dir=back, pos=\"e," << curvature << "\", ";
        }

        int hue = (currentEdgeIndex * 60) % 360;
        ss << "color=\"" << getColorForHue(hue) << "\"";
      } else {
        ss << "label=\"" << weight << "\"";
      }

      ss << "];\n";
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
    std::string command = "circo -T" + format + " -Gdpi=300 \"" + dotFile +
                          "\" -o \"" + outputFile + "\"";
    int result = std::system(command.c_str());

    if (result != 0) {
      command = "fdp -T" + format + " -Gdpi=300 \"" + dotFile + "\" -o \"" +
                outputFile + "\"";
      result = std::system(command.c_str());

      if (result != 0) {
        command = "neato -T" + format + " -Gdpi=300 \"" + dotFile + "\" -o \"" +
                  outputFile + "\"";
        result = std::system(command.c_str());

        if (result != 0) {
          command = "dot -T" + format + " -Gdpi=300 \"" + dotFile + "\" -o \"" +
                    outputFile + "\"";
          result = std::system(command.c_str());
        }
      }
    }

    return result == 0;
  }

  static bool openFileWithDefaultApp(const std::string& filePath) {
#if defined(_WIN32) || defined(_WIN64)
    std::string command = "start \"\" \"" + filePath + "\"";
#elif defined(__APPLE__) || defined(__MACH__)
    std::string command = "open \"" + filePath + "\"";
#else
    std::string command = "xdg-open \"" + filePath + "\"";
#endif

    return std::system(command.c_str()) == 0;
  }

 private:
  static std::string getColorForHue(int hue) {
    double r, g, b;
    double h = hue / 60.0;
    double x = 1.0 - std::abs(std::fmod(h, 2) - 1.0);

    if (h < 1) {
      r = 1.0;
      g = x;
      b = 0.0;
    } else if (h < 2) {
      r = x;
      g = 1.0;
      b = 0.0;
    } else if (h < 3) {
      r = 0.0;
      g = 1.0;
      b = x;
    } else if (h < 4) {
      r = 0.0;
      g = x;
      b = 1.0;
    } else if (h < 5) {
      r = x;
      g = 0.0;
      b = 1.0;
    } else {
      r = 1.0;
      g = 0.0;
      b = x;
    }

    std::stringstream hex;
    hex << "#";
    hex << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(r * 255);
    hex << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(g * 255);
    hex << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(b * 255);

    return hex.str();
  }
};

}  // namespace visualization
}  // namespace graph
