#pragma once

#include <unordered_map>
#include <vector>

#include "depth_first_search.hpp"

namespace graph {
namespace algorithms {

template <typename Graph>
class BipartiteChecker {
 public:
  using vertex_type = typename DepthFirstSearch<Graph>::vertex_type;

  explicit BipartiteChecker(const Graph &graph) : graph_(graph) {}

  bool isBipartite() {
    reset();

    for (auto vertex : graph_.getVertices()) {
      if (colors_.find(vertex) == colors_.end()) {
        colors_[vertex] = 0;
        if (!checkComponentDFS(vertex)) {
          is_bipartite_ = false;
          return false;
        }
      }
    }

    is_bipartite_ = true;
    return true;
  }

  std::pair<std::vector<vertex_type>, std::vector<vertex_type>> getPartition() {
    if (!is_checked_) {
      is_bipartite_ = isBipartite();
    }

    std::pair<std::vector<vertex_type>, std::vector<vertex_type>> result;

    if (!is_bipartite_) {
      return result;
    }

    for (const auto &pair : colors_) {
      if (pair.second == 0) {
        result.first.push_back(pair.first);
      } else {
        result.second.push_back(pair.first);
      }
    }

    return result;
  }

 private:
  const Graph &graph_;
  std::unordered_map<vertex_type, int> colors_;
  bool is_checked_ = false;
  bool is_bipartite_ = false;

  void reset() {
    colors_.clear();
    is_checked_ = true;
  }

  bool checkComponentDFS(vertex_type current) {
    for (auto neighbor : graph_.getNeighbors(current)) {
      if (colors_.find(neighbor) == colors_.end()) {
        colors_[neighbor] = 1 - colors_[current];

        if (!checkComponentDFS(neighbor)) {
          return false;
        }
      }

      else if (colors_[neighbor] == colors_[current]) {
        return false;
      }
    }

    return true;
  }
};

}  // namespace algorithms
}  // namespace graph
