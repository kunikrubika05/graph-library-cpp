#pragma once

#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>

#include "depth_first_search.hpp"

namespace graph {
namespace algorithms {

template <typename Graph>
class Bridges {
 public:
  using vertex_type = typename DepthFirstSearch<Graph>::vertex_type;
  using edge_type = std::pair<vertex_type, vertex_type>;

  explicit Bridges(const Graph &graph) : graph_(graph) {}

  void findBridges() {
    reset();

    for (const auto &vertex : graph_.getVertices()) {
      if (!visited_[vertex]) {
        dfs(vertex, vertex);
      }
    }
  }

  const std::vector<edge_type> &getBridges() const { return bridges_; }

 private:
  const Graph &graph_;
  std::unordered_map<vertex_type, bool> visited_;
  std::unordered_map<vertex_type, int> discovery_time_;
  std::unordered_map<vertex_type, int> low_link_;
  std::vector<edge_type> bridges_;
  int time_ = 0;

  void reset() {
    visited_.clear();
    discovery_time_.clear();
    low_link_.clear();
    bridges_.clear();
    time_ = 0;
  }

  void dfs(vertex_type current, vertex_type parent) {
    visited_[current] = true;

    discovery_time_[current] = low_link_[current] = ++time_;

    for (const auto &neighbor : graph_.getNeighbors(current)) {
      if (!visited_[neighbor]) {
        dfs(neighbor, current);

        low_link_[current] = std::min(low_link_[current], low_link_[neighbor]);

        if (low_link_[neighbor] > discovery_time_[current]) {
          bridges_.push_back({current, neighbor});
        }
      } else if (neighbor != parent) {
        low_link_[current] =
            std::min(low_link_[current], discovery_time_[neighbor]);
      }
    }
  }
};
}  // namespace algorithms
}  // namespace graph
