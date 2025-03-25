#pragma once

#include <algorithm>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "depth_first_search.hpp"

namespace graph {
namespace algorithms {

template <typename Graph>
class TopologicalSort {
 public:
  using vertex_type = typename DepthFirstSearch<Graph>::vertex_type;

  explicit TopologicalSort(const Graph &graph) : graph_(graph), dfs_(graph) {}

  bool sort() {
    reset();

    if (hasCycle()) {
      return false;
    }

    std::unordered_map<vertex_type, bool> visited;
    sorted_vertices_.clear();

    for (const auto &vertex : graph_.getVertices()) {
      if (!visited[vertex]) {
        visitVertex(vertex, visited);
      }
    }

    std::reverse(sorted_vertices_.begin(), sorted_vertices_.end());

    return true;
  }

  const std::vector<vertex_type> &getSortedVertices() const {
    return sorted_vertices_;
  }

 private:
  const Graph &graph_;
  DepthFirstSearch<Graph> dfs_;
  std::vector<vertex_type> sorted_vertices_;
  std::unordered_map<vertex_type, int> vertex_states_;

  void reset() {
    sorted_vertices_.clear();
    vertex_states_.clear();
  }

  bool hasCycle() {
    for (const auto &vertex : graph_.getVertices()) {
      vertex_states_[vertex] = 0;
    }

    for (const auto &vertex : graph_.getVertices()) {
      if (vertex_states_[vertex] == 0) {
        if (detectCycle(vertex)) {
          return true;
        }
      }
    }
    return false;
  }

  bool detectCycle(vertex_type current) {
    vertex_states_[current] = 1;

    for (const auto &neighbor : graph_.getNeighbors(current)) {
      if (vertex_states_[neighbor] == 1) {
        return true;
      }
      if (vertex_states_[neighbor] == 0 && detectCycle(neighbor)) {
        return true;
      }
    }

    vertex_states_[current] = 2;
    return false;
  }

  void visitVertex(vertex_type current,
                   std::unordered_map<vertex_type, bool> &visited) {
    visited[current] = true;

    for (const auto &neighbor : graph_.getNeighbors(current)) {
      if (!visited[neighbor]) {
        visitVertex(neighbor, visited);
      }
    }

    sorted_vertices_.push_back(current);
  }
};

}  // namespace algorithms
}  // namespace graph
