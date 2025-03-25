#pragma once

#include <algorithm>
#include <stack>
#include <unordered_map>
#include <vector>

#include "depth_first_search.hpp"

namespace graph {
namespace algorithms {

template <typename Graph>
class GraphCondensation {
 public:
  using vertex_type = typename DepthFirstSearch<Graph>::vertex_type;
  using condensed_vertex = std::vector<vertex_type>;

  explicit GraphCondensation(const Graph &graph) : graph_(graph) {}

  void buildCondensation() {
    reset();

    for (const auto &vertex : graph_.getVertices()) {
      if (!visited_[vertex]) {
        firstDFS(vertex);
      }
    }

    buildTransposedGraph();

    visited_.clear();
    while (!finish_order_.empty()) {
      vertex_type v = finish_order_.top();
      finish_order_.pop();
      if (!visited_[v]) {
        condensed_vertex component;
        secondDFS(v, component);
        condensed_vertices_.push_back(component);

        int component_id = condensed_vertices_.size() - 1;
        for (const auto &vertex : component) {
          component_mapping_[vertex] = component_id;
        }
      }
    }

    buildCondensedEdges();
  }

  const std::vector<condensed_vertex> &getCondensedVertices() const {
    return condensed_vertices_;
  }

  const std::vector<std::pair<int, int>> &getCondensedEdges() const {
    return condensed_edges_;
  }

  int getComponentId(const vertex_type &vertex) const {
    auto it = component_mapping_.find(vertex);
    if (it != component_mapping_.end()) {
      return it->second;
    }
    return -1;
  }

 private:
  const Graph &graph_;
  std::unordered_map<vertex_type, bool> visited_;
  std::stack<vertex_type> finish_order_;

  std::unordered_map<vertex_type, std::vector<vertex_type>> transposed_graph_;

  std::vector<condensed_vertex> condensed_vertices_;
  std::unordered_map<vertex_type, int> component_mapping_;
  std::vector<std::pair<int, int>> condensed_edges_;

  void reset() {
    visited_.clear();
    while (!finish_order_.empty()) {
      finish_order_.pop();
    }
    transposed_graph_.clear();
    condensed_vertices_.clear();
    component_mapping_.clear();
    condensed_edges_.clear();
  }

  void firstDFS(const vertex_type &vertex) {
    visited_[vertex] = true;
    for (const auto &neighbor : graph_.getNeighbors(vertex)) {
      if (!visited_[neighbor]) {
        firstDFS(neighbor);
      }
    }
    finish_order_.push(vertex);
  }

  void buildTransposedGraph() {
    for (const auto &vertex : graph_.getVertices()) {
      for (const auto &neighbor : graph_.getNeighbors(vertex)) {
        transposed_graph_[neighbor].push_back(vertex);
      }
    }
  }

  void secondDFS(const vertex_type &vertex, condensed_vertex &component) {
    visited_[vertex] = true;
    component.push_back(vertex);

    for (const auto &neighbor : transposed_graph_[vertex]) {
      if (!visited_[neighbor]) {
        secondDFS(neighbor, component);
      }
    }
  }

  void buildCondensedEdges() {
    std::unordered_map<int, std::unordered_map<int, bool>> edge_exists;

    for (const auto &vertex : graph_.getVertices()) {
      int from_component = component_mapping_[vertex];

      for (const auto &neighbor : graph_.getNeighbors(vertex)) {
        int to_component = component_mapping_[neighbor];

        if (from_component != to_component &&
            !edge_exists[from_component][to_component]) {
          condensed_edges_.push_back({from_component, to_component});
          edge_exists[from_component][to_component] = true;
        }
      }
    }
  }
};

}  // namespace algorithms
}  // namespace graph
