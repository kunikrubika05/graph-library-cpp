#pragma once

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace graph {
namespace algorithms {

template <typename Graph>
class BellmanFord {
 public:
  using vertex_type = typename std::remove_reference_t<
      decltype(*std::declval<Graph>().getVertices().begin())>;
  using weight_type =
      std::remove_reference_t<decltype(std::declval<Graph>().getEdgeWeight(
          *std::declval<Graph>().getVertices().begin(),
          *std::declval<Graph>()
               .getNeighbors(*std::declval<Graph>().getVertices().begin())
               .begin()))>;

  explicit BellmanFord(const Graph &graph) : graph_(graph) {}

  void compute(const vertex_type &source) {
    reset();

    for (const auto &vertex : graph_.getVertices()) {
      distance_[vertex] = std::numeric_limits<weight_type>::max();
    }

    distance_[source] = 0;

    const auto vertices = graph_.getVertices();
    const size_t n = vertices.size();

    for (size_t i = 0; i < n - 1; i++) {
      bool relaxed = false;
      for (const auto &u : vertices) {
        if (distance_[u] == std::numeric_limits<weight_type>::max()) {
          continue;
        }

        for (const auto &v : graph_.getNeighbors(u)) {
          weight_type weight = graph_.getEdgeWeight(u, v);
          if (distance_[u] + weight < distance_[v]) {
            distance_[v] = distance_[u] + weight;
            parent_[v] = u;
            relaxed = true;
          }
        }
      }

      if (!relaxed) break;
    }

    for (const auto &u : vertices) {
      if (distance_[u] == std::numeric_limits<weight_type>::max()) {
        continue;
      }

      for (const auto &v : graph_.getNeighbors(u)) {
        weight_type weight = graph_.getEdgeWeight(u, v);
        if (distance_[u] + weight < distance_[v]) {
          negative_cycle_ = true;
          return;
        }
      }
    }
  }

  std::vector<vertex_type> retrievePath(const vertex_type &target) const {
    if (negative_cycle_) {
      throw std::runtime_error("График содержит отрицательный цикл");
    }

    if (distance_.find(target) == distance_.end() ||
        distance_.at(target) == std::numeric_limits<weight_type>::max()) {
      throw std::runtime_error("Целевая вершина недостижима");
    }

    std::vector<vertex_type> path;
    vertex_type current = target;

    while (parent_.find(current) != parent_.end()) {
      path.push_back(current);
      current = parent_.at(current);
    }
    path.push_back(current);

    std::reverse(path.begin(), path.end());
    return path;
  }

  weight_type getDistance(const vertex_type &vertex) const {
    if (negative_cycle_) {
      throw std::runtime_error("Граф содержит отрицательный цикл");
    }

    auto it = distance_.find(vertex);
    if (it == distance_.end()) {
      throw std::runtime_error("Вершина не найдена");
    }

    if (it->second == std::numeric_limits<weight_type>::max()) {
      throw std::runtime_error("Вершина недостижима");
    }

    return it->second;
  }

  const std::unordered_map<vertex_type, weight_type> &getAllDistances() const {
    return distance_;
  }

  const std::unordered_map<vertex_type, vertex_type> &getParents() const {
    return parent_;
  }

  bool hasNegativeCycle() const { return negative_cycle_; }

 private:
  const Graph &graph_;
  std::unordered_map<vertex_type, weight_type> distance_;
  std::unordered_map<vertex_type, vertex_type> parent_;
  bool negative_cycle_ = false;

  void reset() {
    distance_.clear();
    parent_.clear();
    negative_cycle_ = false;
  }
};

}  // namespace algorithms
}  // namespace graph
