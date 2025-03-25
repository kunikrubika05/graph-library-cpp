#pragma once

#include <unordered_map>

#include "floyd.hpp"

namespace graph {
namespace algorithms {

template <typename Graph>
class GraphMetrics {
 public:
  using vertex_type = typename std::remove_reference_t<
      decltype(*std::declval<Graph>().getVertices().begin())>;
  using weight_type = decltype(std::declval<Graph>().getEdgeWeight(
      vertex_type(), vertex_type()));

  explicit GraphMetrics(const Graph &graph) : graph_(graph) {
    floyd_ = std::make_unique<FloydWarshall<Graph>>(graph);
    floyd_->compute();
  }

  weight_type eccentricity(const vertex_type &vertex) const {
    const auto &distances = floyd_->getDistances();
    auto it = distances.find(vertex);
    if (it == distances.end()) {
      throw std::runtime_error("Вершина не найдена");
    }

    weight_type max_dist = 0;
    for (const auto &pair : it->second) {
      if (pair.second != std::numeric_limits<weight_type>::max() &&
          pair.second > max_dist) {
        max_dist = pair.second;
      }
    }
    return max_dist;
  }

  weight_type diameter() const {
    const auto &distances = floyd_->getDistances();
    weight_type max_dist = 0;

    for (const auto &from_pair : distances) {
      for (const auto &to_pair : from_pair.second) {
        if (to_pair.second != std::numeric_limits<weight_type>::max() &&
            to_pair.second > max_dist) {
          max_dist = to_pair.second;
        }
      }
    }
    return max_dist;
  }

  weight_type radius() const {
    const auto vertices = graph_.getVertices();
    if (vertices.empty()) {
      throw std::runtime_error("Граф пуст");
    }

    weight_type min_eccentricity = std::numeric_limits<weight_type>::max();
    for (const auto &vertex : vertices) {
      try {
        weight_type ecc = eccentricity(vertex);
        if (ecc < min_eccentricity) {
          min_eccentricity = ecc;
        }
      } catch (const std::exception &) {
      }
    }
    return min_eccentricity;
  }

  std::vector<vertex_type> center() const {
    const auto vertices = graph_.getVertices();
    std::vector<vertex_type> center_vertices;

    weight_type min_eccentricity = radius();
    for (const auto &vertex : vertices) {
      try {
        if (eccentricity(vertex) == min_eccentricity) {
          center_vertices.push_back(vertex);
        }
      } catch (const std::exception &) {
      }
    }
    return center_vertices;
  }

 private:
  const Graph &graph_;
  std::unique_ptr<FloydWarshall<Graph>> floyd_;
};

}  // namespace algorithms
}  // namespace graph
