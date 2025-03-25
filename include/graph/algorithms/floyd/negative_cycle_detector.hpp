#pragma once

#include "floyd.hpp"

namespace graph {
namespace algorithms {

template <typename Graph>
class NegativeCycleDetector {
 public:
  using vertex_type = typename std::remove_reference_t<
      decltype(*std::declval<Graph>().getVertices().begin())>;
  using weight_type = decltype(std::declval<Graph>().getEdgeWeight(
      vertex_type(), vertex_type()));

  explicit NegativeCycleDetector(const Graph &graph) : graph_(graph) {
    floyd_ = std::make_unique<FloydWarshall<Graph>>(graph);
    floyd_->compute();
  }

  bool hasNegativeCycle() const { return floyd_->hasNegativeCycle(); }

  std::vector<vertex_type> findNegativeCycle() const {
    if (!hasNegativeCycle()) {
      return {};
    }

    const auto vertices = graph_.getVertices();
    const auto &distances = floyd_->getDistances();

    for (const auto &vertex : vertices) {
      if (distances.at(vertex).at(vertex) < 0) {
        return reconstructCycle(vertex);
      }
    }

    return {};
  }

 private:
  const Graph &graph_;
  std::unique_ptr<FloydWarshall<Graph>> floyd_;

  std::vector<vertex_type> reconstructCycle(const vertex_type &start) const {
    std::vector<vertex_type> cycle;
    std::unordered_map<vertex_type, bool> visited;

    vertex_type current = start;
    do {
      if (visited[current]) {
        while (cycle.front() != current) {
          cycle.erase(cycle.begin());
        }
        return cycle;
      }

      visited[current] = true;
      cycle.push_back(current);

      bool found = false;
      for (const auto &neighbor : graph_.getNeighbors(current)) {
        weight_type weight = graph_.getEdgeWeight(current, neighbor);
        if (floyd_->getDistances().at(neighbor).at(neighbor) < 0) {
          current = neighbor;
          found = true;
          break;
        }
      }

      if (!found) break;

    } while (true);

    return cycle;
  }
};

}  // namespace algorithms
}  // namespace graph
