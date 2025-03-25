#pragma once

#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace graph {
namespace algorithms {

template <typename Graph>
class Dijkstra {
 public:
  using vertex_type = std::remove_reference_t<
      decltype(*std::declval<Graph>().getVertices().begin())>;
  using weight_type =
      std::remove_reference_t<decltype(std::declval<Graph>().getEdgeWeight(
          *std::declval<Graph>().getVertices().begin(),
          *std::declval<Graph>()
               .getNeighbors(*std::declval<Graph>().getVertices().begin())
               .begin()))>;

  explicit Dijkstra(const Graph &graph) : graph_(graph) {}

  void compute(const vertex_type &source) {
    reset();

    using queue_element = std::pair<weight_type, vertex_type>;
    auto compare = [](const queue_element &a, const queue_element &b) {
      return a.first > b.first;
    };
    std::priority_queue<queue_element, std::vector<queue_element>,
                        decltype(compare)>
        pq(compare);

    for (const auto &vertex : graph_.getVertices()) {
      distance_[vertex] = std::numeric_limits<weight_type>::max();
    }

    distance_[source] = 0;
    pq.push({0, source});

    while (!pq.empty()) {
      auto [dist, u] = pq.top();
      pq.pop();

      if (dist > distance_[u]) continue;

      for (const auto &v : graph_.getNeighbors(u)) {
        weight_type weight = graph_.getEdgeWeight(u, v);

        if (weight < 0) {
          throw std::runtime_error(
              "Dijkstra's algorithm doesn't support negative weights");
        }

        if (distance_[u] + weight < distance_[v]) {
          distance_[v] = distance_[u] + weight;
          parent_[v] = u;
          pq.push({distance_[v], v});
        }
      }
    }
  }

  std::vector<vertex_type> retrievePath(const vertex_type &target) const {
    if (distance_.find(target) == distance_.end() ||
        distance_.at(target) == std::numeric_limits<weight_type>::max()) {
      throw std::runtime_error("Target vertex is not reachable");
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
    auto it = distance_.find(vertex);
    if (it == distance_.end()) {
      throw std::runtime_error("Vertex not found");
    }
    if (it->second == std::numeric_limits<weight_type>::max()) {
      throw std::runtime_error("Vertex not reachable");
    }
    return it->second;
  }

  const std::unordered_map<vertex_type, weight_type> &getAllDistances() const {
    return distance_;
  }

  const std::unordered_map<vertex_type, vertex_type> &getParents() const {
    return parent_;
  }

 private:
  const Graph &graph_;
  std::unordered_map<vertex_type, weight_type> distance_;
  std::unordered_map<vertex_type, vertex_type> parent_;

  void reset() {
    distance_.clear();
    parent_.clear();
  }
};

}  // namespace algorithms
}  // namespace graph
