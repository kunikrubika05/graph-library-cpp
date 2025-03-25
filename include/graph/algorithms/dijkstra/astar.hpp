#pragma once

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace graph {
namespace algorithms {

template <typename Graph>
class AStar {
 public:
  using vertex_type = typename std::remove_reference_t<
      decltype(*std::declval<Graph>().getVertices().begin())>;
  using weight_type = typename std::remove_reference_t<
      decltype(*std::declval<Graph>().getVertices().begin())>;
  using HeuristicFunction =
      std::function<weight_type(const vertex_type &, const vertex_type &)>;

  AStar(const Graph &graph, HeuristicFunction heuristic)
      : graph_(graph), heuristic_(heuristic) {}

  void compute(const vertex_type &source, const vertex_type &target) {
    reset();

    using queue_element = std::pair<weight_type, vertex_type>;
    auto compare = [](const queue_element &a, const queue_element &b) {
      return a.first > b.first;
    };
    std::priority_queue<queue_element, std::vector<queue_element>,
                        decltype(compare)>
        openSet(compare);

    g_score_[source] = 0;
    f_score_[source] = heuristic_(source, target);
    openSet.push({f_score_[source], source});

    while (!openSet.empty()) {
      auto [current_f, current] = openSet.top();
      openSet.pop();

      if (current == target) return;

      for (const auto &neighbor : graph_.getNeighbors(current)) {
        weight_type tentative_g =
            g_score_[current] + graph_.getEdgeWeight(current, neighbor);
        if (!g_score_.count(neighbor) || tentative_g < g_score_[neighbor]) {
          g_score_[neighbor] = tentative_g;
          f_score_[neighbor] = tentative_g + heuristic_(neighbor, target);
          parent_[neighbor] = current;
          openSet.push({f_score_[neighbor], neighbor});
        }
      }
    }
  }

  std::vector<vertex_type> retrievePath(const vertex_type &target) const {
    if (!g_score_.count(target))
      throw std::runtime_error("Target vertex is not reachable");

    std::vector<vertex_type> path;
    vertex_type current = target;
    while (parent_.count(current)) {
      path.push_back(current);
      current = parent_.at(current);
    }
    path.push_back(current);
    std::reverse(path.begin(), path.end());
    return path;
  }

  weight_type getDistance(const vertex_type &vertex) const {
    if (!g_score_.count(vertex))
      throw std::runtime_error("Vertex is not reachable");
    return g_score_.at(vertex);
  }

  const std::unordered_map<vertex_type, weight_type> &getAllDistances() const {
    return g_score_;
  }
  const std::unordered_map<vertex_type, vertex_type> &getParents() const {
    return parent_;
  }

 private:
  const Graph &graph_;
  HeuristicFunction heuristic_;
  std::unordered_map<vertex_type, weight_type> g_score_;
  std::unordered_map<vertex_type, weight_type> f_score_;
  std::unordered_map<vertex_type, vertex_type> parent_;

  void reset() {
    g_score_.clear();
    f_score_.clear();
    parent_.clear();
  }
};

}  // namespace algorithms
}  // namespace graph
