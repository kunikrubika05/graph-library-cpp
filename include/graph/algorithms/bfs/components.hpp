#pragma once

#include <queue>
#include <unordered_map>
#include <vector>

namespace graph {
namespace algorithms {

template <typename Graph>
class ConnectedComponents {
 public:
  using vertex_type = std::remove_reference_t<
      decltype(*std::declval<Graph>().getVertices().begin())>;

  explicit ConnectedComponents(const Graph &graph) : graph_(graph) {}

  std::vector<std::vector<vertex_type>> computeComponents() {
    std::unordered_map<vertex_type, bool> visited;
    std::vector<std::vector<vertex_type>> components;

    for (const auto &vertex : graph_.getVertices()) {
      visited[vertex] = false;
    }

    for (const auto &vertex : graph_.getVertices()) {
      if (!visited[vertex]) {
        std::vector<vertex_type> component;
        bfs(vertex, visited, component);
        components.push_back(component);
      }
    }
    return components;
  }

 private:
  const Graph &graph_;

  void bfs(const vertex_type &start,
           std::unordered_map<vertex_type, bool> &visited,
           std::vector<vertex_type> &component) {
    std::queue<vertex_type> q;
    visited[start] = true;
    q.push(start);

    while (!q.empty()) {
      vertex_type current = q.front();
      q.pop();
      component.push_back(current);
      for (const auto &neighbor : graph_.getNeighbors(current)) {
        if (!visited[neighbor]) {
          visited[neighbor] = true;
          q.push(neighbor);
        }
      }
    }
  }
};

}  // namespace algorithms
}  // namespace graph
