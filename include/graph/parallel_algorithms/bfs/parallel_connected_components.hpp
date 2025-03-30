#pragma once

#include <atomic>
#include <vector>

#include "graph/parallel_algorithms/bfs/parallel_breadth_first_search.hpp"

namespace graph {
namespace parallel_algorithms {

template <typename Graph>
class ParallelConnectedComponents {
 public:
  using vertex_type = std::remove_reference_t<
      decltype(*std::declval<Graph>().getVertices().begin())>;
  using index_type = typename Graph::index_type;

  explicit ParallelConnectedComponents(
      const Graph& graph,
      size_t num_threads = std::thread::hardware_concurrency())
      : graph_(graph),
        num_threads_(num_threads),
        vertex_count_(graph.getVertexCount()) {
    visited_ = std::make_unique<std::atomic<bool>[]>(vertex_count_);
    for (size_t i = 0; i < vertex_count_; ++i) {
      visited_[i] = false;
    }
  }

  std::vector<std::vector<vertex_type>> computeComponents() {
    for (size_t i = 0; i < vertex_count_; ++i) {
      visited_[i] = false;
    }

    std::vector<std::vector<vertex_type>> components;

    for (size_t i = 0; i < vertex_count_; ++i) {
      if (!visited_[i]) {
        std::vector<vertex_type> component = findComponentStartingFrom(i);
        components.push_back(component);
      }
    }

    return components;
  }

  std::vector<vertex_type> findComponentStartingFrom(index_type start_idx) {
    ParallelBreadthFirstSearch<Graph> pbfs(graph_, num_threads_);
    pbfs.traverse(graph_.getVertexByIndex(start_idx));

    auto bfs_visited = pbfs.getVisited();
    std::vector<vertex_type> component;

    for (size_t i = 0; i < vertex_count_; ++i) {
      if (bfs_visited[i]) {
        visited_[i] = true;
        component.push_back(graph_.getVertexByIndex(i));
      }
    }

    return component;
  }

 private:
  const Graph& graph_;
  size_t num_threads_;
  size_t vertex_count_;
  std::unique_ptr<std::atomic<bool>[]> visited_;
};

}  // namespace parallel_algorithms
}  // namespace graph
