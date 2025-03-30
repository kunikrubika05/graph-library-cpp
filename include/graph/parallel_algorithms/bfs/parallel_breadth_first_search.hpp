#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <queue>
#include <thread>
#include <unordered_map>
#include <vector>

namespace graph {
namespace parallel_algorithms {

template <typename Graph>
class ParallelBreadthFirstSearch {
 public:
  using vertex_type = std::remove_reference_t<
      decltype(*std::declval<Graph>().getVertices().begin())>;
  using index_type = typename Graph::index_type;
  using on_vertex_callback = std::function<void(const vertex_type&)>;
  using on_edge_callback =
      std::function<void(const vertex_type&, const vertex_type&)>;
  using on_start_callback = std::function<void(const vertex_type&)>;
  using on_finish_callback = std::function<void()>;

  ParallelBreadthFirstSearch(
      const Graph& graph,
      size_t num_threads = std::thread::hardware_concurrency())
      : graph_(graph),
        num_threads_(num_threads),
        vertex_count_(graph.getVertexCount()) {
    visited_ = std::make_unique<std::atomic<int>[]>(vertex_count_);
    for (size_t i = 0; i < graph.getVertexCount(); ++i) {
      visited_[i] = 0;
    }

    distance_.resize(graph.getVertexCount(), -1);
    parent_.resize(graph.getVertexCount(), -1);
  }

  void traverse(
      const vertex_type& start_vertex,
      on_start_callback on_start = [](const vertex_type&) {},
      on_vertex_callback on_discover = [](const vertex_type&) {},
      on_edge_callback on_edge = [](const vertex_type&, const vertex_type&) {},
      on_finish_callback on_finish = []() {}) {
    reset();
    on_start(start_vertex);

    index_type start_idx = graph_.getVertexIndex(start_vertex);
    parallel_bfs(start_idx, on_discover, on_edge);

    on_finish();
  }

  std::vector<bool> getVisited() const {
    std::vector<bool> result(vertex_count_);

    for (size_t i = 0; i < vertex_count_; ++i) {
      result[i] = visited_[i] != 0;
    }

    return result;
  }

  std::vector<int> getDistance() const { return distance_; }

  std::vector<index_type> getParent() const { return parent_; }

 private:
  const Graph& graph_;
  size_t num_threads_;

  std::unique_ptr<std::atomic<int>[]> visited_;
  size_t vertex_count_;

  std::vector<int> distance_;
  std::vector<index_type> parent_;

  std::vector<index_type> current_frontier_;
  std::vector<index_type> next_frontier_;
  std::atomic<size_t> next_frontier_size_{0};

  void reset() {
    for (size_t i = 0; i < vertex_count_; ++i) {
      visited_[i] = 0;
    }

    std::fill(distance_.begin(), distance_.end(), -1);
    std::fill(parent_.begin(), parent_.end(), -1);
    current_frontier_.clear();
    next_frontier_.clear();
    next_frontier_.resize(graph_.getVertexCount());
    next_frontier_size_ = 0;
  }

  void parallel_bfs(index_type start_idx, on_vertex_callback on_discover,
                    on_edge_callback on_edge) {
    visited_[start_idx] = 1;
    distance_[start_idx] = 0;
    current_frontier_.push_back(start_idx);
    on_discover(graph_.getVertexByIndex(start_idx));

    while (!current_frontier_.empty()) {
      std::vector<std::thread> threads;
      size_t chunk_size =
          (current_frontier_.size() + num_threads_ - 1) / num_threads_;

      for (size_t t = 0; t < num_threads_; ++t) {
        threads.emplace_back([&, t]() {
          size_t start = t * chunk_size;
          size_t end = std::min(start + chunk_size, current_frontier_.size());

          for (size_t i = start; i < end; ++i) {
            index_type u = current_frontier_[i];

            auto [neighbors_begin, neighbors_end] = graph_.getNeighborRange(u);

            for (auto neighbor_ptr = neighbors_begin;
                 neighbor_ptr != neighbors_end; ++neighbor_ptr) {
              index_type v = *neighbor_ptr;

              on_edge(graph_.getVertexByIndex(u), graph_.getVertexByIndex(v));

              int expected = 0;
              if (visited_[v].compare_exchange_strong(expected, 1)) {
                distance_[v] = distance_[u] + 1;
                parent_[v] = u;

                size_t idx = next_frontier_size_.fetch_add(1);
                next_frontier_[idx] = v;

                on_discover(graph_.getVertexByIndex(v));
              }
            }
          }
        });
      }

      for (auto& thread : threads) {
        thread.join();
      }

      current_frontier_.clear();
      size_t frontier_size = next_frontier_size_.load();
      current_frontier_.insert(current_frontier_.end(), next_frontier_.begin(),
                               next_frontier_.begin() + frontier_size);
      next_frontier_size_ = 0;
    }
  }
};

}  // namespace parallel_algorithms
}  // namespace graph
