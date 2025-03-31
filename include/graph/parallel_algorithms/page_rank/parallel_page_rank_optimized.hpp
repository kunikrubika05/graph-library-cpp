#pragma once

#include <atomic>
#include <cmath>
#include <functional>
#include <mutex>
#include <numeric>
#include <thread>
#include <unordered_map>
#include <vector>

namespace graph {
namespace parallel_algorithms {

template <typename Graph>
class ParallelPageRankOptimized {
 public:
  ParallelPageRankOptimized(
      const Graph& graph,
      size_t num_threads = std::thread::hardware_concurrency())
      : graph_(graph), num_threads_(num_threads) {
    precomputeIncomingEdges();
  }

  std::vector<double> compute(double damping_factor = 0.85,
                              double tolerance = 1e-6,
                              int max_iterations = 100) {
    const size_t vertex_count = graph_.getVertexCount();

    std::vector<double> current_ranks(vertex_count, 1.0 / vertex_count);
    std::vector<double> next_ranks(vertex_count, 0.0);

    std::vector<std::thread> threads;
    const size_t block_size = vertex_count / num_threads_;

    double diff = tolerance + 1.0;
    int iterations = 0;

    while (diff > tolerance && iterations < max_iterations) {
      std::fill(next_ranks.begin(), next_ranks.end(), 0.0);

      threads.clear();
      for (size_t t = 0; t < num_threads_; ++t) {
        size_t start_idx = t * block_size;
        size_t end_idx =
            (t == num_threads_ - 1) ? vertex_count : (t + 1) * block_size;

        threads.emplace_back(&ParallelPageRankOptimized::computeIteration, this,
                             std::ref(current_ranks), std::ref(next_ranks),
                             damping_factor, start_idx, end_idx);
      }

      for (auto& thread : threads) {
        thread.join();
      }

      diff = computeDifference(current_ranks, next_ranks);
      std::swap(current_ranks, next_ranks);
      ++iterations;
    }

    normalizeRanks(current_ranks);
    return current_ranks;
  }

 private:
  const Graph& graph_;
  size_t num_threads_;
  std::vector<std::vector<size_t>> incoming_edges_;
  std::vector<size_t> out_degree_;

  void precomputeIncomingEdges() {
    const size_t vertex_count = graph_.getVertexCount();
    incoming_edges_.resize(vertex_count);
    out_degree_.resize(vertex_count);

    for (size_t i = 0; i < vertex_count; ++i) {
      auto neighbors = graph_.getNeighbors(i);
      out_degree_[i] = neighbors.size();

      for (auto neighbor : neighbors) {
        incoming_edges_[neighbor].push_back(i);
      }
    }
  }

  void computeIteration(std::vector<double>& current_ranks,
                        std::vector<double>& next_ranks, double damping_factor,
                        size_t start_idx, size_t end_idx) {
    const size_t vertex_count = graph_.getVertexCount();
    const double base_rank = (1.0 - damping_factor) / vertex_count;

    for (size_t i = start_idx; i < end_idx; ++i) {
      double sum = 0.0;

      for (auto j : incoming_edges_[i]) {
        if (out_degree_[j] > 0) {
          sum += current_ranks[j] / out_degree_[j];
        }
      }

      next_ranks[i] = base_rank + damping_factor * sum;
    }
  }

  double computeDifference(const std::vector<double>& prev_ranks,
                           const std::vector<double>& curr_ranks) {
    double diff = 0.0;
    for (size_t i = 0; i < prev_ranks.size(); ++i) {
      diff += std::abs(prev_ranks[i] - curr_ranks[i]);
    }
    return diff;
  }

  void normalizeRanks(std::vector<double>& ranks) {
    double sum = std::accumulate(ranks.begin(), ranks.end(), 0.0);
    if (sum > 0) {
      for (auto& rank : ranks) {
        rank /= sum;
      }
    }
  }
};

}  // namespace parallel_algorithms
}  // namespace graph
