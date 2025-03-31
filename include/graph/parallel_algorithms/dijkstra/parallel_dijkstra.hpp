#pragma once

#include <atomic>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace graph {
namespace parallel_algorithms {

template <typename Graph, typename WeightType = double>
class ParallelDijkstra {
 public:
  using vertex_type = std::remove_reference_t<
      decltype(*std::declval<Graph>().getVertices().begin())>;
  using index_type = typename Graph::index_type;

  ParallelDijkstra(const Graph& graph,
                   size_t num_threads = std::thread::hardware_concurrency())
      : graph_(graph),
        num_threads_(num_threads),
        vertex_count_(graph.getVertexCount()),
        vertex_mutexes_(vertex_count_) {
    delta_ = 1.0;
  }

  void findShortestPaths(const vertex_type& source_vertex) {
    distance_.assign(vertex_count_, std::numeric_limits<WeightType>::max());
    predecessor_.assign(vertex_count_, static_cast<index_type>(-1));
    processed_.assign(vertex_count_, false);

    index_type source_idx = graph_.getVertexIndex(source_vertex);
    distance_[source_idx] = 0;

    std::vector<std::vector<index_type>> buckets(vertex_count_);
    buckets[0].push_back(source_idx);

    for (size_t current_bucket = 0; current_bucket < buckets.size();
         ++current_bucket) {
      if (buckets[current_bucket].empty()) continue;

      processVerticesInBucket(buckets[current_bucket], buckets);
    }
  }

  std::vector<WeightType> getDistances() const { return distance_; }
  std::vector<index_type> getPredecessors() const { return predecessor_; }

 private:
  const Graph& graph_;
  size_t num_threads_;
  size_t vertex_count_;
  WeightType delta_;

  std::vector<WeightType> distance_;
  std::vector<index_type> predecessor_;
  std::vector<bool> processed_;
  std::vector<std::mutex> vertex_mutexes_;

  void processVerticesInBucket(const std::vector<index_type>& bucket,
                               std::vector<std::vector<index_type>>& buckets) {
    std::vector<std::mutex> bucket_mutexes(buckets.size());

    std::vector<std::thread> threads;
    size_t chunk_size = (bucket.size() + num_threads_ - 1) / num_threads_;

    for (size_t t = 0; t < num_threads_; ++t) {
      threads.emplace_back([&, t]() {
        size_t start = t * chunk_size;
        size_t end = std::min(start + chunk_size, bucket.size());

        for (size_t i = start; i < end; ++i) {
          index_type u = bucket[i];

          if (processed_[u]) continue;
          processed_[u] = true;

          auto [neighbors_begin, neighbors_end] = graph_.getNeighborRange(u);

          for (auto neighbor_ptr = neighbors_begin;
               neighbor_ptr != neighbors_end; ++neighbor_ptr) {
            index_type v = *neighbor_ptr;
            WeightType weight = graph_.getEdgeValue(static_cast<index_type>(u),
                                                    static_cast<index_type>(v));

            WeightType new_dist = distance_[u] + weight;

            {
              std::lock_guard<std::mutex> lock(vertex_mutexes_[v]);

              if (new_dist < distance_[v]) {
                distance_[v] = new_dist;
                predecessor_[v] = u;

                size_t bucket_idx = static_cast<size_t>(new_dist / delta_);
                if (bucket_idx >= buckets.size()) {
                  bucket_idx = buckets.size() - 1;
                }

                std::lock_guard<std::mutex> bucket_lock(
                    bucket_mutexes[bucket_idx]);
                buckets[bucket_idx].push_back(v);
              }
            }
          }
        }
      });
    }

    for (auto& thread : threads) {
      thread.join();
    }
  }
};

}  // namespace parallel_algorithms
}  // namespace graph
