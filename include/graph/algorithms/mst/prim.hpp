#pragma once

#include <algorithm>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace graph {
namespace algorithms {

template <typename VertexType, typename WeightType>
struct PrimEdge {
  VertexType from;
  VertexType to;
  WeightType weight;

  bool operator<(const PrimEdge &other) const { return weight < other.weight; }
  bool operator>(const PrimEdge &other) const { return weight > other.weight; }
};

template <typename Graph>
class Prim {
 public:
  using vertex_type = typename std::remove_reference_t<
      decltype(*std::declval<Graph>().getVertices().begin())>;
  using weight_type =
      std::remove_reference_t<decltype(std::declval<Graph>().getEdgeWeight(
          *std::declval<Graph>().getVertices().begin(),
          *std::declval<Graph>()
               .getNeighbors(*std::declval<Graph>().getVertices().begin())
               .begin()))>;
  using edge_type = PrimEdge<vertex_type, weight_type>;

  explicit Prim(const Graph &graph) : graph_(graph) {}

  void compute(const vertex_type &start_vertex = vertex_type()) {
    reset();

    if (graph_.vertexCount() == 0) {
      connected_ = false;
      return;
    }

    if (graph_.vertexCount() == 1) {
      connected_ = true;
      return;
    }

    vertex_type start = start_vertex;
    if (!graph_.hasVertex(start)) {
      start = *graph_.getVertices().begin();
    }

    std::unordered_set<vertex_type> visited;
    visited.insert(start);

    std::priority_queue<edge_type, std::vector<edge_type>,
                        std::greater<edge_type>>
        pq;

    addEdgesToQueue(start, visited, pq);

    while (!pq.empty() && visited.size() < graph_.vertexCount()) {
      edge_type edge = pq.top();
      pq.pop();

      if (visited.count(edge.to)) {
        continue;
      }

      mst_edges_.push_back(edge);
      total_weight_ += edge.weight;

      visited.insert(edge.to);

      addEdgesToQueue(edge.to, visited, pq);
    }

    connected_ = (visited.size() == graph_.vertexCount());
  }

  bool isConnected() const { return connected_; }

  weight_type getTotalWeight() const {
    if (!connected_) {
      throw std::runtime_error(
          "Граф не связный, минимальное остовное дерево не существует");
    }
    return total_weight_;
  }

  const std::vector<edge_type> &getMSTEdges() const {
    if (!connected_) {
      throw std::runtime_error(
          "Граф не связный, минимальное остовное дерево не существует");
    }
    return mst_edges_;
  }

 private:
  const Graph &graph_;
  std::vector<edge_type> mst_edges_;
  weight_type total_weight_ = 0;
  bool connected_ = false;

  void reset() {
    mst_edges_.clear();
    total_weight_ = 0;
    connected_ = false;
  }

  void addEdgesToQueue(const vertex_type &vertex,
                       const std::unordered_set<vertex_type> &visited,
                       std::priority_queue<edge_type, std::vector<edge_type>,
                                           std::greater<edge_type>> &pq) {
    for (const auto &neighbor : graph_.getNeighbors(vertex)) {
      if (!visited.count(neighbor)) {
        pq.push({vertex, neighbor, graph_.getEdgeWeight(vertex, neighbor)});
      }
    }
  }
};

}  // namespace algorithms
}  // namespace graph
