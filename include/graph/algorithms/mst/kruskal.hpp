#pragma once

#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace graph {
namespace algorithms {

template <typename T>
class DisjointSet {
 public:
  DisjointSet() = default;

  void makeSet(const std::vector<T> &elements) {
    parent_.clear();
    rank_.clear();

    for (const auto &element : elements) {
      parent_[element] = element;
      rank_[element] = 0;
    }
  }

  T findSet(const T &element) {
    if (parent_.find(element) == parent_.end()) {
      throw std::runtime_error("Элемент не найден в системе множеств");
    }

    if (element != parent_[element]) {
      parent_[element] = findSet(parent_[element]);
    }

    return parent_[element];
  }

  void unionSets(T x, T y) {
    x = findSet(x);
    y = findSet(y);

    if (x == y) return;

    if (rank_[x] < rank_[y]) {
      parent_[x] = y;
    } else {
      parent_[y] = x;
      if (rank_[x] == rank_[y]) {
        ++rank_[x];
      }
    }
  }

 private:
  std::unordered_map<T, T> parent_;
  std::unordered_map<T, int> rank_;
};

template <typename VertexType, typename WeightType>
struct Edge {
  VertexType from;
  VertexType to;
  WeightType weight;

  bool operator<(const Edge &other) const { return weight < other.weight; }
  bool operator>(const Edge &other) const { return weight > other.weight; }
};

template <typename Graph>
class Kruskal {
 public:
  using vertex_type = typename std::remove_reference_t<
      decltype(*std::declval<Graph>().getVertices().begin())>;
  using weight_type =
      std::remove_reference_t<decltype(std::declval<Graph>().getEdgeWeight(
          *std::declval<Graph>().getVertices().begin(),
          *std::declval<Graph>()
               .getNeighbors(*std::declval<Graph>().getVertices().begin())
               .begin()))>;
  using edge_type = Edge<vertex_type, weight_type>;

  explicit Kruskal(const Graph &graph) : graph_(graph) {}

  void compute() {
    reset();

    if (graph_.vertexCount() == 0) {
      connected_ = false;
      return;
    }

    if (graph_.vertexCount() == 1) {
      connected_ = true;
      return;
    }

    std::vector<edge_type> edges = getAllEdges();

    std::sort(edges.begin(), edges.end());

    DisjointSet<vertex_type> dsu;
    dsu.makeSet(graph_.getVertices());

    for (const auto &edge : edges) {
      if (dsu.findSet(edge.from) != dsu.findSet(edge.to)) {
        mst_edges_.push_back(edge);
        total_weight_ += edge.weight;
        dsu.unionSets(edge.from, edge.to);
      }
    }

    if (mst_edges_.size() != graph_.vertexCount() - 1) {
      connected_ = false;
    } else {
      connected_ = true;
    }
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

  std::vector<edge_type> getAllEdges() const {
    std::vector<edge_type> edges;
    auto vertices = graph_.getVertices();

    std::unordered_map<vertex_type, std::unordered_set<vertex_type>>
        processed_edges;

    for (const auto &vertex : vertices) {
      auto neighbors = graph_.getNeighbors(vertex);
      for (const auto &neighbor : neighbors) {
        if (!processed_edges[neighbor].count(vertex)) {
          edges.push_back(
              {vertex, neighbor, graph_.getEdgeWeight(vertex, neighbor)});
          processed_edges[vertex].insert(neighbor);
        }
      }
    }

    return edges;
  }
};

}  // namespace algorithms
}  // namespace graph
