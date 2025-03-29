#pragma once

#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>

namespace graph {

template <typename V, typename W>
class MultiGraph {
 public:
  struct Edge {
    V to;
    W weight;
    int id;
  };

 private:
  std::unordered_map<V, std::vector<Edge>> adjacency_list;
  int next_edge_id = 0;

 public:
  MultiGraph& addVertex(const V& vertex) {
    if (adjacency_list.find(vertex) == adjacency_list.end()) {
      adjacency_list[vertex] = std::vector<Edge>();
    }
    return *this;
  }

  MultiGraph& addEdge(const V& from, const V& to, const W& weight) {
    addVertex(from);
    addVertex(to);

    int edge_id = next_edge_id++;

    adjacency_list[from].push_back({to, weight, edge_id});
    adjacency_list[to].push_back({from, weight, edge_id});

    return *this;
  }

  std::vector<V> getVertices() const {
    std::vector<V> vertices;
    for (const auto& pair : adjacency_list) {
      vertices.push_back(pair.first);
    }
    return vertices;
  }

  std::vector<V> getNeighbors(const V& vertex) const {
    std::vector<V> neighbors;
    auto it = adjacency_list.find(vertex);
    if (it != adjacency_list.end()) {
      for (const auto& edge : it->second) {
        neighbors.push_back(edge.to);
      }
    }
    return neighbors;
  }

  const std::vector<Edge>& getEdges(const V& vertex) const {
    static const std::vector<Edge> empty;
    auto it = adjacency_list.find(vertex);
    if (it != adjacency_list.end()) {
      return it->second;
    }
    return empty;
  }

  W getEdgeWeight(const V& from, const V& to) const {
    auto it = adjacency_list.find(from);
    if (it != adjacency_list.end()) {
      for (const auto& edge : it->second) {
        if (edge.to == to) {
          return edge.weight;
        }
      }
    }
    throw std::runtime_error("Edge not found");
  }

  std::vector<std::pair<int, W>> getAllEdges(const V& from, const V& to) const {
    std::vector<std::pair<int, W>> result;
    auto it = adjacency_list.find(from);
    if (it != adjacency_list.end()) {
      for (const auto& edge : it->second) {
        if (edge.to == to) {
          result.push_back({edge.id, edge.weight});
        }
      }
    }
    return result;
  }

  std::vector<std::tuple<V, V, W, int>> getAllUniquedEdges() const {
    std::vector<std::tuple<V, V, W, int>> result;
    std::unordered_map<int, bool> processed_edge_ids;

    for (const auto& [vertex, edges] : adjacency_list) {
      for (const auto& edge : edges) {
        if (processed_edge_ids.find(edge.id) == processed_edge_ids.end()) {
          result.push_back(
              std::make_tuple(vertex, edge.to, edge.weight, edge.id));
          processed_edge_ids[edge.id] = true;
        }
      }
    }
    return result;
  }
};

}  // namespace graph
