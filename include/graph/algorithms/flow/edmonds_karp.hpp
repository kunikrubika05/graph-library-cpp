#pragma once

#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "flow_algorithm.hpp"

namespace graph {
namespace algorithms {
namespace flow {

template <typename Graph, typename VertexType, typename WeightType>
class EdmondsKarp : public FlowAlgorithm<Graph, VertexType, WeightType> {
 public:
  EdmondsKarp(const Graph& graph, const VertexType& source,
              const VertexType& sink)
      : FlowAlgorithm<Graph, VertexType, WeightType>(graph, source, sink) {
    bool source_exists = false;
    bool sink_exists = false;
    for (const auto& vertex : graph.getVertices()) {
      if (vertex == source) source_exists = true;
      if (vertex == sink) sink_exists = true;
    }

    if (!source_exists || !sink_exists) {
      throw std::invalid_argument(
          "Source or sink vertex does not exist in the graph");
    }

    for (const auto& u : graph.getVertices()) {
      for (const auto& v : graph.getNeighbors(u)) {
        flow_[{u, v}] = 0;
      }
    }
  }

  void compute() override {
    this->max_flow_ = 0;

    while (true) {
      auto path = findAugmentingPath();
      if (path.empty()) {
        break;
      }

      WeightType path_capacity = findPathCapacity(path);

      augmentFlow(path, path_capacity);

      this->max_flow_ += path_capacity;
    }
  }

  WeightType getFlow(const VertexType& u, const VertexType& v) const override {
    auto it = flow_.find({u, v});
    if (it != flow_.end()) {
      return it->second;
    }

    it = flow_.find({v, u});
    if (it != flow_.end()) {
      return -it->second;
    }

    return 0;
  }

  std::vector<std::tuple<VertexType, VertexType, WeightType>> getFlowEdges()
      const override {
    std::vector<std::tuple<VertexType, VertexType, WeightType>> result;
    for (const auto& [edge, flow] : flow_) {
      if (flow > 0) {
        result.push_back(std::make_tuple(edge.first, edge.second, flow));
      }
    }
    return result;
  }

 private:
  using EdgeKey = std::pair<VertexType, VertexType>;

  struct PairHash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
      return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
  };

  std::unordered_map<EdgeKey, WeightType, PairHash> flow_;

  std::vector<VertexType> findAugmentingPath() {
    std::unordered_map<VertexType, VertexType> parent;
    std::queue<VertexType> queue;

    queue.push(this->source_);
    parent[this->source_] = this->source_;

    while (!queue.empty() && parent.find(this->sink_) == parent.end()) {
      auto current = queue.front();
      queue.pop();

      for (const auto& neighbor : this->graph_.getNeighbors(current)) {
        if (parent.find(neighbor) == parent.end()) {
          WeightType capacity = this->graph_.getEdgeWeight(current, neighbor);
          WeightType flow_value = getFlow(current, neighbor);

          if (capacity > flow_value) {
            parent[neighbor] = current;
            queue.push(neighbor);
          }
        }
      }

      for (const auto& vertex : this->graph_.getVertices()) {
        if (parent.find(vertex) == parent.end()) {
          bool edge_exists = false;
          for (const auto& adj : this->graph_.getNeighbors(vertex)) {
            if (adj == current) {
              edge_exists = true;
              break;
            }
          }

          if (edge_exists) {
            WeightType flow_value = getFlow(vertex, current);
            if (flow_value > 0) {
              parent[vertex] = current;
              queue.push(vertex);
            }
          }
        }
      }
    }

    std::vector<VertexType> path;
    if (parent.find(this->sink_) != parent.end()) {
      VertexType current = this->sink_;
      while (current != this->source_) {
        path.push_back(current);
        current = parent[current];
      }
      path.push_back(this->source_);
      std::reverse(path.begin(), path.end());
    }

    return path;
  }

  WeightType findPathCapacity(const std::vector<VertexType>& path) {
    if (path.size() < 2) {
      return 0;
    }

    WeightType min_capacity = std::numeric_limits<WeightType>::max();

    for (size_t i = 0; i < path.size() - 1; ++i) {
      VertexType u = path[i];
      VertexType v = path[i + 1];

      bool forward_edge = false;
      for (const auto& adj : this->graph_.getNeighbors(u)) {
        if (adj == v) {
          forward_edge = true;
          break;
        }
      }

      WeightType residual;

      if (forward_edge) {
        WeightType capacity = this->graph_.getEdgeWeight(u, v);
        WeightType flow_value = getFlow(u, v);
        residual = capacity - flow_value;
      } else {
        WeightType flow_value = getFlow(v, u);
        residual = flow_value;
      }

      min_capacity = std::min(min_capacity, residual);
    }

    return min_capacity;
  }

  void augmentFlow(const std::vector<VertexType>& path, WeightType amount) {
    for (size_t i = 0; i < path.size() - 1; ++i) {
      VertexType u = path[i];
      VertexType v = path[i + 1];

      bool forward_edge = false;
      for (const auto& adj : this->graph_.getNeighbors(u)) {
        if (adj == v) {
          forward_edge = true;
          break;
        }
      }

      if (forward_edge) {
        flow_[{u, v}] += amount;
      } else {
        flow_[{v, u}] -= amount;
      }
    }
  }
};

}  // namespace flow
}  // namespace algorithms
}  // namespace graph
