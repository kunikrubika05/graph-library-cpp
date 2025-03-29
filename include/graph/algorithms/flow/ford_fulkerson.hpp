#pragma once

#include <algorithm>
#include <limits>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "flow_algorithm.hpp"

namespace graph {
namespace algorithms {
namespace flow {

template <typename Graph, typename VertexType, typename WeightType>
class FordFulkerson : public FlowAlgorithm<Graph, VertexType, WeightType> {
 public:
  FordFulkerson(const Graph& graph, const VertexType& source,
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
        if (flow_.find({u, v}) == flow_.end()) {
          flow_[{u, v}] = 0;
          flow_[{v, u}] = 0;
        }
      }
    }
  }

  void compute() override {
    this->max_flow_ = 0;
    WeightType max_iterations = 100000 * this->graph_.edgeCount();

    for (WeightType iter = 0; iter < max_iterations; ++iter) {
      WeightType path_flow = findAugmentingPath();
      if (path_flow == 0) break;
      this->max_flow_ += path_flow;
    }
  }

  WeightType getFlow(const VertexType& u, const VertexType& v) const override {
    auto it = flow_.find({u, v});
    if (it != flow_.end()) {
      return it->second;
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
  mutable std::unordered_map<VertexType, VertexType> parent_;
  mutable std::unordered_set<VertexType> visited_;

  WeightType findAugmentingPath() {
    parent_.clear();
    visited_.clear();
    std::stack<std::pair<VertexType, WeightType>> stack;
    const WeightType INF = std::numeric_limits<WeightType>::max();

    stack.push({this->source_, INF});
    visited_.insert(this->source_);
    parent_[this->source_] = this->source_;

    WeightType result_flow = 0;

    while (!stack.empty() && result_flow == 0) {
      auto [current, flow_in_path] = stack.top();
      stack.pop();

      if (current == this->sink_) {
        result_flow = flow_in_path;
        break;
      }

      for (const auto& next : this->graph_.getNeighbors(current)) {
        if (visited_.find(next) == visited_.end()) {
          WeightType capacity = this->graph_.getEdgeWeight(current, next);
          WeightType current_flow = getFlow(current, next);

          if (capacity > current_flow) {
            WeightType new_flow =
                std::min(flow_in_path, capacity - current_flow);
            parent_[next] = current;
            visited_.insert(next);
            stack.push({next, new_flow});
          }
        }
      }

      for (const auto& prev : this->graph_.getVertices()) {
        if (visited_.find(prev) == visited_.end()) {
          auto flow_it = flow_.find({prev, current});
          if (flow_it != flow_.end() && flow_it->second > 0) {
            WeightType new_flow = std::min(flow_in_path, flow_it->second);
            parent_[prev] = current;
            visited_.insert(prev);
            stack.push({prev, new_flow});
          }
        }
      }
    }

    if (result_flow > 0) {
      VertexType current = this->sink_;
      while (current != this->source_) {
        VertexType prev = parent_[current];
        if (this->graph_.hasEdge(prev, current)) {
          flow_[{prev, current}] += result_flow;
        } else {
          flow_[{current, prev}] -= result_flow;
        }
        current = prev;
      }
    }

    return result_flow;
  }
};

}  // namespace flow
}  // namespace algorithms
}  // namespace graph
