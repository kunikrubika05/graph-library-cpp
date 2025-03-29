#pragma once

#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

#include "flow_algorithm.hpp"

namespace graph {
namespace algorithms {
namespace flow {

template <typename Graph, typename VertexType, typename WeightType>
class Dinic : public FlowAlgorithm<Graph, VertexType, WeightType> {
 public:
  Dinic(const Graph& graph, const VertexType& source, const VertexType& sink)
      : FlowAlgorithm<Graph, VertexType, WeightType>(graph, source, sink) {
    bool source_exists = false;
    bool sink_exists = false;

    VertexType max_vertex = 0;
    for (const auto& vertex : graph.getVertices()) {
      if (vertex == source) source_exists = true;
      if (vertex == sink) sink_exists = true;
      max_vertex = std::max(max_vertex, vertex);
    }

    if (!source_exists || !sink_exists) {
      throw std::invalid_argument(
          "Source or sink vertex does not exist in the graph");
    }

    level_.resize(max_vertex + 1, -1);
    next_.resize(max_vertex + 1, 0);

    for (const auto& u : graph.getVertices()) {
      residual_network_[u] = {};
    }

    for (const auto& u : graph.getVertices()) {
      for (const auto& v : graph.getNeighbors(u)) {
        WeightType capacity = graph.getEdgeWeight(u, v);

        bool exists = false;
        for (size_t i = 0; i < residual_network_[u].size(); ++i) {
          if (residual_network_[u][i].to == v) {
            residual_network_[u][i].capacity += capacity;
            exists = true;
            break;
          }
        }

        if (!exists) {
          size_t fwd_idx = residual_network_[u].size();
          size_t rev_idx = residual_network_[v].size();

          residual_network_[u].push_back({v, capacity, 0, rev_idx});
          residual_network_[v].push_back({u, 0, 0, fwd_idx});
        }
      }
    }
  }

  void compute() override {
    this->max_flow_ = 0;

    while (buildLevelGraph()) {
      std::fill(next_.begin(), next_.end(), 0);

      WeightType flow;
      do {
        flow = findBlockingFlow(this->source_,
                                std::numeric_limits<WeightType>::max());
        this->max_flow_ += flow;
      } while (flow > 0);
    }
  }

  WeightType getFlow(const VertexType& u, const VertexType& v) const override {
    for (const auto& edge : residual_network_.at(u)) {
      if (edge.to == v) {
        return edge.flow;
      }
    }
    return 0;
  }

  std::vector<std::tuple<VertexType, VertexType, WeightType>> getFlowEdges()
      const override {
    std::vector<std::tuple<VertexType, VertexType, WeightType>> result;

    for (const auto& [u, edges] : residual_network_) {
      for (const auto& edge : edges) {
        if (edge.flow > 0) {
          result.push_back(std::make_tuple(u, edge.to, edge.flow));
        }
      }
    }

    return result;
  }

 private:
  struct Edge {
    VertexType to;
    WeightType capacity;
    WeightType flow;
    size_t rev_idx;
  };

  std::unordered_map<VertexType, std::vector<Edge>> residual_network_;
  std::vector<int> level_;
  std::vector<size_t> next_;

  bool buildLevelGraph() {
    std::fill(level_.begin(), level_.end(), -1);

    std::queue<VertexType> q;
    q.push(this->source_);
    level_[this->source_] = 0;

    while (!q.empty()) {
      VertexType u = q.front();
      q.pop();

      for (const auto& edge : residual_network_[u]) {
        if (level_[edge.to] == -1 && edge.capacity > edge.flow) {
          level_[edge.to] = level_[u] + 1;
          q.push(edge.to);
        }
      }
    }

    return level_[this->sink_] != -1;
  }

  WeightType findBlockingFlow(VertexType u, WeightType flow) {
    if (u == this->sink_) {
      return flow;
    }

    for (size_t& i = next_[u]; i < residual_network_[u].size(); ++i) {
      Edge& edge = residual_network_[u][i];

      if (level_[edge.to] == level_[u] + 1 && edge.capacity > edge.flow) {
        WeightType bottleneck = std::min(flow, edge.capacity - edge.flow);
        WeightType pushed_flow = findBlockingFlow(edge.to, bottleneck);

        if (pushed_flow > 0) {
          edge.flow += pushed_flow;
          residual_network_[edge.to][edge.rev_idx].flow -= pushed_flow;
          return pushed_flow;
        }
      }
    }

    return 0;
  }
};

}  // namespace flow
}  // namespace algorithms
}  // namespace graph
