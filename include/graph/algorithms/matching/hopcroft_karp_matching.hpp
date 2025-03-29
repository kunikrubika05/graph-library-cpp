#pragma once

#include <algorithm>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "graph/algorithms/dfs/bipartite_checker.hpp"
#include "matching_algorithm.hpp"

namespace graph {
namespace algorithms {
namespace matching {

template <typename Graph, typename VertexType>
class HopcroftKarpMatching : public MatchingAlgorithm<Graph, VertexType> {
 public:
  explicit HopcroftKarpMatching(const Graph& graph)
      : MatchingAlgorithm<Graph, VertexType>(graph), NIL(0) {
    graph::algorithms::BipartiteChecker<Graph> checker(graph);

    if (!checker.isBipartite()) {
      throw std::invalid_argument("The graph is not bipartite");
    }

    auto partition = checker.getPartition();
    left_part_ = partition.first;
    right_part_ = partition.second;

    initializeStructures();
  }

  HopcroftKarpMatching(const Graph& graph,
                       const std::vector<VertexType>& left_part)
      : MatchingAlgorithm<Graph, VertexType>(graph),
        left_part_(left_part),
        NIL(0) {
    for (const auto& vertex : left_part_) {
      if (!graph.hasVertex(vertex)) {
        throw std::invalid_argument(
            "Vertex in left part does not exist in the graph");
      }
    }

    right_part_.clear();
    std::unordered_set<VertexType> left_set(left_part_.begin(),
                                            left_part_.end());

    for (const auto& vertex : graph.getVertices()) {
      if (left_set.find(vertex) == left_set.end()) {
        right_part_.push_back(vertex);
      }
    }

    for (const auto& left : left_part_) {
      for (const auto& neighbor : graph.getNeighbors(left)) {
        if (left_set.find(neighbor) != left_set.end()) {
          throw std::invalid_argument(
              "The graph is not bipartite according to the given left part");
        }
      }
    }

    initializeStructures();
  }

  void compute() override {
    matching_size_ = 0;

    while (bfs()) {
      for (const auto& u : left_part_) {
        if (mate_left_[u] == NIL && dfs(u)) {
          matching_size_++;
        }
      }
    }
  }

  std::vector<std::pair<VertexType, VertexType>> getMatching() const override {
    std::vector<std::pair<VertexType, VertexType>> result;
    for (const auto& u : left_part_) {
      if (mate_left_[u] != NIL) {
        result.emplace_back(u, mate_left_[u]);
      }
    }
    return result;
  }

  size_t getMatchingSize() const override { return matching_size_; }

  bool isVertexMatched(const VertexType& vertex) const override {
    if (left_vertices_.find(vertex) != left_vertices_.end()) {
      return mate_left_[vertex] != NIL;
    } else {
      return mate_right_[vertex] != NIL;
    }
  }

  VertexType getMatchedVertex(const VertexType& vertex) const override {
    if (left_vertices_.find(vertex) != left_vertices_.end()) {
      if (mate_left_[vertex] != NIL) {
        return mate_left_[vertex];
      }
    } else {
      if (mate_right_[vertex] != NIL) {
        return mate_right_[vertex];
      }
    }

    throw std::runtime_error("Vertex is not matched");
  }

 private:
  std::vector<VertexType> left_part_;
  std::vector<VertexType> right_part_;
  std::unordered_set<VertexType> left_vertices_;
  std::vector<VertexType> mate_left_;
  std::vector<VertexType> mate_right_;
  std::vector<int> dist_;
  size_t matching_size_;
  const VertexType NIL;

  void initializeStructures() {
    VertexType max_vertex = 0;
    for (const auto& vertex : this->graph_.getVertices()) {
      max_vertex = std::max(max_vertex, vertex);
    }

    mate_left_.assign(max_vertex + 1, NIL);
    mate_right_.assign(max_vertex + 1, NIL);
    dist_.assign(max_vertex + 1, 0);

    for (const auto& vertex : left_part_) {
      left_vertices_.insert(vertex);
    }
  }

  bool bfs() {
    std::queue<VertexType> q;

    for (const auto& u : left_part_) {
      if (mate_left_[u] == NIL) {
        dist_[u] = 0;
        q.push(u);
      } else {
        dist_[u] = std::numeric_limits<int>::max();
      }
    }

    dist_[NIL] = std::numeric_limits<int>::max();

    while (!q.empty()) {
      VertexType u = q.front();
      q.pop();

      if (dist_[u] < dist_[NIL]) {
        for (const auto& v : this->graph_.getNeighbors(u)) {
          if (dist_[mate_right_[v]] == std::numeric_limits<int>::max()) {
            dist_[mate_right_[v]] = dist_[u] + 1;
            q.push(mate_right_[v]);
          }
        }
      }
    }

    return dist_[NIL] != std::numeric_limits<int>::max();
  }

  bool dfs(VertexType u) {
    if (u == NIL) {
      return true;
    }

    for (const auto& v : this->graph_.getNeighbors(u)) {
      if (dist_[mate_right_[v]] == dist_[u] + 1 && dfs(mate_right_[v])) {
        mate_right_[v] = u;
        mate_left_[u] = v;
        return true;
      }
    }

    dist_[u] = std::numeric_limits<int>::max();
    return false;
  }
};

}  // namespace matching
}  // namespace algorithms
}  // namespace graph
