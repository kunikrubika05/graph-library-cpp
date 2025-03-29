#pragma once

#include <algorithm>
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
class KuhnMatching : public MatchingAlgorithm<Graph, VertexType> {
 public:
  explicit KuhnMatching(const Graph& graph)
      : MatchingAlgorithm<Graph, VertexType>(graph) {
    graph::algorithms::BipartiteChecker<Graph> checker(graph);

    if (!checker.isBipartite()) {
      throw std::invalid_argument("The graph is not bipartite");
    }

    auto partition = checker.getPartition();
    left_part_ = partition.first;
    right_part_ = partition.second;
  }

  KuhnMatching(const Graph& graph, const std::vector<VertexType>& left_part)
      : MatchingAlgorithm<Graph, VertexType>(graph), left_part_(left_part) {
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
  }

  void compute() override {
    matching_left_to_right_.clear();
    matching_right_to_left_.clear();

    for (const auto& vertex : right_part_) {
      matching_right_to_left_[vertex] = VertexType{};
    }

    bool found_new_match;
    do {
      found_new_match = false;
      for (const auto& vertex : left_part_) {
        if (matching_left_to_right_.find(vertex) ==
            matching_left_to_right_.end()) {
          used_.clear();
          if (tryKuhn(vertex)) {
            found_new_match = true;
          }
        }
      }
    } while (found_new_match);
  }

  std::vector<std::pair<VertexType, VertexType>> getMatching() const override {
    std::vector<std::pair<VertexType, VertexType>> result;
    for (const auto& [left, right] : matching_left_to_right_) {
      result.emplace_back(left, right);
    }
    return result;
  }

  size_t getMatchingSize() const override {
    return matching_left_to_right_.size();
  }

  bool isVertexMatched(const VertexType& vertex) const override {
    auto it_left = matching_left_to_right_.find(vertex);
    if (it_left != matching_left_to_right_.end()) {
      return true;
    }

    auto it_right = matching_right_to_left_.find(vertex);
    if (it_right != matching_right_to_left_.end()) {
      return it_right->second != VertexType{};
    }

    return false;
  }

  VertexType getMatchedVertex(const VertexType& vertex) const override {
    auto it_left = matching_left_to_right_.find(vertex);
    if (it_left != matching_left_to_right_.end()) {
      return it_left->second;
    }

    auto it_right = matching_right_to_left_.find(vertex);
    if (it_right != matching_right_to_left_.end() &&
        it_right->second != VertexType{}) {
      return it_right->second;
    }

    throw std::runtime_error("Vertex is not matched");
  }

 private:
  std::vector<VertexType> left_part_;
  std::vector<VertexType> right_part_;
  std::unordered_map<VertexType, VertexType> matching_left_to_right_;
  std::unordered_map<VertexType, VertexType> matching_right_to_left_;
  mutable std::unordered_set<VertexType> used_;

  bool tryKuhn(const VertexType& vertex) {
    if (used_.find(vertex) != used_.end()) {
      return false;
    }
    used_.insert(vertex);

    for (const auto& to : this->graph_.getNeighbors(vertex)) {
      if (matching_right_to_left_.find(to) == matching_right_to_left_.end() ||
          matching_right_to_left_[to] == VertexType{} ||
          tryKuhn(matching_right_to_left_[to])) {
        matching_left_to_right_[vertex] = to;
        matching_right_to_left_[to] = vertex;
        return true;
      }
    }

    return false;
  }
};

}  // namespace matching
}  // namespace algorithms
}  // namespace graph
