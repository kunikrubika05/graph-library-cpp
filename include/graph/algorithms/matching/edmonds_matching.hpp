#pragma once

#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "matching_algorithm.hpp"

namespace graph {
namespace algorithms {
namespace matching {

template <typename Graph, typename VertexType>
class EdmondsMatching : public MatchingAlgorithm<Graph, VertexType> {
 public:
  explicit EdmondsMatching(const Graph& graph)
      : MatchingAlgorithm<Graph, VertexType>(graph), NIL(0) {
    VertexType max_vertex = 0;
    for (const auto& vertex : graph.getVertices()) {
      max_vertex = std::max(max_vertex, vertex);
      vertices_.push_back(vertex);
    }

    match_.assign(max_vertex + 1, NIL);
    mark_.assign(max_vertex + 1, false);
    parent_.assign(max_vertex + 1, NIL);
    base_.assign(max_vertex + 1, NIL);
  }

  void compute() override {
    std::fill(match_.begin(), match_.end(), NIL);

    for (const auto& v : vertices_) {
      if (match_[v] == NIL) {
        findAugmentingPath(v);
      }
    }
  }

  std::vector<std::pair<VertexType, VertexType>> getMatching() const override {
    std::vector<std::pair<VertexType, VertexType>> result;
    std::unordered_set<VertexType> visited;

    for (const auto& v : vertices_) {
      if (match_[v] != NIL && visited.find(v) == visited.end()) {
        result.emplace_back(v, match_[v]);
        visited.insert(v);
        visited.insert(match_[v]);
      }
    }

    return result;
  }

  size_t getMatchingSize() const override { return getMatching().size(); }

  bool isVertexMatched(const VertexType& vertex) const override {
    return match_[vertex] != NIL;
  }

  VertexType getMatchedVertex(const VertexType& vertex) const override {
    if (match_[vertex] != NIL) {
      return match_[vertex];
    }

    throw std::runtime_error("Vertex is not matched");
  }

 private:
  std::vector<VertexType> vertices_;
  std::vector<VertexType> match_;
  std::vector<bool> mark_;
  std::vector<VertexType> parent_;
  std::vector<VertexType> base_;
  const VertexType NIL;

  static const int MAX_ITERATIONS = 10000;

  void findAugmentingPath(VertexType start) {
    std::fill(mark_.begin(), mark_.end(), false);
    std::fill(parent_.begin(), parent_.end(), NIL);

    for (VertexType i = 0; i < base_.size(); ++i) {
      base_[i] = i;
    }

    mark_[start] = true;
    std::queue<VertexType> q;
    q.push(start);

    while (!q.empty()) {
      VertexType u = q.front();
      q.pop();

      for (const auto& v : this->graph_.getNeighbors(u)) {
        if (base_[u] != base_[v] && v != match_[u]) {
          if ((v == start) || (match_[v] != NIL && parent_[match_[v]] != NIL)) {
            VertexType lca = findLowestCommonAncestor(u, v);

            markBlossom(u, v, lca);
            markBlossom(v, u, lca);
          } else if (parent_[v] == NIL) {
            parent_[v] = u;

            if (match_[v] == NIL) {
              augmentPath(v);
              return;
            } else {
              mark_[match_[v]] = true;
              q.push(match_[v]);
            }
          }
        }
      }
    }
  }

  VertexType findLowestCommonAncestor(VertexType u, VertexType v) {
    std::vector<bool> used(mark_.size(), false);

    while (true) {
      u = base_[u];
      used[u] = true;
      if (match_[u] == NIL || u == 0) break;
      u = parent_[match_[u]];
    }

    while (true) {
      v = base_[v];
      if (used[v]) return v;
      if (match_[v] == NIL || v == 0) break;
      v = parent_[match_[v]];
    }

    return 0;
  }

  void markBlossom(VertexType u, VertexType v, VertexType lca) {
    int iterations = 0;

    while (base_[u] != lca && iterations < MAX_ITERATIONS) {
      VertexType next_u = match_[u];
      VertexType next_next_u = parent_[next_u];

      if (base_[next_u] != lca) {
        parent_[next_u] = v;
      }

      base_[u] = base_[next_u] = lca;

      u = next_next_u;
      iterations++;
    }

    if (iterations >= MAX_ITERATIONS) {
      return;
    }
  }

  void augmentPath(VertexType v) {
    int iterations = 0;

    while (v != NIL && iterations < MAX_ITERATIONS) {
      VertexType pv = parent_[v];
      VertexType next_v = match_[pv];
      match_[v] = pv;
      match_[pv] = v;
      v = next_v;
      iterations++;
    }

    if (iterations >= MAX_ITERATIONS) {
      return;
    }
  }
};

}  // namespace matching
}  // namespace algorithms
}  // namespace graph
