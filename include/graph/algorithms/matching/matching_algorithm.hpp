#pragma once

#include <utility>
#include <vector>

namespace graph {
namespace algorithms {
namespace matching {

template <typename Graph, typename VertexType>
class MatchingAlgorithm {
 public:
  explicit MatchingAlgorithm(const Graph& graph) : graph_(graph) {}
  virtual ~MatchingAlgorithm() = default;

  virtual void compute() = 0;

  virtual std::vector<std::pair<VertexType, VertexType>> getMatching()
      const = 0;

  virtual size_t getMatchingSize() const { return getMatching().size(); }

  virtual bool isVertexMatched(const VertexType& vertex) const = 0;

  virtual VertexType getMatchedVertex(const VertexType& vertex) const = 0;

 protected:
  const Graph& graph_;
};

}  // namespace matching
}  // namespace algorithms
}  // namespace graph
