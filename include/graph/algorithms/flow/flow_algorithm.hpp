#pragma once

#include <limits>
#include <vector>

namespace graph {
namespace algorithms {
namespace flow {

template <typename Graph, typename VertexType, typename WeightType>
class FlowAlgorithm {
 public:
  FlowAlgorithm(const Graph& graph, const VertexType& source,
                const VertexType& sink)
      : graph_(graph), source_(source), sink_(sink), max_flow_(0) {}

  virtual ~FlowAlgorithm() = default;

  virtual void compute() = 0;

  WeightType getMaxFlow() const { return max_flow_; }

  virtual WeightType getFlow(const VertexType& u,
                             const VertexType& v) const = 0;

  virtual std::vector<std::tuple<VertexType, VertexType, WeightType>>
  getFlowEdges() const = 0;

 protected:
  const Graph& graph_;
  VertexType source_;
  VertexType sink_;
  WeightType max_flow_;
};

}  // namespace flow
}  // namespace algorithms
}  // namespace graph
