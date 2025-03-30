#pragma once

#include "csr_graph.hpp"
#include <vector>
#include <unordered_map>

namespace graph {
namespace structure {

template <typename VertexType, typename EdgeValueType>
class CSRGraphBuilder {
 public:
  using index_type = typename CSRGraph<VertexType, EdgeValueType>::index_type;
  using CSRGraphType = CSRGraph<VertexType, EdgeValueType>;

  CSRGraphBuilder(size_t vertex_count_hint = 0, size_t edge_count_hint = 0) {
    vertices_.reserve(vertex_count_hint);
    edges_.reserve(edge_count_hint);
  }

  CSRGraphBuilder& addVertex(const VertexType& vertex) {
    if (vertex_map_.find(vertex) == vertex_map_.end()) {
      vertex_map_[vertex] = vertices_.size();
      vertices_.push_back(vertex);
    }
    return *this;
  }

  template <typename T = EdgeValueType>
  typename std::enable_if<std::is_void<T>::value, CSRGraphBuilder&>::type
  addEdge(const VertexType& src, const VertexType& dst) {
    if (vertex_map_.find(src) == vertex_map_.end()) addVertex(src);
    if (vertex_map_.find(dst) == vertex_map_.end()) addVertex(dst);

    index_type src_idx = vertex_map_[src];
    index_type dst_idx = vertex_map_[dst];
    edges_.push_back({src_idx, dst_idx});

    return *this;
  }

  template <typename T = EdgeValueType>
  typename std::enable_if<!std::is_void<T>::value, CSRGraphBuilder&>::type
  addEdge(const VertexType& src, const VertexType& dst, const T& value) {
    if (vertex_map_.find(src) == vertex_map_.end()) addVertex(src);
    if (vertex_map_.find(dst) == vertex_map_.end()) addVertex(dst);

    index_type src_idx = vertex_map_[src];
    index_type dst_idx = vertex_map_[dst];
    edges_.push_back({src_idx, dst_idx, value});

    return *this;
  }

  CSRGraphType build() const {
    CSRGraphType result;

    const size_t vertex_count = vertices_.size();
    result.vertex_count_ = vertex_count;
    result.edge_count_ = edges_.size();

    result.index_to_vertex_ = vertices_;
    result.vertex_to_index_ = vertex_map_;

    result.row_offsets_.resize(vertex_count + 1, 0);

    for (const auto& edge : edges_) {
      result.row_offsets_[edge.src + 1]++;
    }

    for (size_t i = 0; i < vertex_count; i++) {
      result.row_offsets_[i + 1] += result.row_offsets_[i];
    }

    result.column_indices_.resize(edges_.size());
    if constexpr (!std::is_void<EdgeValueType>::value) {
      result.edge_values_.resize(edges_.size());
    }

    std::vector<index_type> current_offsets = result.row_offsets_;

    for (const auto& edge : edges_) {
      index_type pos = current_offsets[edge.src]++;
      result.column_indices_[pos] = edge.dst;

      if constexpr (!std::is_void<EdgeValueType>::value) {
        result.edge_values_[pos] = edge.value;
      }
    }

    return result;
  }

 private:
  struct BaseEdge {
    index_type src;
    index_type dst;
  };

  template <typename T>
  struct WeightedEdge {
    index_type src;
    index_type dst;
    T value;
  };

  using Edge = typename std::conditional<
      std::is_void<EdgeValueType>::value,
      BaseEdge,
      WeightedEdge<EdgeValueType>
      >::type;

  std::vector<VertexType> vertices_;
  std::unordered_map<VertexType, index_type> vertex_map_;
  std::vector<Edge> edges_;
};

}  // namespace structure
}  // namespace graph
