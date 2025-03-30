#pragma once

#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace graph {
namespace structure {

template <typename VertexType, typename EdgeValueType = void>
class CSRGraphBuilder;

template <typename VertexType, typename EdgeValueType>
class CSRGraph {
 public:
  using index_type = std::size_t;

  CSRGraph() = default;

  template <typename Graph>
  explicit CSRGraph(const Graph& graph) {
    buildFromGraph(graph);
  }

  index_type getVertexCount() const { return vertex_count_; }

  index_type getEdgeCount() const { return edge_count_; }

  std::pair<const index_type*, const index_type*> getNeighborRange(
      index_type vertex_idx) const {
    assert(vertex_idx < vertex_count_ && "Vertex index out of bounds");

    index_type begin = row_offsets_[vertex_idx];
    index_type end = row_offsets_[vertex_idx + 1];

    return {&column_indices_[begin], &column_indices_[end]};
  }

  std::vector<index_type> getNeighbors(index_type vertex_idx) const {
    auto [begin, end] = getNeighborRange(vertex_idx);
    return std::vector<index_type>(begin, end);
  }

  std::vector<VertexType> getNeighbors(const VertexType& vertex) const {
    auto vertex_idx = getVertexIndex(vertex);
    auto indices = getNeighbors(vertex_idx);

    std::vector<VertexType> result;
    result.reserve(indices.size());
    for (auto idx : indices) {
      result.push_back(getVertexByIndex(idx));
    }
    return result;
  }

  const VertexType& getVertexByIndex(index_type idx) const {
    assert(idx < vertex_count_ && "Vertex index out of bounds");
    return index_to_vertex_[idx];
  }

  index_type getVertexIndex(const VertexType& vertex) const {
    auto it = vertex_to_index_.find(vertex);
    if (it == vertex_to_index_.end()) {
      throw std::runtime_error("Vertex not found in graph");
    }
    return it->second;
  }

  template <typename T = EdgeValueType>
  typename std::enable_if<!std::is_void<T>::value, T>::type getEdgeValue(
      index_type src_idx, index_type dst_idx) const {
    auto [begin, end] = getNeighborRange(src_idx);
    for (auto ptr = begin; ptr != end; ++ptr) {
      if (*ptr == dst_idx) {
        return edge_values_[ptr - column_indices_.data()];
      }
    }
    throw std::runtime_error("Edge not found");
  }

  template <typename T = EdgeValueType>
  typename std::enable_if<!std::is_void<T>::value, T>::type getEdgeValue(
      const VertexType& src, const VertexType& dst) const {
    return getEdgeValue(getVertexIndex(src), getVertexIndex(dst));
  }

  std::vector<VertexType> getVertices() const {
    std::vector<VertexType> vertices(vertex_count_);
    for (index_type i = 0; i < vertex_count_; ++i) {
      vertices[i] = index_to_vertex_[i];
    }
    return vertices;
  }

 private:
  std::vector<index_type> row_offsets_;
  std::vector<index_type> column_indices_;

  template <typename T = EdgeValueType>
  using EdgeValues =
      typename std::conditional<std::is_void<T>::value, std::vector<char>,
                                std::vector<T>>::type;
  EdgeValues<EdgeValueType> edge_values_;

  std::vector<VertexType> index_to_vertex_;
  std::unordered_map<VertexType, index_type> vertex_to_index_;

  index_type vertex_count_ = 0;
  index_type edge_count_ = 0;

  template <typename Graph>
  void buildFromGraph(const Graph& graph) {
    index_type idx = 0;
    for (const auto& vertex : graph.getVertices()) {
      vertex_to_index_[vertex] = idx;
      index_to_vertex_.push_back(vertex);
      idx++;
    }

    vertex_count_ = index_to_vertex_.size();
    row_offsets_.resize(vertex_count_ + 1, 0);

    for (index_type i = 0; i < vertex_count_; ++i) {
      const auto& vertex = index_to_vertex_[i];
      row_offsets_[i + 1] = row_offsets_[i] + graph.getNeighbors(vertex).size();
    }

    edge_count_ = row_offsets_[vertex_count_];
    column_indices_.resize(edge_count_);

    if constexpr (!std::is_void<EdgeValueType>::value) {
      edge_values_.resize(edge_count_);
    }

    for (index_type i = 0; i < vertex_count_; ++i) {
      const auto& vertex = index_to_vertex_[i];
      const auto& neighbors = graph.getNeighbors(vertex);

      index_type offset = row_offsets_[i];
      for (const auto& neighbor : neighbors) {
        column_indices_[offset] = vertex_to_index_[neighbor];

        if constexpr (!std::is_void<EdgeValueType>::value) {
          edge_values_[offset] = graph.getEdgeValue(vertex, neighbor);
        }

        offset++;
      }
    }
  }

  template <typename V, typename E>
  friend class CSRGraphBuilder;
};

}  // namespace structure
}  // namespace graph
