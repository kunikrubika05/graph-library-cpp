#pragma once

#include <cstddef>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <unordered_map>

#include "error_policy.hpp"
#include "graph_base.hpp"
#include "graph_exceptions.hpp"
#include "graph_options.hpp"

namespace graph {

template <typename VertexType, typename WeightType>
class DirectedAdjacencyMatrix
    : public GraphBase<DirectedAdjacencyMatrix<VertexType, WeightType>,
                       VertexType, WeightType> {
 public:
  using ErrorHandler = std::function<void(GraphErrorCode, const std::string &)>;

  DirectedAdjacencyMatrix(std::size_t maxVertices,
                          GraphOptions options = GraphOptions(),
                          ErrorPolicy errorPolicy = ErrorPolicy::SILENT_IGNORE)
      : GraphBase<DirectedAdjacencyMatrix, VertexType, WeightType>(options),
        capacity_(maxVertices),
        count_(0),
        errorPolicy_(errorPolicy) {
    matrix_ = new WeightType[capacity_ * capacity_];
    edgeExists_ = new bool[capacity_ * capacity_];
    vertices_ = new VertexType[capacity_];
    freeIndices_ = new std::size_t[capacity_];

    for (std::size_t i = 0; i < capacity_ * capacity_; ++i) {
      edgeExists_[i] = false;
    }

    for (std::size_t i = 0; i < capacity_; ++i) {
      freeIndices_[i] = i;
    }
    freeIndicesCount_ = capacity_;

    if (errorPolicy == ErrorPolicy::SILENT_IGNORE) {
      errorHandler_ = [](GraphErrorCode code, const std::string &msg) {
        std::cerr << "Ошибка: " << msg << "\n";
      };
    }
  }

  ~DirectedAdjacencyMatrix() override {
    delete[] matrix_;
    delete[] edgeExists_;
    delete[] vertices_;
    delete[] freeIndices_;
  }

  [[nodiscard]] std::size_t vertexCount() const override { return count_; }

  [[nodiscard]] std::size_t edgeCount() const override {
    std::size_t result = 0;
    for (std::size_t i = 0; i < capacity_ * capacity_; ++i) {
      if (edgeExists_[i]) ++result;
    }
    return result;
  }

  DirectedAdjacencyMatrix &addVertex(const VertexType &vertex) override {
    if (indices_.find(vertex) != indices_.end()) {
      handleError(GraphErrorCode::VERTEX_ALREADY_EXISTS);
      return *this;
    }
    if (freeIndicesCount_ == 0) {
      handleError(GraphErrorCode::OPERATION_NOT_SUPPORTED);
      return *this;
    }
    std::size_t index = freeIndices_[--freeIndicesCount_];
    indices_[vertex] = index;
    vertices_[index] = vertex;
    ++count_;
    return *this;
  }

  DirectedAdjacencyMatrix &removeVertex(const VertexType &vertex) override {
    auto it = indices_.find(vertex);
    if (it == indices_.end()) {
      handleError(GraphErrorCode::VERTEX_NOT_FOUND);
      return *this;
    }
    std::size_t index = it->second;
    for (std::size_t col = 0; col < capacity_; ++col) {
      edgeExists_[index * capacity_ + col] = false;
    }
    for (std::size_t row = 0; row < capacity_; ++row) {
      edgeExists_[row * capacity_ + index] = false;
    }
    indices_.erase(it);
    --count_;
    freeIndices_[freeIndicesCount_++] = index;
    return *this;
  }

  bool hasVertex(const VertexType &vertex) const override {
    return indices_.find(vertex) != indices_.end();
  }

  std::vector<VertexType> getVertices() const override {
    std::vector<VertexType> result;
    for (const auto &pair : indices_) {
      result.push_back(pair.first);
    }
    return result;
  }

  DirectedAdjacencyMatrix &addEdge(const VertexType &from, const VertexType &to,
                                   const WeightType &weight) override {
    auto fromIt = indices_.find(from);
    auto toIt = indices_.find(to);
    if (fromIt == indices_.end() || toIt == indices_.end()) {
      handleError(GraphErrorCode::VERTEX_NOT_FOUND);
      return *this;
    }
    if (from == to && !this->options_.allowSelfLoops) {
      handleError(GraphErrorCode::SELF_LOOPS_FORBIDDEN);
      return *this;
    }
    std::size_t pos = fromIt->second * capacity_ + toIt->second;
    if (this->options_.checkDuplicateEdges && edgeExists_[pos]) {
      handleError(GraphErrorCode::EDGE_ALREADY_EXISTS);
      return *this;
    }
    matrix_[pos] = weight;
    edgeExists_[pos] = true;
    return *this;
  }

  DirectedAdjacencyMatrix &removeEdge(const VertexType &from,
                                      const VertexType &to) override {
    auto fromIt = indices_.find(from);
    auto toIt = indices_.find(to);
    if (fromIt == indices_.end() || toIt == indices_.end()) {
      handleError(GraphErrorCode::VERTEX_NOT_FOUND);
      return *this;
    }
    std::size_t pos = fromIt->second * capacity_ + toIt->second;
    if (!edgeExists_[pos]) {
      handleError(GraphErrorCode::EDGE_NOT_FOUND);
      return *this;
    }
    edgeExists_[pos] = false;
    return *this;
  }

  bool hasEdge(const VertexType &from, const VertexType &to) const override {
    auto fromIt = indices_.find(from);
    auto toIt = indices_.find(to);
    if (fromIt == indices_.end() || toIt == indices_.end()) return false;
    std::size_t pos = fromIt->second * capacity_ + toIt->second;
    return edgeExists_[pos];
  }

  std::vector<VertexType> getNeighbors(
      const VertexType &vertex) const override {
    auto it = indices_.find(vertex);
    if (it == indices_.end()) {
      throw vertexNotFound();
    }
    std::vector<VertexType> neighbors;
    std::size_t row = it->second;
    for (std::size_t col = 0; col < capacity_; ++col) {
      if (edgeExists_[row * capacity_ + col]) {
        for (const auto &pr : indices_) {
          if (pr.second == col) {
            neighbors.push_back(pr.first);
            break;
          }
        }
      }
    }
    return neighbors;
  }

  WeightType getEdgeWeight(const VertexType &from,
                           const VertexType &to) const override {
    auto fromIt = indices_.find(from);
    auto toIt = indices_.find(to);
    if (fromIt == indices_.end() || toIt == indices_.end()) {
      throw vertexNotFound();
    }
    std::size_t pos = fromIt->second * capacity_ + toIt->second;
    if (!edgeExists_[pos]) {
      throw edgeNotFound();
    }
    return matrix_[pos];
  }

  DirectedAdjacencyMatrix &setErrorHandler(ErrorHandler handler) {
    errorHandler_ = handler;
    return *this;
  }

  DirectedAdjacencyMatrix &setErrorPolicy(ErrorPolicy policy) {
    errorPolicy_ = policy;
    return *this;
  }

 private:
  std::size_t capacity_;
  std::size_t count_;
  WeightType *matrix_;
  bool *edgeExists_;
  VertexType *vertices_;
  std::size_t *freeIndices_;
  std::size_t freeIndicesCount_;
  std::unordered_map<VertexType, std::size_t> indices_;
  ErrorPolicy errorPolicy_;
  ErrorHandler errorHandler_ = nullptr;

  void handleError(GraphErrorCode code) {
    if (errorHandler_) {
      errorHandler_(code, error_messages.at(code));
    }
    if (errorPolicy_ == ErrorPolicy::THROW_EXCEPTIONS) {
      switch (code) {
        case GraphErrorCode::VERTEX_ALREADY_EXISTS:
          throw vertexAlreadyExists();
        case GraphErrorCode::VERTEX_NOT_FOUND:
          throw vertexNotFound();
        case GraphErrorCode::EDGE_ALREADY_EXISTS:
          throw edgeAlreadyExists();
        case GraphErrorCode::EDGE_NOT_FOUND:
          throw edgeNotFound();
        case GraphErrorCode::SELF_LOOPS_FORBIDDEN:
          throw selfLoopsForbidden();
        case GraphErrorCode::OPERATION_NOT_SUPPORTED:
          throw operationNotSupported();
      }
    }
  }
};

}  // namespace graph
