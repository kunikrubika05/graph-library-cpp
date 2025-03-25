#pragma once

#include <cstddef>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "graph/structure/error_policy.hpp"
#include "graph/structure/graph_base.hpp"
#include "graph/structure/graph_exceptions.hpp"
#include "graph/structure/graph_options.hpp"

namespace graph {

template <typename VertexType, typename WeightType>
class UndirectedAdjacencyMatrix
    : public GraphBase<UndirectedAdjacencyMatrix<VertexType, WeightType>,
                       VertexType, WeightType> {
 public:
  using ErrorHandler = std::function<void(GraphErrorCode, const std::string &)>;

  UndirectedAdjacencyMatrix(
      std::size_t maxVertices, GraphOptions options = GraphOptions(),
      ErrorPolicy errorPolicy = ErrorPolicy::SILENT_IGNORE)
      : GraphBase<UndirectedAdjacencyMatrix, VertexType, WeightType>(options),
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

  ~UndirectedAdjacencyMatrix() override {
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
    return result / 2;
  }

  UndirectedAdjacencyMatrix &addVertex(const VertexType &vertex) override {
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

  UndirectedAdjacencyMatrix &removeVertex(const VertexType &vertex) override {
    auto it = indices_.find(vertex);
    if (it == indices_.end()) {
      handleError(GraphErrorCode::VERTEX_NOT_FOUND);
      return *this;
    }
    std::size_t index = it->second;
    for (std::size_t i = 0; i < capacity_; ++i) {
      edgeExists_[index * capacity_ + i] = false;
      edgeExists_[i * capacity_ + index] = false;
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

  UndirectedAdjacencyMatrix &addEdge(const VertexType &v1, const VertexType &v2,
                                     const WeightType &weight) override {
    auto it1 = indices_.find(v1);
    auto it2 = indices_.find(v2);
    if (it1 == indices_.end() || it2 == indices_.end()) {
      handleError(GraphErrorCode::VERTEX_NOT_FOUND);
      return *this;
    }
    if (v1 == v2 && !this->options_.allowSelfLoops) {
      handleError(GraphErrorCode::SELF_LOOPS_FORBIDDEN);
      return *this;
    }
    std::size_t pos1 = it1->second * capacity_ + it2->second;
    std::size_t pos2 = it2->second * capacity_ + it1->second;
    if (this->options_.checkDuplicateEdges && edgeExists_[pos1]) {
      handleError(GraphErrorCode::EDGE_ALREADY_EXISTS);
      return *this;
    }
    matrix_[pos1] = weight;
    matrix_[pos2] = weight;
    edgeExists_[pos1] = true;
    edgeExists_[pos2] = true;
    return *this;
  }

  UndirectedAdjacencyMatrix &removeEdge(const VertexType &v1,
                                        const VertexType &v2) override {
    auto it1 = indices_.find(v1);
    auto it2 = indices_.find(v2);
    if (it1 == indices_.end() || it2 == indices_.end()) {
      handleError(GraphErrorCode::VERTEX_NOT_FOUND);
      return *this;
    }
    std::size_t pos1 = it1->second * capacity_ + it2->second;
    std::size_t pos2 = it2->second * capacity_ + it1->second;
    if (!edgeExists_[pos1]) {
      handleError(GraphErrorCode::EDGE_NOT_FOUND);
      return *this;
    }
    edgeExists_[pos1] = false;
    edgeExists_[pos2] = false;
    return *this;
  }

  bool hasEdge(const VertexType &v1, const VertexType &v2) const override {
    auto it1 = indices_.find(v1);
    auto it2 = indices_.find(v2);
    if (it1 == indices_.end() || it2 == indices_.end()) return false;
    std::size_t pos = it1->second * capacity_ + it2->second;
    return edgeExists_[pos];
  }

  std::vector<VertexType> getNeighbors(
      const VertexType &vertex) const override {
    auto it = indices_.find(vertex);
    if (it == indices_.end()) throw vertexNotFound();
    std::vector<VertexType> neighbors;
    std::size_t row = it->second;
    for (std::size_t col = 0; col < capacity_; ++col) {
      if (edgeExists_[row * capacity_ + col]) {
        for (const auto &pair : indices_) {
          if (pair.second == col) {
            neighbors.push_back(pair.first);
            break;
          }
        }
      }
    }
    return neighbors;
  }

  WeightType getEdgeWeight(const VertexType &v1,
                           const VertexType &v2) const override {
    auto it1 = indices_.find(v1);
    auto it2 = indices_.find(v2);
    if (it1 == indices_.end() || it2 == indices_.end()) throw vertexNotFound();
    std::size_t pos = it1->second * capacity_ + it2->second;
    if (!edgeExists_[pos]) throw edgeNotFound();
    return matrix_[pos];
  }

  UndirectedAdjacencyMatrix &setErrorHandler(ErrorHandler handler) {
    errorHandler_ = handler;
    return *this;
  }

  UndirectedAdjacencyMatrix &setErrorPolicy(ErrorPolicy policy) {
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
