#include <algorithm>
#include <functional>
#include <iostream>
#include <unordered_set>
#include <vector>

#include "graph/structure/error_policy.hpp"
#include "graph/structure/graph_base.hpp"
#include "graph/structure/graph_exceptions.hpp"
#include "graph/structure/graph_options.hpp"

namespace graph {

template <typename VertexType, typename WeightType>
class UndirectedEdgeList
    : public GraphBase<UndirectedEdgeList<VertexType, WeightType>, VertexType,
                       WeightType> {
 public:
  using ErrorHandler = std::function<void(GraphErrorCode, const std::string &)>;

  struct Edge {
    VertexType v1;
    VertexType v2;
    WeightType weight;
  };

  UndirectedEdgeList(GraphOptions options = GraphOptions(),
                     ErrorPolicy errorPolicy = ErrorPolicy::SILENT_IGNORE)
      : GraphBase<UndirectedEdgeList, VertexType, WeightType>(options),
        errorPolicy_(errorPolicy) {
    if (errorPolicy_ == ErrorPolicy::SILENT_IGNORE) {
      errorHandler_ = [](GraphErrorCode code, const std::string &msg) {
        std::cerr << "Ошибка: " << msg << "\n";
      };
    }
  }

  ~UndirectedEdgeList() override = default;

  [[nodiscard]] std::size_t vertexCount() const override {
    return vertices_.size();
  }

  [[nodiscard]] std::size_t edgeCount() const override { return edges_.size(); }

  UndirectedEdgeList &addVertex(const VertexType &vertex) override {
    if (vertices_.find(vertex) != vertices_.end()) {
      handleError(GraphErrorCode::VERTEX_ALREADY_EXISTS);
      return *this;
    }
    vertices_.insert(vertex);
    return *this;
  }

  UndirectedEdgeList &removeVertex(const VertexType &vertex) override {
    if (vertices_.find(vertex) == vertices_.end()) {
      handleError(GraphErrorCode::VERTEX_NOT_FOUND);
      return *this;
    }
    vertices_.erase(vertex);
    edges_.erase(std::remove_if(edges_.begin(), edges_.end(),
                                [&vertex](const Edge &e) {
                                  return e.v1 == vertex || e.v2 == vertex;
                                }),
                 edges_.end());
    return *this;
  }

  bool hasVertex(const VertexType &vertex) const override {
    return vertices_.find(vertex) != vertices_.end();
  }

  std::vector<VertexType> getVertices() const override {
    return std::vector<VertexType>(vertices_.begin(), vertices_.end());
  }

  UndirectedEdgeList &addEdge(const VertexType &v1, const VertexType &v2,
                              const WeightType &weight) override {
    if (vertices_.find(v1) == vertices_.end() ||
        vertices_.find(v2) == vertices_.end()) {
      handleError(GraphErrorCode::VERTEX_NOT_FOUND);
      return *this;
    }
    if (v1 == v2 && !this->options_.allowSelfLoops) {
      handleError(GraphErrorCode::SELF_LOOPS_FORBIDDEN);
      return *this;
    }
    if (this->options_.checkDuplicateEdges) {
      auto it =
          std::find_if(edges_.begin(), edges_.end(), [&v1, &v2](const Edge &e) {
            return (e.v1 == v1 && e.v2 == v2) || (e.v1 == v2 && e.v2 == v1);
          });
      if (it != edges_.end()) {
        handleError(GraphErrorCode::EDGE_ALREADY_EXISTS);
        return *this;
      }
    }
    edges_.push_back({v1, v2, weight});
    return *this;
  }

  UndirectedEdgeList &removeEdge(const VertexType &v1,
                                 const VertexType &v2) override {
    auto it =
        std::find_if(edges_.begin(), edges_.end(), [&v1, &v2](const Edge &e) {
          return (e.v1 == v1 && e.v2 == v2) || (e.v1 == v2 && e.v2 == v1);
        });
    if (it == edges_.end()) {
      handleError(GraphErrorCode::EDGE_NOT_FOUND);
      return *this;
    }
    edges_.erase(it);
    return *this;
  }

  bool hasEdge(const VertexType &v1, const VertexType &v2) const override {
    auto it =
        std::find_if(edges_.begin(), edges_.end(), [&v1, &v2](const Edge &e) {
          return (e.v1 == v1 && e.v2 == v2) || (e.v1 == v2 && e.v2 == v1);
        });
    return it != edges_.end();
  }

  std::vector<VertexType> getNeighbors(
      const VertexType &vertex) const override {
    if (vertices_.find(vertex) == vertices_.end()) {
      throw vertexNotFound();
    }
    std::vector<VertexType> neighbors;
    for (const auto &edge : edges_) {
      if (edge.v1 == vertex)
        neighbors.push_back(edge.v2);
      else if (edge.v2 == vertex)
        neighbors.push_back(edge.v1);
    }
    return neighbors;
  }

  WeightType getEdgeWeight(const VertexType &v1,
                           const VertexType &v2) const override {
    auto it =
        std::find_if(edges_.begin(), edges_.end(), [&v1, &v2](const Edge &e) {
          return (e.v1 == v1 && e.v2 == v2) || (e.v1 == v2 && e.v2 == v1);
        });
    if (it == edges_.end()) {
      throw edgeNotFound();
    }
    return it->weight;
  }

  UndirectedEdgeList &setErrorHandler(ErrorHandler handler) {
    errorHandler_ = handler;
    return *this;
  }

  UndirectedEdgeList &setErrorPolicy(ErrorPolicy policy) {
    errorPolicy_ = policy;
    return *this;
  }

 private:
  std::vector<Edge> edges_;
  std::unordered_set<VertexType> vertices_;
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
        default:
          break;
      }
    }
  }
};

}  // namespace graph
