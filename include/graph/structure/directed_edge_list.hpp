#pragma once

#include "error_policy.hpp"
#include "graph_base.hpp"
#include "graph_exceptions.hpp"
#include "graph_options.hpp"

#include <algorithm>
#include <functional>
#include <iostream>
#include <unordered_set>
#include <vector>

namespace graph {

    template<typename VertexType, typename WeightType>
    class DirectedEdgeList : public GraphBase<DirectedEdgeList<VertexType, WeightType>, VertexType, WeightType> {
    public:
        using ErrorHandler = std::function<void(GraphErrorCode, const std::string &)>;

        struct Edge {
            VertexType from;
            VertexType to;
            WeightType weight;
        };

        DirectedEdgeList(GraphOptions options = GraphOptions(), ErrorPolicy errorPolicy = ErrorPolicy::SILENT_IGNORE)
            : GraphBase<DirectedEdgeList, VertexType, WeightType>(options),
              errorPolicy_(errorPolicy) {
            if (errorPolicy_ == ErrorPolicy::SILENT_IGNORE) {
                errorHandler_ = [](GraphErrorCode code, const std::string &msg) {
                    std::cerr << "Ошибка: " << msg << "\n";
                };
            }
        }

        ~DirectedEdgeList() override = default;

        [[nodiscard]] std::size_t vertexCount() const override {
            return vertices_.size();
        }

        [[nodiscard]] std::size_t edgeCount() const override {
            return edges_.size();
        }

        DirectedEdgeList &addVertex(const VertexType &vertex) override {
            if (vertices_.find(vertex) != vertices_.end()) {
                handleError(GraphErrorCode::VERTEX_ALREADY_EXISTS);
                return *this;
            }
            vertices_.insert(vertex);
            return *this;
        }

        DirectedEdgeList &removeVertex(const VertexType &vertex) override {
            if (vertices_.find(vertex) == vertices_.end()) {
                handleError(GraphErrorCode::VERTEX_NOT_FOUND);
                return *this;
            }
            vertices_.erase(vertex);
            edges_.erase(std::remove_if(edges_.begin(), edges_.end(),
                                        [&vertex](const Edge &e) {
                                            return e.from == vertex || e.to == vertex;
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

        DirectedEdgeList &addEdge(const VertexType &from, const VertexType &to, const WeightType &weight) override {
            if (vertices_.find(from) == vertices_.end() || vertices_.find(to) == vertices_.end()) {
                handleError(GraphErrorCode::VERTEX_NOT_FOUND);
                return *this;
            }
            if (from == to && !this->options_.allowSelfLoops) {
                handleError(GraphErrorCode::SELF_LOOPS_FORBIDDEN);
                return *this;
            }
            if (this->options_.checkDuplicateEdges) {
                auto it = std::find_if(edges_.begin(), edges_.end(),
                                       [&from, &to](const Edge &e) {
                                           return e.from == from && e.to == to;
                                       });
                if (it != edges_.end()) {
                    handleError(GraphErrorCode::EDGE_ALREADY_EXISTS);
                    return *this;
                }
            }
            edges_.push_back({from, to, weight});
            return *this;
        }

        DirectedEdgeList &removeEdge(const VertexType &from, const VertexType &to) override {
            auto it = std::find_if(edges_.begin(), edges_.end(),
                                   [&from, &to](const Edge &e) {
                                       return e.from == from && e.to == to;
                                   });
            if (it == edges_.end()) {
                handleError(GraphErrorCode::EDGE_NOT_FOUND);
                return *this;
            }
            edges_.erase(it);
            return *this;
        }

        bool hasEdge(const VertexType &from, const VertexType &to) const override {
            auto it = std::find_if(edges_.begin(), edges_.end(),
                                   [&from, &to](const Edge &e) {
                                       return e.from == from && e.to == to;
                                   });
            return it != edges_.end();
        }

        std::vector<VertexType> getNeighbors(const VertexType &vertex) const override {
            if (vertices_.find(vertex) == vertices_.end()) {
                throw vertexNotFound();
            }
            std::vector<VertexType> neighbors;
            for (const auto &edge: edges_) {
                if (edge.from == vertex) {
                    neighbors.push_back(edge.to);
                }
            }
            return neighbors;
        }

        WeightType getEdgeWeight(const VertexType &from, const VertexType &to) const override {
            auto it = std::find_if(edges_.begin(), edges_.end(),
                                   [&from, &to](const Edge &e) {
                                       return e.from == from && e.to == to;
                                   });
            if (it == edges_.end()) {
                throw edgeNotFound();
            }
            return it->weight;
        }

        DirectedEdgeList &setErrorHandler(ErrorHandler handler) {
            errorHandler_ = handler;
            return *this;
        }

        DirectedEdgeList &setErrorPolicy(ErrorPolicy policy) {
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

}// namespace graph
