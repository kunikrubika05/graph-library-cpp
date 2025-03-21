#pragma once

#include "graph/structure/error_policy.hpp"
#include "graph/structure/graph_base.hpp"
#include "graph/structure/graph_exceptions.hpp"

#include <functional>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace graph {

    template<typename VertexType, typename WeightType>
    class UndirectedAdjacencyList : public GraphBase<UndirectedAdjacencyList<VertexType, WeightType>, VertexType, WeightType> {
    public:
        using ErrorHandler = std::function<void(GraphErrorCode, const std::string &)>;

        explicit UndirectedAdjacencyList(
                GraphOptions options = GraphOptions(),
                ErrorPolicy errorPolicy = ErrorPolicy::SILENT_IGNORE)
            : GraphBase<UndirectedAdjacencyList, VertexType, WeightType>(options), errorPolicy_(errorPolicy)
        {
            if (errorPolicy == ErrorPolicy::SILENT_IGNORE) {
                errorHandler_ = [](GraphErrorCode code, const std::string &msg) {
                    std::cerr << "Ошибка при обработке: " << msg << "\n";
                };
            }
        }

        UndirectedAdjacencyList &setErrorHandler(ErrorHandler handler) {
            errorHandler_ = std::move(handler);
            return *this;
        }

        UndirectedAdjacencyList &setErrorPolicy(ErrorPolicy policy) {
            errorPolicy_ = policy;
            return *this;
        }

        ~UndirectedAdjacencyList() override = default;

        [[nodiscard]] std::size_t vertexCount() const override {
            return adjacency_list_.size();
        }

        [[nodiscard]] std::size_t edgeCount() const override {
            std::size_t count = 0;
            for (const auto &pair : adjacency_list_) {
                count += pair.second.size();
            }
            return count / 2;
        }

        UndirectedAdjacencyList &addVertex(const VertexType &vertex) override {
            auto [iter, inserted] = adjacency_list_.try_emplace(vertex);
            if (!inserted) {
                handleError(GraphErrorCode::VERTEX_ALREADY_EXISTS);
            }
            return *this;
        }

        UndirectedAdjacencyList &removeVertex(const VertexType &vertex) override {
            if (adjacency_list_.find(vertex) == adjacency_list_.end()) {
                handleError(GraphErrorCode::VERTEX_NOT_FOUND);
                return *this;
            }
            for (auto &pair : adjacency_list_) {
                pair.second.erase(vertex);
            }
            adjacency_list_.erase(vertex);
            return *this;
        }

        bool hasVertex(const VertexType &vertex) const override {
            return adjacency_list_.count(vertex) != 0;
        }

        std::vector<VertexType> getVertices() const override {
            std::vector<VertexType> vertices;
            vertices.reserve(adjacency_list_.size());
            for (const auto &pair : adjacency_list_) {
                vertices.push_back(pair.first);
            }
            return vertices;
        }

        UndirectedAdjacencyList &addEdge(const VertexType &v1, const VertexType &v2, const WeightType &weight) override {
            auto node1It = adjacency_list_.find(v1);
            auto node2It = adjacency_list_.find(v2);
            if (node1It == adjacency_list_.end() || node2It == adjacency_list_.end()) {
                handleError(GraphErrorCode::VERTEX_NOT_FOUND);
                return *this;
            }
            if (v1 == v2 && !this->options_.allowSelfLoops) {
                handleError(GraphErrorCode::SELF_LOOPS_FORBIDDEN);
                return *this;
            }
            if (this->options_.checkDuplicateEdges && node1It->second.find(v2) != node1It->second.end()) {
                handleError(GraphErrorCode::EDGE_ALREADY_EXISTS);
                return *this;
            }
            node1It->second[v2] = weight;
            node2It->second[v1] = weight;
            return *this;
        }

        UndirectedAdjacencyList &removeEdge(const VertexType &v1, const VertexType &v2) override {
            auto node1It = adjacency_list_.find(v1);
            auto node2It = adjacency_list_.find(v2);
            if (node1It == adjacency_list_.end() || node2It == adjacency_list_.end()) {
                handleError(GraphErrorCode::VERTEX_NOT_FOUND);
                return *this;
            }
            if (node1It->second.find(v2) == node1It->second.end()) {
                handleError(GraphErrorCode::EDGE_NOT_FOUND);
                return *this;
            }
            node1It->second.erase(v2);
            node2It->second.erase(v1);
            return *this;
        }

        bool hasEdge(const VertexType &v1, const VertexType &v2) const override {
            auto node1It = adjacency_list_.find(v1);
            if (node1It == adjacency_list_.end()) return false;
            return node1It->second.find(v2) != node1It->second.end();
        }

        std::vector<VertexType> getNeighbors(const VertexType &vertex) const override {
            auto it = adjacency_list_.find(vertex);
            if (it == adjacency_list_.end()) {
                throw vertexNotFound();
            }
            std::vector<VertexType> neighbors;
            neighbors.reserve(it->second.size());
            for (const auto &entry : it->second) {
                neighbors.push_back(entry.first);
            }
            return neighbors;
        }

        WeightType getEdgeWeight(const VertexType &v1, const VertexType &v2) const override {
            auto node1It = adjacency_list_.find(v1);
            if (node1It == adjacency_list_.end()) {
                throw vertexNotFound();
            }
            auto edgeIt = node1It->second.find(v2);
            if (edgeIt == node1It->second.end()) {
                throw edgeNotFound();
            }
            return edgeIt->second;
        }

    private:
        std::unordered_map<VertexType, std::unordered_map<VertexType, WeightType>> adjacency_list_;
        ErrorPolicy errorPolicy_ = ErrorPolicy::THROW_EXCEPTIONS;
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

} // namespace graph
