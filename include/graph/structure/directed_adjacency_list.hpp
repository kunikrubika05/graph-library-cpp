#pragma once

#include "error_policy.hpp"
#include "graph_base.hpp"
#include "graph_exceptions.hpp"

#include <functional>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace graph {

    template<typename VertexType, typename WeightType>
    class DirectedAdjacencyList : public GraphBase<DirectedAdjacencyList<VertexType, WeightType>, VertexType, WeightType> {
    public:
        using ErrorHandler = std::function<void(GraphErrorCode, const std::string &)>;

        explicit DirectedAdjacencyList(
                GraphOptions options = GraphOptions(),
                ErrorPolicy errorPolicy = ErrorPolicy::SILENT_IGNORE)
            : GraphBase<DirectedAdjacencyList, VertexType, WeightType>(options), errorPolicy_(errorPolicy)
        {
            if (errorPolicy == ErrorPolicy::SILENT_IGNORE) {
                errorHandler_ = [](GraphErrorCode code, const std::string &msg) {
                    std::cerr << "Ошибка при обработке: " << msg << "\n";
                };
            }
        }

        DirectedAdjacencyList &setErrorHandler(ErrorHandler handler) {
            errorHandler_ = std::move(handler);
            return *this;
        }

        DirectedAdjacencyList &setErrorPolicy(ErrorPolicy policy) {
            errorPolicy_ = policy;
            return *this;
        }

        ~DirectedAdjacencyList() override = default;

        [[nodiscard]] std::size_t vertexCount() const override {
            return adjacency_list_.size();
        }

        [[nodiscard]] std::size_t edgeCount() const override {
            std::size_t count = 0;
            for (const auto &pair : adjacency_list_) {
                count += pair.second.size();
            }
            return count;
        }

        DirectedAdjacencyList &addVertex(const VertexType &vertex) override {
            auto [iter, inserted] = adjacency_list_.try_emplace(vertex);
            if (!inserted) {
                handleError(GraphErrorCode::VERTEX_ALREADY_EXISTS);
            }
            return *this;
        }

        DirectedAdjacencyList &removeVertex(const VertexType &vertex) override {
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

        DirectedAdjacencyList &addEdge(const VertexType &from, const VertexType &to, const WeightType &weight) override {
            auto fromIt = adjacency_list_.find(from);
            auto toIt = adjacency_list_.find(to);
            if (fromIt == adjacency_list_.end() || toIt == adjacency_list_.end()) {
                handleError(GraphErrorCode::VERTEX_NOT_FOUND);
                return *this;
            }
            if (from == to && !this->options_.allowSelfLoops) {
                handleError(GraphErrorCode::SELF_LOOPS_FORBIDDEN);
                return *this;
            }
            if (this->options_.checkDuplicateEdges && fromIt->second.find(to) != fromIt->second.end()) {
                handleError(GraphErrorCode::EDGE_ALREADY_EXISTS);
                return *this;
            }
            fromIt->second[to] = weight;
            return *this;
        }

        DirectedAdjacencyList &removeEdge(const VertexType &from, const VertexType &to) override {
            auto fromIt = adjacency_list_.find(from);
            auto toIt = adjacency_list_.find(to);
            if (fromIt == adjacency_list_.end() || toIt == adjacency_list_.end()) {
                handleError(GraphErrorCode::VERTEX_NOT_FOUND);
                return *this;
            }
            if (fromIt->second.find(to) == fromIt->second.end()) {
                handleError(GraphErrorCode::EDGE_NOT_FOUND);
                return *this;
            }
            fromIt->second.erase(to);
            return *this;
        }

        bool hasEdge(const VertexType &from, const VertexType &to) const override {
            auto fromIt = adjacency_list_.find(from);
            if (fromIt == adjacency_list_.end()) return false;
            return fromIt->second.find(to) != fromIt->second.end();
        }

        std::vector<VertexType> getNeighbors(const VertexType &vertex) const override {
            auto it = adjacency_list_.find(vertex);
            if (it == adjacency_list_.end()) {
                throw vertexNotFound();
            }
            std::vector<VertexType> neighbors;
            const auto &edges = it->second;
            neighbors.reserve(edges.size());
            for (const auto &entry : edges) {
                neighbors.push_back(entry.first);
            }
            return neighbors;
        }

        WeightType getEdgeWeight(const VertexType &from, const VertexType &to) const override {
            auto fromIt = adjacency_list_.find(from);
            if (fromIt == adjacency_list_.end()) {
                throw vertexNotFound();
            }
            auto edgeIt = fromIt->second.find(to);
            if (edgeIt == fromIt->second.end()) {
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
