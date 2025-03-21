#pragma once

#include "graph_options.hpp"
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace graph {

    template<typename Derived, typename VertexType, typename WeightType>
    class GraphBase {
    public:
        explicit GraphBase(GraphOptions options = GraphOptions()) : options_(options) {}
        virtual ~GraphBase() = default;

        [[nodiscard]] virtual std::size_t vertexCount() const = 0;
        [[nodiscard]] virtual std::size_t edgeCount() const = 0;

        virtual Derived &addVertex(const VertexType &vertex) = 0;
        virtual Derived &removeVertex(const VertexType &vertex) = 0;
        virtual bool hasVertex(const VertexType &vertex) const = 0;

        virtual Derived &addEdge(const VertexType &from, const VertexType &to, const WeightType &weight) = 0;
        virtual Derived &removeEdge(const VertexType &from, const VertexType &to) = 0;
        virtual bool hasEdge(const VertexType &from, const VertexType &to) const = 0;

        virtual std::vector<VertexType> getVertices() const = 0;
        virtual std::vector<VertexType> getNeighbors(const VertexType &vertex) const = 0;
        virtual WeightType getEdgeWeight(const VertexType &from, const VertexType &to) const = 0;

    protected:
        GraphOptions options_;
    };

} // namespace graph
