#pragma once

#include "graph/structure/directed_adjacency_list.hpp"
#include "graph/structure/directed_adjacency_matrix.hpp"
#include "graph/structure/directed_edge_list.hpp"
#include "undirected_adjacency_list.hpp"
#include "undirected_adjacency_matrix.hpp"
#include "undirected_edge_list.hpp"

namespace graph {

    template<typename VertexType, typename WeightType>
    class GraphConverter {
    public:
        static DirectedAdjacencyList<VertexType, WeightType> toDirectedAdjacencyList(
                const DirectedEdgeList<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            DirectedAdjacencyList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyDirectedEdges(source, target);
            return target;
        }

        static DirectedAdjacencyList<VertexType, WeightType> toDirectedAdjacencyList(
                const DirectedAdjacencyMatrix<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            DirectedAdjacencyList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyDirectedEdges(source, target);
            return target;
        }

        static DirectedAdjacencyList<VertexType, WeightType> toDirectedAdjacencyList(
                const UndirectedEdgeList<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            DirectedAdjacencyList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyUndirectedToDirected(source, target);
            return target;
        }

        static DirectedAdjacencyList<VertexType, WeightType> toDirectedAdjacencyList(
                const UndirectedAdjacencyList<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            DirectedAdjacencyList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyUndirectedToDirected(source, target);
            return target;
        }

        static DirectedAdjacencyList<VertexType, WeightType> toDirectedAdjacencyList(
                const UndirectedAdjacencyMatrix<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            DirectedAdjacencyList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyUndirectedToDirected(source, target);
            return target;
        }

        static DirectedEdgeList<VertexType, WeightType> toDirectedEdgeList(
                const DirectedAdjacencyList<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            DirectedEdgeList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyDirectedEdges(source, target);
            return target;
        }

        static DirectedEdgeList<VertexType, WeightType> toDirectedEdgeList(
                const DirectedAdjacencyMatrix<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            DirectedEdgeList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyDirectedEdges(source, target);
            return target;
        }

        static DirectedEdgeList<VertexType, WeightType> toDirectedEdgeList(
                const UndirectedEdgeList<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            DirectedEdgeList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyUndirectedToDirected(source, target);
            return target;
        }

        static DirectedEdgeList<VertexType, WeightType> toDirectedEdgeList(
                const UndirectedAdjacencyList<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            DirectedEdgeList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyUndirectedToDirected(source, target);
            return target;
        }

        static DirectedEdgeList<VertexType, WeightType> toDirectedEdgeList(
                const UndirectedAdjacencyMatrix<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            DirectedEdgeList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyUndirectedToDirected(source, target);
            return target;
        }

        static UndirectedAdjacencyList<VertexType, WeightType> toUndirectedAdjacencyList(
                const UndirectedEdgeList<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            UndirectedAdjacencyList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyUndirectedEdges(source, target);
            return target;
        }

        static UndirectedAdjacencyList<VertexType, WeightType> toUndirectedAdjacencyList(
                const UndirectedAdjacencyMatrix<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            UndirectedAdjacencyList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyUndirectedEdges(source, target);
            return target;
        }

        static UndirectedAdjacencyList<VertexType, WeightType> toUndirectedAdjacencyList(
                const DirectedEdgeList<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            UndirectedAdjacencyList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyDirectedToUndirected(source, target);
            return target;
        }

        static UndirectedAdjacencyList<VertexType, WeightType> toUndirectedAdjacencyList(
                const DirectedAdjacencyList<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            UndirectedAdjacencyList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyDirectedToUndirected(source, target);
            return target;
        }

        static UndirectedAdjacencyList<VertexType, WeightType> toUndirectedAdjacencyList(
                const DirectedAdjacencyMatrix<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            UndirectedAdjacencyList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyDirectedToUndirected(source, target);
            return target;
        }

        static UndirectedEdgeList<VertexType, WeightType> toUndirectedEdgeList(
                const UndirectedAdjacencyList<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            UndirectedEdgeList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyUndirectedEdges(source, target);
            return target;
        }

        static UndirectedEdgeList<VertexType, WeightType> toUndirectedEdgeList(
                const UndirectedAdjacencyMatrix<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            UndirectedEdgeList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyUndirectedEdges(source, target);
            return target;
        }

        static UndirectedEdgeList<VertexType, WeightType> toUndirectedEdgeList(
                const DirectedEdgeList<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            UndirectedEdgeList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyDirectedToUndirected(source, target);
            return target;
        }

        static UndirectedEdgeList<VertexType, WeightType> toUndirectedEdgeList(
                const DirectedAdjacencyList<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            UndirectedEdgeList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyDirectedToUndirected(source, target);
            return target;
        }

        static UndirectedEdgeList<VertexType, WeightType> toUndirectedEdgeList(
                const DirectedAdjacencyMatrix<VertexType, WeightType> &source,
                GraphOptions options = GraphOptions()) {
            UndirectedEdgeList<VertexType, WeightType> target(options);
            copyVertices(source, target);
            copyDirectedToUndirected(source, target);
            return target;
        }

        static DirectedAdjacencyMatrix<VertexType, WeightType> toDirectedAdjacencyMatrix(
                const DirectedEdgeList<VertexType, WeightType> &source,
                std::size_t maxVertices,
                GraphOptions options = GraphOptions()) {
            DirectedAdjacencyMatrix<VertexType, WeightType> target(maxVertices, options);
            copyVertices(source, target);
            copyDirectedEdges(source, target);
            return target;
        }

        static DirectedAdjacencyMatrix<VertexType, WeightType> toDirectedAdjacencyMatrix(
                const DirectedAdjacencyList<VertexType, WeightType> &source,
                std::size_t maxVertices,
                GraphOptions options = GraphOptions()) {
            DirectedAdjacencyMatrix<VertexType, WeightType> target(maxVertices, options);
            copyVertices(source, target);
            copyDirectedEdges(source, target);
            return target;
        }

        static DirectedAdjacencyMatrix<VertexType, WeightType> toDirectedAdjacencyMatrix(
                const UndirectedEdgeList<VertexType, WeightType> &source,
                std::size_t maxVertices,
                GraphOptions options = GraphOptions()) {
            DirectedAdjacencyMatrix<VertexType, WeightType> target(maxVertices, options);
            copyVertices(source, target);
            copyUndirectedToDirected(source, target);
            return target;
        }

        static DirectedAdjacencyMatrix<VertexType, WeightType> toDirectedAdjacencyMatrix(
                const UndirectedAdjacencyList<VertexType, WeightType> &source,
                std::size_t maxVertices,
                GraphOptions options = GraphOptions()) {
            DirectedAdjacencyMatrix<VertexType, WeightType> target(maxVertices, options);
            copyVertices(source, target);
            copyUndirectedToDirected(source, target);
            return target;
        }

        static DirectedAdjacencyMatrix<VertexType, WeightType> toDirectedAdjacencyMatrix(
                const UndirectedAdjacencyMatrix<VertexType, WeightType> &source,
                std::size_t maxVertices,
                GraphOptions options = GraphOptions()) {
            DirectedAdjacencyMatrix<VertexType, WeightType> target(maxVertices, options);
            copyVertices(source, target);
            copyUndirectedToDirected(source, target);
            return target;
        }

        static UndirectedAdjacencyMatrix<VertexType, WeightType> toUndirectedAdjacencyMatrix(
                const UndirectedEdgeList<VertexType, WeightType> &source,
                std::size_t maxVertices,
                GraphOptions options = GraphOptions()) {
            UndirectedAdjacencyMatrix<VertexType, WeightType> target(maxVertices, options);
            copyVertices(source, target);
            copyUndirectedEdges(source, target);
            return target;
        }

        static UndirectedAdjacencyMatrix<VertexType, WeightType> toUndirectedAdjacencyMatrix(
                const UndirectedAdjacencyList<VertexType, WeightType> &source,
                std::size_t maxVertices,
                GraphOptions options = GraphOptions()) {
            UndirectedAdjacencyMatrix<VertexType, WeightType> target(maxVertices, options);
            copyVertices(source, target);
            copyUndirectedEdges(source, target);
            return target;
        }

        static UndirectedAdjacencyMatrix<VertexType, WeightType> toUndirectedAdjacencyMatrix(
                const DirectedEdgeList<VertexType, WeightType> &source,
                std::size_t maxVertices,
                GraphOptions options = GraphOptions()) {
            UndirectedAdjacencyMatrix<VertexType, WeightType> target(maxVertices, options);
            copyVertices(source, target);
            copyDirectedToUndirected(source, target);
            return target;
        }

        static UndirectedAdjacencyMatrix<VertexType, WeightType> toUndirectedAdjacencyMatrix(
                const DirectedAdjacencyList<VertexType, WeightType> &source,
                std::size_t maxVertices,
                GraphOptions options = GraphOptions()) {
            UndirectedAdjacencyMatrix<VertexType, WeightType> target(maxVertices, options);
            copyVertices(source, target);
            copyDirectedToUndirected(source, target);
            return target;
        }

        static UndirectedAdjacencyMatrix<VertexType, WeightType> toUndirectedAdjacencyMatrix(
                const DirectedAdjacencyMatrix<VertexType, WeightType> &source,
                std::size_t maxVertices,
                GraphOptions options = GraphOptions()) {
            UndirectedAdjacencyMatrix<VertexType, WeightType> target(maxVertices, options);
            copyVertices(source, target);
            copyDirectedToUndirected(source, target);
            return target;
        }

    private:
        template<typename Source, typename Target>
        static void copyVertices(const Source &source, Target &target) {
            auto vertices = source.getVertices();
            for (const auto &v: vertices) {
                target.addVertex(v);
            }
        }


        template<typename Source, typename Target>
        static void copyDirectedEdges(const Source &source, Target &target) {
            auto vertices = source.getVertices();
            for (const auto &from: vertices) {
                auto neighbors = source.getNeighbors(from);
                for (const auto &to: neighbors) {
                    target.addEdge(from, to, source.getEdgeWeight(from, to));
                }
            }
        }

        template<typename Source, typename Target>
        static void copyUndirectedEdges(const Source &source, Target &target) {
            auto vertices = source.getVertices();
            for (size_t i = 0; i < vertices.size(); ++i) {
                const auto &v1 = vertices[i];
                auto neighbors = source.getNeighbors(v1);
                for (const auto &v2: neighbors) {
                    if (v1 <= v2) {
                        target.addEdge(v1, v2, source.getEdgeWeight(v1, v2));
                    }
                }
            }
        }

        template<typename Source, typename Target>
        static void copyUndirectedToDirected(const Source &source, Target &target) {
            auto vertices = source.getVertices();

            for (size_t i = 0; i < vertices.size(); ++i) {
                const auto &v1 = vertices[i];
                auto neighbors = source.getNeighbors(v1);

                for (const auto &v2 : neighbors) {
                    if (v1 < v2 || (source.hasEdge(v1, v2) && !source.hasEdge(v2, v1))) {
                        WeightType weight = source.getEdgeWeight(v1, v2);
                        target.addEdge(v1, v2, weight);
                        target.addEdge(v2, v1, weight);
                    }
                }
            }
        }

        template<typename Source, typename Target>
        static void copyDirectedToUndirected(const Source &source, Target &target) {
            auto vertices = source.getVertices();
            for (const auto &from: vertices) {
                auto neighbors = source.getNeighbors(from);
                for (const auto &to: neighbors) {
                    if (!target.hasEdge(from, to) && !target.hasEdge(to, from)) {
                        target.addEdge(from, to, source.getEdgeWeight(from, to));
                    }
                }
            }
        }
    };

}// namespace graph
