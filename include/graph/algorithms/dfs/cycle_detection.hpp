#pragma once

#include "depth_first_search.hpp"

#include <algorithm>
#include <unordered_map>
#include <vector>

namespace graph {
    namespace algorithms {

        template<typename Graph>
        class CycleDetection {
        public:
            using vertex_type = typename DepthFirstSearch<Graph>::vertex_type;

            explicit CycleDetection(const Graph &graph) : graph_(graph) {}

            bool hasCycle() {
                reset();

                for (const auto &vertex: graph_.getVertices()) {
                    if (!visited_[vertex]) {
                        if (detectCycle(vertex, vertex)) {
                            return true;
                        }
                    }
                }

                return false;
            }

            std::vector<vertex_type> getCycle() {
                if (cycle_.empty()) {
                    hasCycle();
                }
                return cycle_;
            }

        private:
            const Graph &graph_;
            std::unordered_map<vertex_type, bool> visited_;
            std::unordered_map<vertex_type, vertex_type> parent_;
            std::vector<vertex_type> cycle_;

            void reset() {
                visited_.clear();
                parent_.clear();
                cycle_.clear();
            }

            bool detectCycle(vertex_type current, vertex_type parent) {
                visited_[current] = true;
                parent_[current] = parent;

                for (const auto &neighbor: graph_.getNeighbors(current)) {
                    if (!visited_[neighbor]) {
                        if (detectCycle(neighbor, current)) {
                            return true;
                        }
                    } else if (neighbor != parent) {
                        reconstructCycle(current, neighbor);
                        return true;
                    }
                }

                return false;
            }

            void reconstructCycle(vertex_type current, vertex_type cycle_node) {
                cycle_.clear();

                cycle_.push_back(cycle_node);
                cycle_.push_back(current);

                vertex_type vertex = parent_[current];
                while (vertex != cycle_node) {
                    cycle_.push_back(vertex);
                    vertex = parent_[vertex];
                }

                cycle_.push_back(cycle_node);
            }
        };

    }// namespace algorithms
}// namespace graph
