#pragma once

#include "depth_first_search.hpp"

#include <algorithm>
#include <unordered_map>
#include <vector>

namespace graph {
    namespace algorithms {

        template<typename Graph>
        class ArticulationPoints {
        public:
            using vertex_type = typename DepthFirstSearch<Graph>::vertex_type;

            explicit ArticulationPoints(const Graph &graph) : graph_(graph) {}

            void findArticulationPoints() {
                reset();
                for (const auto &vertex: graph_.getVertices()) {
                    if (!visited_[vertex]) {
                        parent_[vertex] = -1;
                        dfs(vertex);
                    }
                }
            }

            const std::vector<vertex_type> &getArticulationPoints() const {
                return articulation_points_vec_;
            }

        private:
            const Graph &graph_;
            std::unordered_map<vertex_type, bool> visited_;
            std::unordered_map<vertex_type, int> discovery_time_;
            std::unordered_map<vertex_type, int> low_;
            std::unordered_map<vertex_type, int> parent_;
            std::unordered_map<vertex_type, bool> articulation_;
            std::vector<vertex_type> articulation_points_vec_;
            int time_ = 0;

            void reset() {
                visited_.clear();
                discovery_time_.clear();
                low_.clear();
                parent_.clear();
                articulation_.clear();
                articulation_points_vec_.clear();
                time_ = 0;
            }

            void dfs(const vertex_type &u) {
                visited_[u] = true;
                discovery_time_[u] = low_[u] = ++time_;
                int children = 0;

                for (auto v: graph_.getNeighbors(u)) {
                    if (!visited_[v]) {
                        children++;
                        parent_[v] = u;
                        dfs(v);
                        low_[u] = std::min(low_[u], low_[v]);

                        if (parent_[u] != -1 && low_[v] >= discovery_time_[u]) {
                            articulation_[u] = true;
                        }
                    } else if (v != parent_[u]) {
                        low_[u] = std::min(low_[u], discovery_time_[v]);
                    }
                }

                if (parent_[u] == -1 && children > 1) {
                    articulation_[u] = true;
                }

                if (articulation_[u]) {
                    articulation_points_vec_.push_back(u);
                }
            }
        };

    }// namespace algorithms
}// namespace graph
