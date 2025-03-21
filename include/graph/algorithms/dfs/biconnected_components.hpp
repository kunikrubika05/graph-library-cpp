#pragma once

#include "depth_first_search.hpp"

#include <stack>
#include <unordered_map>
#include <vector>

namespace graph {
    namespace algorithms {

        template<typename Graph>
        class BiconnectedComponents {
        public:
            using vertex_type = typename DepthFirstSearch<Graph>::vertex_type;
            using edge_type = std::pair<vertex_type, vertex_type>;
            using component_type = std::vector<edge_type>;

            explicit BiconnectedComponents(const Graph &graph) : graph_(graph) {}

            void findComponents() {
                reset();

                for (const auto &vertex: graph_.getVertices()) {
                    if (!visited_[vertex]) {
                        dfs(vertex, vertex);
                    }
                }
            }

            const std::vector<component_type> &getComponents() const {
                return components_;
            }

        private:
            const Graph &graph_;
            std::unordered_map<vertex_type, bool> visited_;
            std::unordered_map<vertex_type, int> discovery_time_;
            std::unordered_map<vertex_type, int> low_link_;
            std::stack<edge_type> edge_stack_;
            std::vector<component_type> components_;
            int time_ = 0;

            void reset() {
                visited_.clear();
                discovery_time_.clear();
                low_link_.clear();
                while (!edge_stack_.empty()) {
                    edge_stack_.pop();
                }
                components_.clear();
                time_ = 0;
            }

            void dfs(vertex_type current, vertex_type parent) {
                visited_[current] = true;
                discovery_time_[current] = low_link_[current] = ++time_;

                int children = 0;

                for (const auto &neighbor: graph_.getNeighbors(current)) {
                    if (!visited_[neighbor]) {
                        children++;
                        edge_stack_.push({current, neighbor});
                        dfs(neighbor, current);

                        low_link_[current] = std::min(low_link_[current], low_link_[neighbor]);

                        if ((parent == current && children > 1) ||
                            (parent != current && low_link_[neighbor] >= discovery_time_[current])) {
                            component_type component;

                            while (true) {
                                edge_type e = edge_stack_.top();
                                edge_stack_.pop();
                                component.push_back(e);

                                if (e.first == current && e.second == neighbor) {
                                    break;
                                }
                            }

                            components_.push_back(component);
                        }
                    } else if (neighbor != parent && discovery_time_[neighbor] < discovery_time_[current]) {
                        low_link_[current] = std::min(low_link_[current], discovery_time_[neighbor]);
                        edge_stack_.push({current, neighbor});
                    }
                }

                if (parent == current && edge_stack_.size() > 0) {
                    component_type component;
                    while (!edge_stack_.empty()) {
                        component.push_back(edge_stack_.top());
                        edge_stack_.pop();
                    }
                    components_.push_back(component);
                }
            }
        };

    }// namespace algorithms
}// namespace graph
