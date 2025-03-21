#pragma once

#include "depth_first_search.hpp"

#include <algorithm>
#include <vector>

namespace graph {
    namespace algorithms {

        template<typename Graph>
        class ConnectedComponents {
        public:
            using vertex_type = typename DepthFirstSearch<Graph>::vertex_type;

            explicit ConnectedComponents(const Graph &graph) : graph_(graph) {}

            std::vector<std::vector<vertex_type>> findAll() {
                std::vector<std::vector<vertex_type>> components;
                std::unordered_map<vertex_type, bool> visited;

                for (auto vertex: graph_.getVertices()) {
                    if (!visited[vertex]) {
                        std::vector<vertex_type> component = findOne(vertex);
                        for (auto v: component) {
                            visited[v] = true;
                        }
                        components.push_back(component);
                    }
                }

                return components;
            }

            std::vector<vertex_type> findOne(vertex_type start) {
                DepthFirstSearch<Graph> dfs(graph_);
                dfs.run(start);

                std::vector<vertex_type> component = dfs.getVisitOrder();
                std::sort(component.begin(), component.end());
                return component;
            }

        private:
            const Graph &graph_;
        };

    }// namespace algorithms
}// namespace graph
