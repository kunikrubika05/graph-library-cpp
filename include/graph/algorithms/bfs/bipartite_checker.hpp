#pragma once

#include <queue>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace graph {
    namespace algorithms {

        template<typename Graph>
        class BipartiteChecker {
        public:
            using vertex_type = std::remove_reference_t<decltype(*std::declval<Graph>().getVertices().begin())>;
            bool isBipartite(const Graph &graph) {
                std::unordered_map<vertex_type, int> colors;
                for (const auto &vertex: graph.getVertices()) {
                    colors[vertex] = 0;
                }
                for (const auto &vertex: graph.getVertices()) {
                    if (colors[vertex] == 0) {
                        if (!bfsCheck(graph, vertex, colors)) {
                            return false;
                        }
                    }
                }
                return true;
            }

        private:
            bool bfsCheck(const Graph &graph, const vertex_type &start, std::unordered_map<vertex_type, int> &colors) {
                std::queue<vertex_type> q;
                q.push(start);
                colors[start] = 1;
                while (!q.empty()) {
                    vertex_type current = q.front();
                    q.pop();
                    for (auto neighbor: graph.getNeighbors(current)) {
                        if (colors[neighbor] == 0) {
                            colors[neighbor] = -colors[current];
                            q.push(neighbor);
                        } else if (colors[neighbor] == colors[current]) {
                            return false;
                        }
                    }
                }
                return true;
            }
        };

    }// namespace algorithms
}// namespace graph
