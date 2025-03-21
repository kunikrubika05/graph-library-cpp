#pragma once

#include <functional>
#include <queue>
#include <unordered_map>
#include <vector>

namespace graph {
    namespace algorithms {

        template<typename Graph>
        class BreadthFirstSearch {
        public:
            using vertex_type = std::remove_reference_t<decltype(*std::declval<Graph>().getVertices().begin())>;
            using on_vertex_callback = std::function<void(const vertex_type &)>;
            using on_edge_callback = std::function<void(const vertex_type &, const vertex_type &)>;
            using on_start_callback = std::function<void(const vertex_type &)>;
            using on_finish_callback = std::function<void()>;

            explicit BreadthFirstSearch(const Graph &graph) : graph_(graph) {}

            void traverse(const vertex_type &start_vertex, on_start_callback on_start = [](const vertex_type &) {}, on_vertex_callback on_discover = [](const vertex_type &) {}, on_edge_callback on_edge = [](const vertex_type &, const vertex_type &) {}, on_finish_callback on_finish = []() {}) {
                reset();
                on_start(start_vertex);
                bfs(start_vertex, on_discover, on_edge);
                on_finish();
            }

            void traverseAll(on_start_callback on_start = [](const vertex_type &) {}, on_vertex_callback on_discover = [](const vertex_type &) {}, on_edge_callback on_edge = [](const vertex_type &, const vertex_type &) {}, on_finish_callback on_finish = []() {}) {
                reset();
                for (const auto &vertex: graph_.getVertices()) {
                    if (!visited_[vertex]) {
                        on_start(vertex);
                        bfs(vertex, on_discover, on_edge);
                    }
                }
                on_finish();
            }

            const std::unordered_map<vertex_type, bool> &getVisited() const {
                return visited_;
            }

            const std::unordered_map<vertex_type, int> &getDistance() const {
                return distance_;
            }

            const std::unordered_map<vertex_type, vertex_type> &getParent() const {
                return parent_;
            }

        protected:
            const Graph &graph_;
            std::unordered_map<vertex_type, bool> visited_;
            std::unordered_map<vertex_type, int> distance_;
            std::unordered_map<vertex_type, vertex_type> parent_;

            void reset() {
                visited_.clear();
                distance_.clear();
                parent_.clear();
            }

            void bfs(const vertex_type &start_vertex,
                     on_vertex_callback on_discover,
                     on_edge_callback on_edge) {
                std::queue<vertex_type> queue;

                visited_[start_vertex] = true;
                distance_[start_vertex] = 0;
                on_discover(start_vertex);
                queue.push(start_vertex);

                while (!queue.empty()) {
                    vertex_type current = queue.front();
                    queue.pop();

                    for (const auto &neighbor: graph_.getNeighbors(current)) {
                        on_edge(current, neighbor);

                        if (!visited_[neighbor]) {
                            visited_[neighbor] = true;
                            distance_[neighbor] = distance_[current] + 1;
                            parent_[neighbor] = current;
                            on_discover(neighbor);
                            queue.push(neighbor);
                        }
                    }
                }
            }
        };

    }// namespace algorithms
}// namespace graph
