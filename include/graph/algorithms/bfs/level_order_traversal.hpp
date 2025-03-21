#pragma once

#include <algorithm>
#include <functional>
#include <queue>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace graph {
    namespace algorithms {

        template<typename Graph>
        class LevelOrderTraversal {
        public:
            using vertex_type = std::remove_reference_t<decltype(*std::declval<Graph>().getVertices().begin())>;
            using on_vertex_callback = std::function<void(const vertex_type &)>;
            using on_edge_callback = std::function<void(const vertex_type &, const vertex_type &)>;
            using on_start_callback = std::function<void(const vertex_type &)>;
            using on_finish_callback = std::function<void()>;

            explicit LevelOrderTraversal(const Graph &graph) : graph_(graph) {}

            void traverse(const vertex_type &start, on_start_callback on_start = [](const vertex_type &) {}, on_vertex_callback on_discover = [](const vertex_type &) {}, on_edge_callback on_edge = [](const vertex_type &, const vertex_type &) {}, on_finish_callback on_finish = []() {}) {
                reset();
                on_start(start);
                std::queue<vertex_type> q;
                visited_[start] = true;
                distance_[start] = 0;
                order_.push_back(start);
                on_discover(start);
                q.push(start);
                while (!q.empty()) {
                    vertex_type current = q.front();
                    q.pop();
                    for (const auto &neighbor: graph_.getNeighbors(current)) {
                        on_edge(current, neighbor);
                        if (!visited_[neighbor]) {
                            visited_[neighbor] = true;
                            distance_[neighbor] = distance_[current] + 1;
                            parent_[neighbor] = current;
                            order_.push_back(neighbor);
                            on_discover(neighbor);
                            q.push(neighbor);
                        }
                    }
                }
                on_finish();
            }

            std::vector<vertex_type> retrievePath(const vertex_type &target) const {
                if (distance_.find(target) == distance_.end())
                    throw std::runtime_error("Target vertex is not reachable.");
                std::vector<vertex_type> path;
                vertex_type cur = target;
                path.push_back(cur);
                while (parent_.find(cur) != parent_.end()) {
                    cur = parent_.at(cur);
                    path.push_back(cur);
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            const std::vector<vertex_type> &getOrder() const {
                return order_;
            }

            const std::unordered_map<vertex_type, int> &getDistance() const {
                return distance_;
            }

            const std::unordered_map<vertex_type, vertex_type> &getParent() const {
                return parent_;
            }

            const std::unordered_map<vertex_type, bool> &getVisited() const {
                return visited_;
            }

        private:
            const Graph &graph_;
            std::unordered_map<vertex_type, bool> visited_;
            std::unordered_map<vertex_type, int> distance_;
            std::unordered_map<vertex_type, vertex_type> parent_;
            std::vector<vertex_type> order_;

            void reset() {
                visited_.clear();
                distance_.clear();
                parent_.clear();
                order_.clear();
            }
        };

    }// namespace algorithms
}// namespace graph
