#pragma once

#include <functional>
#include <stack>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace graph {
    namespace algorithms {

        template<typename Graph>
        class DepthFirstSearch {
        public:
            using vertex_type = std::remove_reference_t<decltype(*std::declval<Graph>().getVertices().begin())>;

            explicit DepthFirstSearch(const Graph &graph) : graph_(graph) {}

            void run(vertex_type start) {
                reset();
                processVertex(start);
            }

            void runOnAll() {
                reset();
                for (auto vertex: graph_.getVertices()) {
                    if (!visited_[vertex]) {
                        processVertex(vertex);
                    }
                }
            }

            const std::vector<vertex_type> &getVisitOrder() const { return visit_order_; }
            const std::unordered_map<vertex_type, bool> &getVisited() const { return visited_; }

            void setVertexHandler(std::function<void(vertex_type)> handler) {
                vertex_handler_ = std::move(handler);
            }

        private:
            const Graph &graph_;
            std::unordered_map<vertex_type, bool> visited_;
            std::vector<vertex_type> visit_order_;
            std::function<void(vertex_type)> vertex_handler_ = [](vertex_type) {};

            void reset() {
                visited_.clear();
                visit_order_.clear();
            }

            void processVertex(vertex_type start) {
                std::stack<vertex_type> stack;
                stack.push(start);

                while (!stack.empty()) {
                    vertex_type current = stack.top();
                    stack.pop();

                    if (visited_[current]) continue;

                    visited_[current] = true;
                    visit_order_.push_back(current);
                    vertex_handler_(current);

                    for (auto neighbor: graph_.getNeighbors(current)) {
                        if (!visited_[neighbor]) {
                            stack.push(neighbor);
                        }
                    }
                }
            }
        };

    }// namespace algorithms
}// namespace graph
