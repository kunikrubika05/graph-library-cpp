#pragma once

#include <unordered_map>

template<typename Graph>
class TransitiveClosure {
public:
    using vertex_type = typename std::remove_reference_t<decltype(*std::declval<Graph>().getVertices().begin())>;

    explicit TransitiveClosure(const Graph &graph) : graph_(graph) {}

    void compute() {
        const auto vertices = graph_.getVertices();

        for (const auto &i: vertices) {
            for (const auto &j: vertices) {
                reachable_[i][j] = (i == j) || graph_.hasEdge(i, j);
            }
        }

        for (const auto &k: vertices) {
            for (const auto &i: vertices) {
                for (const auto &j: vertices) {
                    reachable_[i][j] = reachable_[i][j] || (reachable_[i][k] && reachable_[k][j]);
                }
            }
        }
    }

    bool isReachable(const vertex_type &from, const vertex_type &to) const {
        auto it_from = reachable_.find(from);
        if (it_from == reachable_.end()) return false;

        auto it_to = it_from->second.find(to);
        if (it_to == it_from->second.end()) return false;

        return it_to->second;
    }

    const std::unordered_map<vertex_type, std::unordered_map<vertex_type, bool>> &getReachabilityMatrix() const {
        return reachable_;
    }

private:
    const Graph &graph_;
    std::unordered_map<vertex_type, std::unordered_map<vertex_type, bool>> reachable_;
};
