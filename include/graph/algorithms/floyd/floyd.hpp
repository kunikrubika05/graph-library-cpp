#pragma once

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

template<typename Graph>
class FloydWarshall {
public:
    using vertex_type = typename std::remove_reference_t<decltype(*std::declval<Graph>().getVertices().begin())>;
    using weight_type = typename std::remove_reference_t<decltype(*std::declval<Graph>().getVertices().begin())>;

    explicit FloydWarshall(const Graph &graph) : graph_(graph) {}

    void compute() {
        reset();
        initializeDistances();

        const auto vertices = graph_.getVertices();

        for (const auto &k: vertices) {
            for (const auto &i: vertices) {
                for (const auto &j: vertices) {
                    if (distance_[i][k] != std::numeric_limits<weight_type>::max() &&
                        distance_[k][j] != std::numeric_limits<weight_type>::max()) {

                        weight_type potential_dist = distance_[i][k] + distance_[k][j];

                        if (potential_dist < distance_[i][j]) {
                            distance_[i][j] = potential_dist;
                            next_[i][j] = next_[i][k];
                        }
                    }
                }
            }
        }

        for (const auto &v: vertices) {
            if (distance_[v][v] < 0) {
                negative_cycle_ = true;
                break;
            }
        }
    }

    weight_type getDistance(const vertex_type &from, const vertex_type &to) const {
        if (negative_cycle_)
            throw std::runtime_error("График содержит отрицательный цикл");

        auto it_from = distance_.find(from);
        if (it_from == distance_.end())
            throw std::runtime_error("Начальная вершина не найдена");

        auto it_to = it_from->second.find(to);
        if (it_to == it_from->second.end())
            throw std::runtime_error("Конечная вершина не найдена");

        if (it_to->second == std::numeric_limits<weight_type>::max())
            throw std::runtime_error("Вершины не связаны");

        return it_to->second;
    }

    std::vector<vertex_type> retrievePath(const vertex_type &from, const vertex_type &to) const {
        if (negative_cycle_)
            throw std::runtime_error("График содержит отрицательный цикл");

        if (from == to)
            return {from};

        if (distance_.find(from) == distance_.end() ||
            distance_.at(from).find(to) == distance_.at(from).end() ||
            distance_.at(from).at(to) == std::numeric_limits<weight_type>::max())
            throw std::runtime_error("Путь не найден");

        std::vector<vertex_type> path;
        vertex_type current = from;
        path.push_back(current);

        while (current != to) {
            current = next_.at(current).at(to);
            path.push_back(current);

            if (path.size() > graph_.getVertices().size())
                throw std::runtime_error("Ошибка реконструкции пути");
        }

        return path;
    }

    const std::unordered_map<vertex_type, std::unordered_map<vertex_type, weight_type>>& getDistances() const {
        return distance_;
    }

    bool hasNegativeCycle() const {
        return negative_cycle_;
    }

private:
    const Graph &graph_;
    std::unordered_map<vertex_type, std::unordered_map<vertex_type, weight_type>> distance_;
    std::unordered_map<vertex_type, std::unordered_map<vertex_type, vertex_type>> next_;
    bool negative_cycle_ = false;

    void reset() {
        distance_.clear();
        next_.clear();
        negative_cycle_ = false;
    }

    void initializeDistances() {
        const auto vertices = graph_.getVertices();

        for (const auto &i: vertices) {
            for (const auto &j: vertices) {
                if (i == j) {
                    distance_[i][j] = 0;
                } else {
                    distance_[i][j] = std::numeric_limits<weight_type>::max();
                }
                next_[i][j] = j;
            }
        }

        for (const auto &from: vertices) {
            try {
                for (const auto &to: graph_.getNeighbors(from)) {
                    weight_type weight = graph_.getEdgeWeight(from, to);
                    distance_[from][to] = weight;
                }
            } catch (const std::exception &) {
            }
        }
    }
};
