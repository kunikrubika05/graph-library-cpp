#include <gtest/gtest.h>

#include <chrono>
#include <random>

#include "graph/algorithms/dijkstra/dijkstra.hpp"
#include "graph/parallel_algorithms/dijkstra/parallel_dijkstra.hpp"
#include "graph/structure/csr_graph.hpp"
#include "graph/structure/csr_graph_builder.hpp"
#include "graph/structure/directed_adjacency_list.hpp"

using namespace graph;
using CSRGraphType = structure::CSRGraph<int, double>;
using index_type = CSRGraphType::index_type;

class ParallelDijkstraTest : public ::testing::Test {
 protected:
  void SetUp() override {
    structure::CSRGraphBuilder<int, double> builder;
    builder.addVertex(0).addVertex(1).addVertex(2).addVertex(3).addVertex(4);

    builder.addEdge(0, 1, 2.0)
        .addEdge(0, 2, 1.0)
        .addEdge(1, 3, 3.0)
        .addEdge(2, 1, 1.0)
        .addEdge(2, 3, 5.0)
        .addEdge(2, 4, 4.0)
        .addEdge(3, 4, 2.0);

    csr_graph = builder.build();
  }

  structure::CSRGraph<int, double> csr_graph;
};

TEST_F(ParallelDijkstraTest, CorrectShortestPaths) {
  DirectedAdjacencyList<int, double> adj_graph;

  for (const auto& vertex : csr_graph.getVertices()) {
    adj_graph.addVertex(vertex);
  }

  for (const auto& u : csr_graph.getVertices()) {
    auto u_idx = csr_graph.getVertexIndex(u);
    auto [neighbors_begin, neighbors_end] = csr_graph.getNeighborRange(u_idx);

    for (auto it = neighbors_begin; it != neighbors_end; ++it) {
      int v_idx = *it;
      int v = csr_graph.getVertexByIndex(v_idx);
      double weight = csr_graph.getEdgeValue(static_cast<index_type>(u_idx),
                                             static_cast<index_type>(v_idx));
      adj_graph.addEdge(u, v, weight);
    }
  }

  algorithms::Dijkstra<DirectedAdjacencyList<int, double>> dijkstra(adj_graph);
  dijkstra.compute(0);
  auto std_distances = dijkstra.getAllDistances();

  parallel_algorithms::ParallelDijkstra<structure::CSRGraph<int, double>>
      parallel_dijkstra(csr_graph);
  parallel_dijkstra.findShortestPaths(0);
  auto parallel_distances = parallel_dijkstra.getDistances();

  for (size_t i = 0; i < csr_graph.getVertexCount(); ++i) {
    int vertex = csr_graph.getVertexByIndex(i);
    EXPECT_DOUBLE_EQ(std_distances.at(vertex), parallel_distances[i]);
  }
}

structure::CSRGraph<int, double> generateRandomGraph(int vertex_count,
                                                     int edge_factor,
                                                     double max_weight) {
  structure::CSRGraphBuilder<int, double> builder;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> weight_dist(1.0, max_weight);

  for (int i = 0; i < vertex_count; ++i) {
    builder.addVertex(i);
  }

  int edge_count = vertex_count * edge_factor;
  std::uniform_int_distribution<> vertex_dist(0, vertex_count - 1);

  for (int i = 0; i < edge_count; ++i) {
    int from = vertex_dist(gen);
    int to = vertex_dist(gen);
    if (from != to) {
      builder.addEdge(from, to, weight_dist(gen));
    }
  }

  return builder.build();
}

TEST_F(ParallelDijkstraTest, PerformanceComparison) {
  int vertex_count = 100000;
  int edge_factor = 1000;
  auto large_graph = generateRandomGraph(vertex_count, edge_factor, 100.0);
  int source_vertex = 0;

  DirectedAdjacencyList<int, double> adj_graph;
  for (int i = 0; i < vertex_count; ++i) {
    adj_graph.addVertex(i);
  }

  for (int u = 0; u < vertex_count; ++u) {
    auto [neighbors_begin, neighbors_end] = large_graph.getNeighborRange(u);
    for (auto it = neighbors_begin; it != neighbors_end; ++it) {
      int v = *it;
      double weight = large_graph.getEdgeValue(static_cast<index_type>(u),
                                               static_cast<index_type>(v));
      adj_graph.addEdge(u, v, weight);
    }
  }

  auto start_std = std::chrono::high_resolution_clock::now();
  algorithms::Dijkstra<DirectedAdjacencyList<int, double>> dijkstra(adj_graph);
  dijkstra.compute(source_vertex);
  auto end_std = std::chrono::high_resolution_clock::now();
  auto std_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_std - start_std)
          .count() /
      1000.0;

  auto start_parallel = std::chrono::high_resolution_clock::now();
  parallel_algorithms::ParallelDijkstra<structure::CSRGraph<int, double>>
      parallel_dijkstra(large_graph);
  parallel_dijkstra.findShortestPaths(source_vertex);
  auto end_parallel = std::chrono::high_resolution_clock::now();
  auto parallel_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                           end_parallel - start_parallel)
                           .count() /
                       1000.0;

  std::cout << "Vertices: " << vertex_count
            << ", Edges: " << large_graph.getEdgeCount() << "\n";

  std::cout << "Dijkstra: " << std_time << " seconds" << "\n";
  std::cout << "Parallel Dijkstra: " << parallel_time << " seconds" << "\n";

  std::cout << "Dijkstra/ParallelDijkstra speedup: " << std_time / parallel_time
            << "x" << "\n";
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
