#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include "graph/parallel_algorithms/page_rank/parallel_page_rank.hpp"
#include "graph/parallel_algorithms/page_rank/parallel_page_rank_optimized.hpp"
#include "graph/structure/csr_graph.hpp"
#include "graph/structure/csr_graph_builder.hpp"

using namespace graph::structure;
using namespace graph::parallel_algorithms;

TEST(ParallelPageRankOptimized, CorrectnessTest) {
  CSRGraphBuilder<int, void> builder;

  builder.addEdge(0, 1);
  builder.addEdge(1, 2);
  builder.addEdge(2, 3);
  builder.addEdge(2, 1);

  auto graph = builder.build();

  ParallelPageRankOptimized<CSRGraph<int, void>> pageRank(graph, 2);
  auto ranks = pageRank.compute(0.85, 1e-6, 100);

  double sum = 0.0;
  for (auto rank : ranks) {
    sum += rank;
  }
  EXPECT_NEAR(sum, 1.0, 1e-6);

  EXPECT_GT(ranks[1], ranks[0]);
  EXPECT_GT(ranks[1], ranks[3]);

  std::cout << "PageRank ranks (optimized version):\n";
  for (size_t i = 0; i < ranks.size(); ++i) {
    std::cout << "Vertex " << i << ": " << ranks[i] << "\n";
  }
}

TEST(ParallelPageRankOptimized, PerformanceTest) {
  const size_t vertex_count = 1000;
  CSRGraphBuilder<int, void> builder(vertex_count, vertex_count * 10);

  for (int i = 0; i < vertex_count; ++i) {
    for (int j = 0; j < 10; ++j) {
      int target = (i + j * 97) % vertex_count;
      builder.addEdge(i, target);
    }
  }

  auto graph = builder.build();

  std::cout << "\n========== Performance test (graph with "
            << vertex_count << " vertices) ==========\n";

  for (size_t threads : {1, 2, 4, 8}) {
    auto start = std::chrono::high_resolution_clock::now();

    ParallelPageRankOptimized<CSRGraph<int, void>> pageRank(graph, threads);
    auto ranks = pageRank.compute(0.85, 1e-6, 50);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    std::cout << "Threads: " << threads << ", Time: " << elapsed.count()
              << " seconds\n";

    double sum = 0.0;
    for (auto rank : ranks) {
      sum += rank;
    }
    EXPECT_NEAR(sum, 1.0, 1e-6);
  }
}

TEST(ParallelPageRankOptimized, LargeGraphTest) {
  const size_t vertex_count = 10000;
  CSRGraphBuilder<int, void> builder(vertex_count, vertex_count * 20);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> distrib(0, vertex_count - 1);

  for (int i = 0; i < vertex_count; ++i) {
    int edges = 5 + distrib(gen) % 20;
    for (int j = 0; j < edges; ++j) {
      builder.addEdge(i, distrib(gen));
    }
  }

  auto graph = builder.build();

  std::cout << "\n========== Large graph test (" << vertex_count
            << " vertices) ==========\n";

  for (size_t threads : {1, 2, 4, 8, 16}) {
    auto start = std::chrono::high_resolution_clock::now();

    ParallelPageRankOptimized<CSRGraph<int, void>> pageRank(graph, threads);
    auto ranks = pageRank.compute(0.85, 1e-6, 30);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    std::cout << "Threads: " << threads << ", Time: " << elapsed.count()
              << " seconds\n";

    static double single_thread_time = 0;
    if (threads == 1) {
      single_thread_time = elapsed.count();
    } else if (single_thread_time > 0) {
      double speedup = single_thread_time / elapsed.count();
      std::cout << "Speedup: x" << speedup << "\n";
    }

    if (threads == 8) {
      std::vector<std::pair<double, int>> top_ranks;
      for (size_t i = 0; i < ranks.size(); ++i) {
        top_ranks.push_back({ranks[i], i});
      }

      std::partial_sort(
          top_ranks.begin(), top_ranks.begin() + 5, top_ranks.end(),
          [](const auto& a, const auto& b) { return a.first > b.first; });

      std::cout << "\nTop-5 vertices by PageRank:\n";
      for (int i = 0; i < 5; ++i) {
        std::cout << "Vertex " << top_ranks[i].second << ": "
                  << top_ranks[i].first << "\n";
      }
    }
  }
}

TEST(ParallelPageRankOptimized, ComparisonTest) {
  const size_t vertex_count = 5000;
  CSRGraphBuilder<int, void> builder(vertex_count, vertex_count * 10);

  for (int i = 0; i < vertex_count; ++i) {
    for (int j = 0; j < 10; ++j) {
      int target = (i + j * 97) % vertex_count;
      builder.addEdge(i, target);
    }
  }

  auto graph = builder.build();

  std::cout << "\n========== Comparison of basic and optimized versions ("
            << vertex_count << " vertices) ==========\n";

  const double damping = 0.85;
  const double tol = 1e-6;
  const int max_iter = 20;

  for (size_t threads : {1, 4, 8}) {
    std::cout << "\nNumber of threads: " << threads << "\n";

    {
      auto start = std::chrono::high_resolution_clock::now();

      ParallelPageRank<CSRGraph<int, void>> pageRank(graph, threads);
      auto ranks = pageRank.compute(damping, tol, max_iter);

      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = end - start;

      std::cout << "Basic version: " << elapsed.count() << " seconds\n";
    }

    {
      auto start = std::chrono::high_resolution_clock::now();

      ParallelPageRankOptimized<CSRGraph<int, void>> pageRank(graph, threads);
      auto ranks = pageRank.compute(damping, tol, max_iter);

      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = end - start;

      std::cout << "Optimized version: " << elapsed.count() << " seconds\n";
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
