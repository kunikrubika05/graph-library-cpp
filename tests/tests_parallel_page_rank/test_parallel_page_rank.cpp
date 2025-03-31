#include <gtest/gtest.h>

#include <chrono>
#include <iostream>

#include "graph/parallel_algorithms/page_rank/parallel_page_rank.hpp"
#include "graph/structure/csr_graph.hpp"
#include "graph/structure/csr_graph_builder.hpp"

using namespace graph::structure;
using namespace graph::parallel_algorithms;

TEST(ParallelPageRank, SimpleGraphTest) {
  CSRGraphBuilder<int, void> builder;

  builder.addEdge(0, 1);
  builder.addEdge(1, 2);
  builder.addEdge(2, 3);
  builder.addEdge(2, 1);

  auto graph = builder.build();

  ParallelPageRank<CSRGraph<int, void>> pageRank(graph, 2);
  auto ranks = pageRank.compute(0.85, 1e-6, 100);

  double sum = 0.0;
  for (auto rank : ranks) {
    sum += rank;
  }
  EXPECT_NEAR(sum, 1.0, 1e-6);

  EXPECT_GT(ranks[1], ranks[0]);
  EXPECT_GT(ranks[1], ranks[3]);

  std::cout << "PageRank values:\n";
  for (size_t i = 0; i < ranks.size(); ++i) {
    std::cout << "Vertex " << i << ": " << ranks[i] << "\n";
  }
}

TEST(ParallelPageRank, PerformanceTest) {
  const size_t vertex_count = 1000;
  CSRGraphBuilder<int, void> builder(vertex_count, vertex_count * 10);

  for (int i = 0; i < vertex_count; ++i) {
    for (int j = 0; j < 10; ++j) {
      int target = (i + j * 97) % vertex_count;
      builder.addEdge(i, target);
    }
  }

  auto graph = builder.build();

  for (size_t threads : {1, 2, 4, 8}) {
    auto start = std::chrono::high_resolution_clock::now();

    ParallelPageRank<CSRGraph<int, void>> pageRank(graph, threads);
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

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
