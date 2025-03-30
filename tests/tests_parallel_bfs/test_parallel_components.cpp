#include <gtest/gtest.h>

#include <chrono>
#include <random>

#include "graph/algorithms/bfs/components.hpp"
#include "graph/algorithms/dfs/components.hpp"
#include "graph/parallel_algorithms/bfs/parallel_connected_components.hpp"
#include "graph/structure/csr_graph_builder.hpp"

using namespace graph::structure;
using namespace graph::parallel_algorithms;
using namespace graph::algorithms;

TEST(ParallelComponentsTest, MatchesSequentialComponents) {
  CSRGraphBuilder<int, void> builder;

  builder.addVertex(0).addVertex(1).addVertex(2);
  builder.addEdge(0, 1).addEdge(1, 2);

  builder.addVertex(3).addVertex(4);
  builder.addEdge(3, 4);

  builder.addVertex(5);

  auto csr_graph = builder.build();

  DFSConnectedComponents<CSRGraph<int, void>> dfs_cc(csr_graph);
  auto dfs_components = dfs_cc.findAll();

  BFSConnectedComponents<CSRGraph<int, void>> bfs_cc(csr_graph);
  auto bfs_components = bfs_cc.computeComponents();

  ParallelConnectedComponents<CSRGraph<int, void>> parallel_cc(csr_graph, 2);
  auto parallel_components = parallel_cc.computeComponents();

  EXPECT_EQ(dfs_components.size(), parallel_components.size());
  EXPECT_EQ(bfs_components.size(), parallel_components.size());
  EXPECT_EQ(dfs_components.size(), 3);
}

TEST(ParallelComponentsTest, ComplexGraph) {
  CSRGraphBuilder<int, void> builder;

  for (int i = 0; i < 5; i++) {
    builder.addVertex(i);
    for (int j = 0; j < i; j++) {
      builder.addEdge(i, j);
    }
  }

  for (int i = 5; i < 10; i++) {
    builder.addVertex(i);
    if (i > 5) {
      builder.addEdge(i - 1, i);
    }
  }

  builder.addVertex(10);
  for (int i = 11; i < 15; i++) {
    builder.addVertex(i);
    builder.addEdge(10, i);
  }

  for (int i = 15; i < 20; i++) {
    builder.addVertex(i);
    if (i > 15) {
      builder.addEdge(i - 1, i);
    }
  }
  builder.addEdge(19, 15);

  for (int i = 20; i < 25; i++) {
    builder.addVertex(i);
  }

  auto csr_graph = builder.build();

  DFSConnectedComponents<CSRGraph<int, void>> dfs_cc(csr_graph);
  auto dfs_components = dfs_cc.findAll();

  BFSConnectedComponents<CSRGraph<int, void>> bfs_cc(csr_graph);
  auto bfs_components = bfs_cc.computeComponents();

  ParallelConnectedComponents<CSRGraph<int, void>> parallel_cc(csr_graph, 4);
  auto parallel_components = parallel_cc.computeComponents();

  EXPECT_EQ(dfs_components.size(), 13);
  EXPECT_EQ(bfs_components.size(), dfs_components.size());
  EXPECT_EQ(parallel_components.size(), dfs_components.size());
}

TEST(ParallelComponentsTest, LargeRandomGraph) {
  const int NUM_VERTICES = 10000;
  const int NUM_COMPONENTS = 20;
  const int EDGES_PER_COMPONENT = 50000;

  std::cout << "Vertices: " << NUM_VERTICES
            << ", Components: " << NUM_COMPONENTS
            << ", Edges per component: " << EDGES_PER_COMPONENT << "\n";

  std::random_device rd;
  std::mt19937 gen(rd());

  CSRGraphBuilder<int, void> builder;

  for (int i = 0; i < NUM_VERTICES; i++) {
    builder.addVertex(i);
  }

  int vertices_per_component = NUM_VERTICES / NUM_COMPONENTS;

  for (int c = 0; c < NUM_COMPONENTS; c++) {
    int start = c * vertices_per_component;
    int end = (c + 1) * vertices_per_component - 1;

    std::uniform_int_distribution<> dist(start, end);

    for (int e = 0; e < EDGES_PER_COMPONENT; e++) {
      int from = dist(gen);
      int to = dist(gen);
      if (from != to) {
        builder.addEdge(from, to);
      }
    }
  }

  auto csr_graph = builder.build();

  auto start_dfs = std::chrono::high_resolution_clock::now();
  DFSConnectedComponents<CSRGraph<int, void>> dfs_cc(csr_graph);
  auto dfs_components = dfs_cc.findAll();
  auto end_dfs = std::chrono::high_resolution_clock::now();

  auto duration_dfs =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_dfs - start_dfs)
          .count() /
      1000.0;

  auto start_bfs = std::chrono::high_resolution_clock::now();
  BFSConnectedComponents<CSRGraph<int, void>> bfs_cc(csr_graph);
  auto bfs_components = bfs_cc.computeComponents();
  auto end_bfs = std::chrono::high_resolution_clock::now();

  auto duration_bfs =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_bfs - start_bfs)
          .count() /
      1000.0;

  auto start_par = std::chrono::high_resolution_clock::now();
  ParallelConnectedComponents<CSRGraph<int, void>> parallel_cc(csr_graph);
  auto parallel_components = parallel_cc.computeComponents();
  auto end_par = std::chrono::high_resolution_clock::now();

  auto duration_par =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_par - start_par)
          .count() /
      1000.0;

  EXPECT_EQ(dfs_components.size(), NUM_COMPONENTS);
  EXPECT_EQ(bfs_components.size(), NUM_COMPONENTS);
  EXPECT_EQ(parallel_components.size(), NUM_COMPONENTS);

  std::cout << "DFS components time: " << duration_dfs << " seconds\n";
  std::cout << "BFS components time: " << duration_bfs << " seconds\n";
  std::cout << "Parallel components time: " << duration_par << " seconds\n";
  std::cout << "DFS/Parallel speedup: " << duration_dfs / duration_par << "x\n";
  std::cout << "BFS/Parallel speedup: " << duration_bfs / duration_par << "x\n";
}

TEST(ParallelComponentsTest, DifferentThreadCounts) {
  //  const int NUM_VERTICES = 5000;
  //  const int NUM_COMPONENTS = 10;
  //  const int EDGES_PER_COMPONENT = 10000;

  const int NUM_VERTICES = 500000;
  const int NUM_COMPONENTS = 99;
  const int EDGES_PER_COMPONENT = 100000;

  std::random_device rd;
  std::mt19937 gen(rd());

  CSRGraphBuilder<int, void> builder;

  for (int i = 0; i < NUM_VERTICES; i++) {
    builder.addVertex(i);
  }

  int vertices_per_component = NUM_VERTICES / NUM_COMPONENTS;

  for (int c = 0; c < NUM_COMPONENTS; c++) {
    int start = c * vertices_per_component;
    int end = (c + 1) * vertices_per_component - 1;

    std::uniform_int_distribution<> dist(start, end);

    for (int e = 0; e < EDGES_PER_COMPONENT; e++) {
      int from = dist(gen);
      int to = dist(gen);
      if (from != to) {
        builder.addEdge(from, to);
      }
    }
  }

  auto csr_graph = builder.build();

  BFSConnectedComponents<CSRGraph<int, void>> bfs_cc(csr_graph);
  auto bfs_components = bfs_cc.computeComponents();

  std::cout << "Thread scaling test:\n";
  for (int num_threads : {1, 2, 4, 8, 16}) {
    auto start_par = std::chrono::high_resolution_clock::now();
    ParallelConnectedComponents<CSRGraph<int, void>> parallel_cc(csr_graph,
                                                                 num_threads);
    auto parallel_components = parallel_cc.computeComponents();
    auto end_par = std::chrono::high_resolution_clock::now();

    auto duration_par = std::chrono::duration_cast<std::chrono::milliseconds>(
                            end_par - start_par)
                            .count() /
                        1000.0;

    EXPECT_EQ(parallel_components.size(), bfs_components.size());
    std::cout << "  " << num_threads << " threads: " << duration_par
              << " seconds\n";
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
