#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "graph/parallel_algorithms/page_rank/parallel_page_rank_optimized.hpp"
#include "graph/structure/csr_graph_builder.hpp"

using namespace graph::structure;
using namespace graph::parallel_algorithms;

int main() {
  const double damping_factor = 0.85;
  const double tolerance = 1e-6;
  const int max_iterations = 50;
  const int top_n = 100;

  std::cout << "Loading Google web graph dataset..." << std::endl;

  std::ifstream file("../datasets/web-Google.txt");
  if (!file) {
    std::cerr << "Error: Could not open dataset file" << std::endl;
    return 1;
  }

  std::string line;
  while (std::getline(file, line) && line[0] == '#') {
  }

  CSRGraphBuilder<int, void> builder(900000, 5500000);

  int src, dst;
  std::istringstream iss(line);
  if (iss >> src >> dst) {
    builder.addEdge(src, dst);
  }

  while (std::getline(file, line)) {
    std::istringstream iss(line);
    if (iss >> src >> dst) {
      builder.addEdge(src, dst);
    }
  }

  std::cout << "Building CSR graph..." << std::endl;
  auto graph = builder.build();
  std::cout << "Graph built: " << graph.getVertexCount() << " vertices, "
            << graph.getEdgeCount() << " edges" << std::endl;

  std::vector<std::pair<int, double>> benchmark_results;
  std::vector<double> result_ranks;

  std::cout
      << "\n========== PageRank Performance on Google Web Graph ==========\n";

  for (int threads : {1, 2, 4, 8, 16}) {
    std::cout << "Running PageRank with " << threads << " threads..."
              << std::endl;

    auto start = std::chrono::high_resolution_clock::now();

    ParallelPageRankOptimized<CSRGraph<int, void>> pageRank(graph, threads);
    auto ranks = pageRank.compute(damping_factor, tolerance, max_iterations);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    benchmark_results.push_back({threads, elapsed.count()});
    std::cout << "Threads: " << threads << ", Time: " << elapsed.count()
              << " seconds" << std::endl;

    if (threads == 16) {
      result_ranks = ranks;
    }

    if (threads > 1) {
      double speedup = benchmark_results[0].second / elapsed.count();
      std::cout << "Speedup: x" << speedup << std::endl;
    }
  }

  std::cout << "\n========== Top " << top_n
            << " Vertices by PageRank ==========\n";

  std::vector<std::pair<double, int>> vertex_ranks;
  for (int i = 0; i < result_ranks.size(); i++) {
    vertex_ranks.push_back({result_ranks[i], i});
  }

  std::partial_sort(
      vertex_ranks.begin(), vertex_ranks.begin() + top_n, vertex_ranks.end(),
      [](const auto& a, const auto& b) { return a.first > b.first; });

  std::cout << std::setw(10) << "Rank" << std::setw(15) << "Vertex ID"
            << std::setw(20) << "PageRank Value" << std::endl;
  std::cout << std::string(45, '-') << std::endl;

  for (int i = 0; i < top_n; i++) {
    std::cout << std::setw(10) << (i + 1) << std::setw(15)
              << vertex_ranks[i].second << std::setw(20) << std::fixed
              << std::setprecision(8) << vertex_ranks[i].first << std::endl;
  }

  ///////
  std::ofstream ranks_file("../page_rank_visualization/data/pagerank_top1000.csv");
  if (ranks_file) {
    ranks_file << "rank,vertex_id,pagerank\n";
    for (int i = 0; i < std::min(1000, static_cast<int>(vertex_ranks.size())); i++) {
      ranks_file << (i+1) << "," << vertex_ranks[i].second << ","
                 << std::setprecision(10) << vertex_ranks[i].first << "\n";
    }
    std::cout << "PageRank results exported to visualization/page_rank/data/pagerank_top1000.csv\n";
  }

  std::ofstream perf_file("../page_rank_visualization/data/performance.csv");
  if (perf_file) {
    perf_file << "threads,time,speedup\n";
    for (const auto& result : benchmark_results) {
      double speedup = benchmark_results[0].second / result.second;
      perf_file << result.first << "," << result.second << "," << speedup << "\n";
    }
    std::cout << "Performance data exported to visualization/page_rank/data/performance.csv\n";
  }
  ///////

  return 0;
}
