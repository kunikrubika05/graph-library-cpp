#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "graph/parallel_algorithms/bfs/parallel_breadth_first_search.hpp"
#include "graph/structure/csr_graph.hpp"
#include "graph/structure/csr_graph_builder.hpp"

namespace fs = std::filesystem;
using namespace graph::structure;
using namespace graph::parallel_algorithms;

void create_directory_if_not_exists(const std::string& path) {
  if (!fs::exists(path)) {
    fs::create_directories(path);
  }
}

int main(int argc, char* argv[]) {
  std::string dataset_path = "../datasets/socfb-A-anon/socfb-A-anon.mtx";

  std::cout << "Analyzing six degrees of separation on Facebook graph"
            << std::endl;
  std::cout << "Loading graph from " << dataset_path << std::endl;

  create_directory_if_not_exists("../six_handshakes/data");
  create_directory_if_not_exists("../six_handshakes/images");

  CSRGraphBuilder<int> builder;
  std::ifstream file(dataset_path);

  if (!fs::exists(dataset_path)) {
    std::cerr << "File does not exist: " << dataset_path << std::endl;
    std::cout << "Current working directory: " << fs::current_path()
              << std::endl;
    return 1;
  }

  if (!file.is_open()) {
    std::cerr << "Error opening file " << dataset_path << std::endl;
    return 1;
  }

  std::string line;
  std::getline(file, line);
  std::cout << "File header: " << line << std::endl;

  std::getline(file, line);
  std::istringstream dims(line);
  int num_vertices, _, num_edges;
  dims >> num_vertices >> _ >> num_edges;
  std::cout << "Graph dimensions: vertices=" << num_vertices
            << ", edges=" << num_edges << std::endl;

  int u, v;
  int edges_read = 0;
  auto start_load = std::chrono::high_resolution_clock::now();

  while (file >> u >> v) {
    u--;
    v--;
    builder.addEdge(u, v);
    builder.addEdge(v, u);

    edges_read++;
    if (edges_read % 1000000 == 0) {
      std::cout << "Read " << edges_read << " edges..." << std::endl;
    }
  }

  file.close();
  auto end_load = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> load_time = end_load - start_load;
  std::cout << "Graph loaded in " << load_time.count() << " seconds"
            << std::endl;

  auto start_build = std::chrono::high_resolution_clock::now();
  auto graph = builder.build();
  auto end_build = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> build_time = end_build - start_build;
  std::cout << "CSR graph built in " << build_time.count() << " seconds"
            << std::endl;

  const int NUM_SAMPLES = 100;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, graph.getVertexCount() - 1);

  std::vector<int> source_vertices;
  for (int i = 0; i < NUM_SAMPLES; ++i) {
    source_vertices.push_back(dis(gen));
  }

  std::vector<long long> path_counts(20, 0);
  std::vector<long long> path_percent(20, 0);
  long long total_paths = 0;
  long long unreachable = 0;

  auto start_bfs = std::chrono::high_resolution_clock::now();
  int max_threads = std::thread::hardware_concurrency();
  std::cout << "Running multithreaded BFS with " << max_threads
            << " threads from " << NUM_SAMPLES << " random vertices..."
            << std::endl;

  for (int i = 0; i < NUM_SAMPLES; ++i) {
    int source = source_vertices[i];
    std::cout << "Analyzing distances from vertex " << source << " (" << (i + 1)
              << "/" << NUM_SAMPLES << ")" << std::endl;

    ParallelBreadthFirstSearch<CSRGraph<int, void>> bfs(graph, max_threads);
    bfs.traverse(source);

    auto distances = bfs.getDistance();

    for (auto dist : distances) {
      if (dist >= 0) {
        path_counts[dist]++;
        total_paths++;
      } else {
        unreachable++;
      }
    }
  }

  auto end_bfs = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> bfs_time = end_bfs - start_bfs;
  std::cout << "BFS analysis completed in " << bfs_time.count() << " seconds"
            << std::endl;

  for (int i = 0; i < 20; ++i) {
    if (total_paths > 0) {
      path_percent[i] = 100.0 * path_counts[i] / total_paths;
    }
  }

  std::cout << "\nDistance distribution in social graph:" << std::endl;
  std::cout << std::setw(10) << "Distance" << std::setw(15) << "Count"
            << std::setw(15) << "Percent" << std::endl;
  std::cout << std::string(40, '-') << std::endl;

  for (int i = 0; i < 20; ++i) {
    if (path_counts[i] > 0) {
      std::cout << std::setw(10) << i << std::setw(15) << path_counts[i]
                << std::setw(15) << std::fixed << std::setprecision(2)
                << (100.0 * path_counts[i] / total_paths) << "%" << std::endl;
    }
  }

  long long paths_within_six = 0;
  for (int i = 0; i <= 6; ++i) {
    paths_within_six += path_counts[i];
  }

  double percent_within_six = 100.0 * paths_within_six / total_paths;
  std::cout << "\nSix degrees of separation analysis:" << std::endl;
  std::cout << "Total paths: " << total_paths << std::endl;
  std::cout << "Paths with length <= 6: " << paths_within_six << " ("
            << std::fixed << std::setprecision(2) << percent_within_six << "%)"
            << std::endl;
  std::cout << "Unreachable vertices: " << unreachable << std::endl;

  double avg_distance = 0;
  for (int i = 0; i < 20; ++i) {
    avg_distance += i * (path_counts[i] / (double)total_paths);
  }
  std::cout << "Average distance: " << std::fixed << std::setprecision(2)
            << avg_distance << std::endl;

  std::ofstream results("../six_handshakes/data/distance_distribution.csv");
  results << "distance,count,percent\n";
  for (int i = 0; i < 20; ++i) {
    if (path_counts[i] > 0) {
      results << i << "," << path_counts[i] << "," << std::fixed
              << std::setprecision(2) << (100.0 * path_counts[i] / total_paths)
              << "\n";
    }
  }
  results.close();

  std::ofstream cumulative(
      "../six_handshakes/data/cumulative_distribution.csv");
  cumulative << "distance,cumulative_percent\n";
  double cum_percent = 0;
  for (int i = 0; i < 20; ++i) {
    if (path_counts[i] > 0) {
      cum_percent += 100.0 * path_counts[i] / total_paths;
      cumulative << i << "," << std::fixed << std::setprecision(2)
                 << cum_percent << "\n";
    }
  }
  cumulative.close();

  std::cout << "\nResults saved in six_handshakes/data/ directory" << std::endl;
  return 0;
}
