#include <gtest/gtest.h>

#include <chrono>
#include <random>
#include <thread>

#include "graph/algorithms/bfs/breadth_first_search.hpp"
#include "graph/parallel_algorithms/bfs/parallel_breadth_first_search.hpp"
#include "graph/structure/csr_graph_builder.hpp"

using namespace graph::structure;
using namespace graph::parallel_algorithms;
using namespace graph::algorithms;

TEST(ParallelBFSTest, MatchesSequentialBFS) {
  CSRGraphBuilder<std::string, int> builder;
  builder.addVertex("A")
      .addVertex("B")
      .addVertex("C")
      .addVertex("D")
      .addVertex("E")
      .addEdge("A", "B", 1)
      .addEdge("A", "C", 2)
      .addEdge("B", "D", 3)
      .addEdge("C", "D", 4)
      .addEdge("D", "E", 5);

  auto csr_graph = builder.build();

  BreadthFirstSearch<CSRGraph<std::string, int>> sequential_bfs(csr_graph);
  sequential_bfs.traverse("A");

  ParallelBreadthFirstSearch<CSRGraph<std::string, int>> parallel_bfs(csr_graph,
                                                                      2);
  parallel_bfs.traverse("A");

  auto seq_visited = sequential_bfs.getVisited();
  auto par_visited = parallel_bfs.getVisited();

  auto seq_dist = sequential_bfs.getDistance();
  auto par_dist = parallel_bfs.getDistance();

  EXPECT_EQ(seq_dist["A"], 0);
  EXPECT_EQ(seq_dist["B"], 1);
  EXPECT_EQ(seq_dist["C"], 1);
  EXPECT_EQ(seq_dist["D"], 2);
  EXPECT_EQ(seq_dist["E"], 3);

  for (const auto& vertex : csr_graph.getVertices()) {
    EXPECT_EQ(par_dist[csr_graph.getVertexIndex(vertex)], seq_dist[vertex]);
  }
}

TEST(ParallelBFSTest, DifferentThreadCounts) {
  CSRGraphBuilder<int, int> builder;

  for (int i = 0; i < 5; i++) {
    builder.addVertex(i);
    for (int j = 0; j < i; j++) {
      builder.addEdge(i, j, 1);
      builder.addEdge(j, i, 1);
    }
  }

  auto csr_graph = builder.build();

  for (int num_threads : {1, 2, 4}) {
    ParallelBreadthFirstSearch<CSRGraph<int, int>> parallel_bfs(csr_graph,
                                                                num_threads);
    parallel_bfs.traverse(0);

    auto distances = parallel_bfs.getDistance();

    for (size_t i = 0; i < distances.size(); i++) {
      EXPECT_TRUE(distances[i] == 0 || distances[i] == 1);
    }
  }
}

TEST(ParallelBFSTest, GraphWithCycles) {
  CSRGraphBuilder<std::string, int> builder;

  builder.addVertex("A")
      .addVertex("B")
      .addVertex("C")
      .addVertex("D")
      .addEdge("A", "B", 1)
      .addEdge("B", "C", 1)
      .addEdge("C", "D", 1)
      .addEdge("D", "A", 1);

  auto csr_graph = builder.build();

  ParallelBreadthFirstSearch<CSRGraph<std::string, int>> parallel_bfs(csr_graph,
                                                                      4);
  parallel_bfs.traverse("A");

  auto distances = parallel_bfs.getDistance();
  auto parents = parallel_bfs.getParent();

  EXPECT_EQ(distances[csr_graph.getVertexIndex("A")], 0);
  EXPECT_EQ(distances[csr_graph.getVertexIndex("B")], 1);
  EXPECT_EQ(distances[csr_graph.getVertexIndex("C")], 2);
  EXPECT_EQ(distances[csr_graph.getVertexIndex("D")], 3);

  auto d_idx = csr_graph.getVertexIndex("D");
  auto c_idx = parents[d_idx];
  auto b_idx = parents[c_idx];
  auto a_idx = parents[b_idx];

  EXPECT_EQ(csr_graph.getVertexByIndex(c_idx), "C");
  EXPECT_EQ(csr_graph.getVertexByIndex(b_idx), "B");
  EXPECT_EQ(csr_graph.getVertexByIndex(a_idx), "A");
}

TEST(ParallelBFSTest, Callbacks) {
  CSRGraphBuilder<int, void> builder;

  for (int i = 0; i < 5; i++) {
    builder.addVertex(i);
    if (i > 0) {
      builder.addEdge(i - 1, i);
    }
  }

  auto csr_graph = builder.build();

  std::vector<int> discovered_vertices;
  std::vector<std::pair<int, int>> traversed_edges;

  ParallelBreadthFirstSearch<CSRGraph<int, void>> parallel_bfs(csr_graph, 2);
  parallel_bfs.traverse(
      0, [](const int& v) { /* on_start callback */ },
      [&discovered_vertices](const int& v) {
        discovered_vertices.push_back(v);
      },
      [&traversed_edges](const int& u, const int& v) {
        traversed_edges.emplace_back(u, v);
      });

  EXPECT_EQ(discovered_vertices.size(), 5);

  EXPECT_EQ(traversed_edges.size(), 4);

  EXPECT_EQ(discovered_vertices[0], 0);
}

///////
TEST(ParallelBFSTest, LargeRandomGraph) {
  const int NUM_VERTICES = 10000;
  const int NUM_EDGES = 50000000;

  std::cout << "Vertices: " << NUM_VERTICES << ", Edges: " << NUM_EDGES << "\n";

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> vertices_dist(0, NUM_VERTICES - 1);

  CSRGraphBuilder<int, int> builder;

  for (int i = 0; i < NUM_VERTICES; i++) {
    builder.addVertex(i);
  }

  for (int i = 0; i < NUM_EDGES; i++) {
    int from = vertices_dist(gen);
    int to = vertices_dist(gen);
    if (from != to) {
      builder.addEdge(from, to, 1);
    }
  }

  auto csr_graph = builder.build();

  const int start_vertex = 0;

  auto start_seq = std::chrono::high_resolution_clock::now();
  BreadthFirstSearch<CSRGraph<int, int>> sequential_bfs(csr_graph);
  sequential_bfs.traverse(start_vertex);
  auto end_seq = std::chrono::high_resolution_clock::now();

  auto duration_seq =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_seq - start_seq)
          .count() /
      1000.0;

  auto start_par = std::chrono::high_resolution_clock::now();
  ParallelBreadthFirstSearch<CSRGraph<int, int>> parallel_bfs(csr_graph);
  parallel_bfs.traverse(start_vertex);
  auto end_par = std::chrono::high_resolution_clock::now();

  auto duration_par =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_par - start_par)
          .count() /
      1000.0;

  auto seq_visited = sequential_bfs.getVisited();
  auto par_visited = parallel_bfs.getVisited();

  int visited_count = 0;
  for (const auto& [vertex, visited] : seq_visited) {
    if (visited) ++visited_count;
  }

  int par_visited_count = 0;
  for (size_t i = 0; i < par_visited.size(); i++) {
    if (par_visited[i]) ++par_visited_count;
  }

  EXPECT_EQ(visited_count, par_visited_count);

  std::cout << "BFS time: " << duration_seq << " seconds\n";
  std::cout << "Parallel BFS time: " << duration_par << " seconds\n";

  std::cout << duration_seq / duration_par << "x speedup\n";
}
///////

TEST(ParallelBFSTest, DisconnectedComponents) {
  CSRGraphBuilder<char, int> builder;

  builder.addVertex('A')
      .addVertex('B')
      .addVertex('C')
      .addEdge('A', 'B', 1)
      .addEdge('B', 'C', 1);

  builder.addVertex('D')
      .addVertex('E')
      .addVertex('F')
      .addEdge('D', 'E', 1)
      .addEdge('E', 'F', 1);

  auto csr_graph = builder.build();

  ParallelBreadthFirstSearch<CSRGraph<char, int>> parallel_bfs(csr_graph, 2);
  parallel_bfs.traverse('A');

  auto visited = parallel_bfs.getVisited();

  EXPECT_TRUE(visited[csr_graph.getVertexIndex('A')]);
  EXPECT_TRUE(visited[csr_graph.getVertexIndex('B')]);
  EXPECT_TRUE(visited[csr_graph.getVertexIndex('C')]);

  EXPECT_FALSE(visited[csr_graph.getVertexIndex('D')]);
  EXPECT_FALSE(visited[csr_graph.getVertexIndex('E')]);
  EXPECT_FALSE(visited[csr_graph.getVertexIndex('F')]);

  parallel_bfs.traverse('D');
  visited = parallel_bfs.getVisited();

  EXPECT_TRUE(visited[csr_graph.getVertexIndex('D')]);
  EXPECT_TRUE(visited[csr_graph.getVertexIndex('E')]);
  EXPECT_TRUE(visited[csr_graph.getVertexIndex('F')]);
}

/////
TEST(ParallelBFSTest, LargeRandomGraphBFS) {
  const int NUM_VERTICES = 1000;
  const int NUM_EDGES = 5000;

  std::cout << "Vertices: " << NUM_VERTICES << ", Edges: " << NUM_EDGES << "\n";

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> vertices_dist(0, NUM_VERTICES - 1);

  CSRGraphBuilder<int, int> builder;

  for (int i = 0; i < NUM_VERTICES; i++) {
    builder.addVertex(i);
  }

  for (int i = 0; i < NUM_EDGES; i++) {
    int from = vertices_dist(gen);
    int to = vertices_dist(gen);
    if (from != to) {
      builder.addEdge(from, to, 1);
    }
  }

  auto csr_graph = builder.build();

  const int start_vertex = 0;

  auto start_seq = std::chrono::high_resolution_clock::now();
  BreadthFirstSearch<CSRGraph<int, int>> sequential_bfs(csr_graph);
  sequential_bfs.traverse(start_vertex);
  auto end_seq = std::chrono::high_resolution_clock::now();

  auto duration_seq =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_seq - start_seq)
          .count() /
      1000.0;

  auto seq_visited = sequential_bfs.getVisited();

  int visited_count = 0;
  for (const auto& [vertex, visited] : seq_visited) {
    if (visited) ++visited_count;
  }

  std::cout << "BFS time: " << duration_seq << " seconds\n";
}
TEST(ParallelBFSTest, LargeRandomGraphParallelBFS) {
  const int NUM_VERTICES = 1000;
  const int NUM_EDGES = 5000;

  std::cout << "Vertices: " << NUM_VERTICES << ", Edges: " << NUM_EDGES << "\n";

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> vertices_dist(0, NUM_VERTICES - 1);

  CSRGraphBuilder<int, int> builder;

  for (int i = 0; i < NUM_VERTICES; i++) {
    builder.addVertex(i);
  }

  for (int i = 0; i < NUM_EDGES; i++) {
    int from = vertices_dist(gen);
    int to = vertices_dist(gen);
    if (from != to) {
      builder.addEdge(from, to, 1);
    }
  }

  auto csr_graph = builder.build();

  const int start_vertex = 0;

  auto start_par = std::chrono::high_resolution_clock::now();
  ParallelBreadthFirstSearch<CSRGraph<int, int>> parallel_bfs(csr_graph);
  parallel_bfs.traverse(start_vertex);
  auto end_par = std::chrono::high_resolution_clock::now();

  auto duration_par =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_par - start_par)
          .count() /
      1000.0;

  auto par_visited = parallel_bfs.getVisited();

  int par_visited_count = 0;
  for (size_t i = 0; i < par_visited.size(); i++) {
    if (par_visited[i]) ++par_visited_count;
  }

  std::cout << "Parallel BFS time: " << duration_par << " seconds\n";
}
/////

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
