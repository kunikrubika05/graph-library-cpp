cmake_minimum_required(VERSION 3.10)
project(graph_analysis_library LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(graph_analysis INTERFACE)
target_include_directories(graph_analysis INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

enable_testing()
find_package(GTest REQUIRED)

#structure tests
add_executable(run_dir_adj_list tests/tests_structure/test_directed_adjacency_list.cpp)
add_executable(run_un_dir_adj_list tests/tests_structure/test_undirected_adjacency_list.cpp)
add_executable(run_dir_adj_matrix tests/tests_structure/test_directed_adjacency_matrix.cpp)
add_executable(run_un_dir_adj_matrix tests/tests_structure/test_undirected_adjacency_matrix.cpp)
add_executable(run_dir_edge_list tests/tests_structure/test_directed_edge_list.cpp)
add_executable(run_un_dir_edge_list tests/tests_structure/test_undirected_edge_list.cpp)
add_executable(run_converter tests/tests_structure/test_converter.cpp)

#DFS tests
add_executable(run_dfs_base tests/tests_dfs/test_base.cpp)
add_executable(run_dfs_components tests/tests_dfs/test_components.cpp)
add_executable(run_dfs_bipartite tests/tests_dfs/test_bipartite_checker.cpp)
add_executable(run_dfs_cycle tests/tests_dfs/test_cycles.cpp)
add_executable(run_dfs_top_sort tests/tests_dfs/test_topological_sort.cpp)
add_executable(run_dfs_bridges tests/tests_dfs/test_bridges.cpp)
add_executable(run_dfs_articulation_points tests/tests_dfs/test_articulation_points.cpp)
add_executable(run_dfs_condensation tests/tests_dfs/test_condensation.cpp)
add_executable(run_dfs_biconnected_components tests/tests_dfs/test_biconnected_components.cpp)

#BFS tests
add_executable(run_bfs_base tests/tests_bfs/test_base.cpp)
add_executable(run_bfs_components tests/tests_bfs/test_components.cpp)
add_executable(run_bfs_bipartite tests/tests_bfs/test_bipartite_checker.cpp)
add_executable(run_bfs_level_ordering tests/tests_bfs/test_level_order_traversal.cpp)

#Dijkstra tests
add_executable(run_dijkstra_base tests/tests_dijkstra/test_base.cpp)
add_executable(run_dijkstra_astar tests/tests_dijkstra/test_astar.cpp)
add_executable(run_dijkstra_faster tests/tests_dijkstra/test_dijsktra_faster.cpp)

#Floyd tests
add_executable(run_floyd_base tests/tests_floyd/test_base.cpp)
add_executable(run_transitive_closure_metrics_neg_cycle tests/tests_floyd/test_transitive_closure_metrics_negative_cycle.cpp)

#Bellman Ford tests
add_executable(run_bellman_ford_base tests/tests_bellman_ford/test_base.cpp)
add_executable(run_bellman_ford_advanced tests/tests_bellman_ford/test_advanced.cpp)

#MST tests
add_executable(run_mst_kruskal_base tests/tests_mst/test_kruskal.cpp)
add_executable(run_mst_kruskal_advanced tests/tests_mst/test_kruskal_advanced.cpp)
add_executable(run_mst_prim_base tests/tests_mst/test_prim.cpp)
add_executable(run_mst_prim_advanced tests/tests_mst/test_prim_advanced.cpp)

#Visualizer tests
add_executable(run_visualizer_base include/graph/visualization/test_graph_export.cpp)
add_executable(run_visualizer_s_g_i include/graph/visualization/save_graph_image.cpp)
add_executable(run_visualizer_friends include/graph/visualization/friends.cpp)
add_executable(run_visualizer_friends_multigraph include/graph/visualization/friends_multigraph.cpp)

add_executable(run_visualizer_mst include/graph/visualization/mst_visualization.cpp)
add_executable(run_visualizer_mst_comparison include/graph/visualization/mst_comparison_visualization.cpp)
add_executable(run_visualizer_mst_comparison_2 include/graph/visualization/mst_comparison_visualization_2.cpp)

#Flow tests
add_executable(run_flow_ford_fulkerson tests/tests_flow/test_ford_fulkerson.cpp)
add_executable(run_flow_edmonds_karp tests/tests_flow/test_edmonds_karp.cpp)
add_executable(run_flow_dinic tests/tests_flow/test_dinic.cpp)

#matching tests
add_executable(run_matching_kuhn tests/tests_matching/test_kuhn.cpp)
add_executable(run_matching_hopcroft_karp tests/tests_matching/test_hopcroft_karp.cpp)
add_executable(run_matching_edmonds tests/tests_matching/test_edmonds.cpp)

#structure tests
target_link_libraries(run_dir_adj_list PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_un_dir_adj_list PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_dir_adj_matrix PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_un_dir_adj_matrix PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_dir_edge_list PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_un_dir_edge_list PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_converter PRIVATE graph_analysis GTest::GTest GTest::Main pthread)

#DFS tests
target_link_libraries(run_dfs_base PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_dfs_components PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_dfs_bipartite PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_dfs_cycle PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_dfs_top_sort PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_dfs_bridges PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_dfs_articulation_points PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_dfs_condensation PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_dfs_biconnected_components PRIVATE graph_analysis GTest::GTest GTest::Main pthread)

#BFS tests
target_link_libraries(run_bfs_base PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_bfs_components PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_bfs_bipartite PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_bfs_level_ordering PRIVATE graph_analysis GTest::GTest GTest::Main pthread)

#Dijkstra tests
target_link_libraries(run_dijkstra_base PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_dijkstra_astar PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_dijkstra_faster PRIVATE graph_analysis GTest::GTest GTest::Main pthread)

#Floyd tests
target_link_libraries(run_floyd_base PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_transitive_closure_metrics_neg_cycle PRIVATE graph_analysis GTest::GTest GTest::Main pthread)

#Bellman Ford tests
target_link_libraries(run_bellman_ford_base PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_bellman_ford_advanced PRIVATE graph_analysis GTest::GTest GTest::Main pthread)

#MST tests
target_link_libraries(run_mst_kruskal_base PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_mst_kruskal_advanced PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_mst_prim_base PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_mst_prim_advanced PRIVATE graph_analysis GTest::GTest GTest::Main pthread)

#Visualizer tests
target_link_libraries(run_visualizer_base PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_visualizer_s_g_i PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_visualizer_friends PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_visualizer_friends_multigraph PRIVATE graph_analysis GTest::GTest GTest::Main pthread)

target_link_libraries(run_visualizer_mst PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_visualizer_mst_comparison PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_visualizer_mst_comparison_2 PRIVATE graph_analysis GTest::GTest GTest::Main pthread)

#Flow tests
target_link_libraries(run_flow_ford_fulkerson PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_flow_edmonds_karp PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_flow_dinic PRIVATE graph_analysis GTest::GTest GTest::Main pthread)

#matching tests
target_link_libraries(run_matching_kuhn PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_matching_hopcroft_karp PRIVATE graph_analysis GTest::GTest GTest::Main pthread)
target_link_libraries(run_matching_edmonds PRIVATE graph_analysis GTest::GTest GTest::Main pthread)

#structure tests
add_test(NAME dir_adj_list COMMAND run_un_dir_adj_list)
add_test(NAME un_dir_adj_list COMMAND run_un_dir_adj_list)
add_test(NAME dir_adj_matrix COMMAND run_dir_adj_matrix)
add_test(NAME un_dir_adj_matrix COMMAND run_un_dir_adj_matrix)
add_test(NAME dir_edge_list COMMAND run_dir_edge_list)
add_test(NAME un_dir_edge_list COMMAND run_un_dir_edge_list)
add_test(NAME converter COMMAND run_converter)

#DFS tests
add_test(NAME dfs_base COMMAND run_dfs_base)
add_test(NAME dfs_components COMMAND run_dfs_components)
add_test(NAME dfs_bipartite COMMAND run_dfs_bipartite)
add_test(NAME dfs_cycle COMMAND run_dfs_cycle)
add_test(NAME dfs_top_sort COMMAND run_dfs_top_sort)
add_test(NAME dfs_bridges COMMAND run_dfs_bridges)
add_test(NAME dfs_articulation_points COMMAND run_dfs_articulation_points)
add_test(NAME dfs_condensation COMMAND run_dfs_condensation)
add_test(NAME dfs_biconnected_components COMMAND run_dfs_biconnected_components)

#BFS tests
add_test(NAME bfs_base COMMAND run_bfs_base)
add_test(NAME bfs_components COMMAND run_bfs_components)
add_test(NAME bfs_bipartite COMMAND run_bfs_bipartite)
add_test(NAME bfs_level_ordering COMMAND run_bfs_level_ordering)

#Dijkstra tests
add_test(NAME dijkstra_base COMMAND run_dijkstra_base)
add_test(NAME dijkstra_astar COMMAND run_dijkstra_astar)
add_test(NAME dijkstra_faster COMMAND run_dijkstra_faster)

#Floyd tests
add_test(NAME floyd_base COMMAND run_floyd_base)
add_test(NAME transitive_closure_metrics_neg_cycle COMMAND run_transitive_closure_metrics_neg_cycle)

#Bellman Ford tests
add_test(NAME bellman_ford_base COMMAND run_bellman_ford_base)
add_test(NAME bellman_ford_advanced COMMAND run_bellman_ford_advanced)

#MST tests
add_test(NAME mst_kruskal_base COMMAND run_mst_kruskal_base)
add_test(NAME mst_kruskal_advanced COMMAND run_mst_kruskal_advanced)
add_test(NAME mst_prim_base COMMAND run_mst_prim_base)
add_test(NAME mst_prim_advanced COMMAND run_mst_prim_advanced)

#Visualizer tests
add_test(NAME visualizer_base COMMAND run_visualizer_base)
add_test(NAME visualizer_s_g_i COMMAND run_visualizer_s_g_i)
add_test(NAME visualizer_friends COMMAND run_visualizer_friends)
add_test(NAME visualizer_friends_multigraph COMMAND run_visualizer_friends_multigraph)

add_test(NAME visualizer_mst COMMAND run_visualizer_mst)
add_test(NAME visualizer_mst_comparison COMMAND run_visualizer_mst_comparison)
add_test(NAME visualizer_mst_comparison_2 COMMAND run_visualizer_mst_comparison)

#Flow tests
add_test(NAME flow_ford_fulkerson COMMAND run_flow_ford_fulkerson)
add_test(NAME flow_edmonds_karp COMMAND run_flow_edmonds_karp)
add_test(NAME flow_dinic COMMAND run_flow_dinic)

#matching tests
add_test(NAME matching_kuhn COMMAND run_matching_kuhn)
add_test(NAME matching_hopcroft_karp COMMAND run_matching_hopcroft_karp)
add_test(NAME matching_edmonds COMMAND run_matching_edmonds)
