#pragma once

namespace graph {
struct GraphOptions {
  bool checkDuplicateEdges = false;
  bool allowSelfLoops = false;

  GraphOptions() = default;
  GraphOptions(bool check_duplicates, bool allow_self_loops)
      : checkDuplicateEdges(check_duplicates),
        allowSelfLoops(allow_self_loops) {}
};
}  // namespace graph
