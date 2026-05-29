#pragma once
//
// Linear assignment via the Jonker-Volgenant algorithm (lapjv), wrapped in a
// ByteTrack-style helper that turns a cost matrix + gate threshold into matched
// pairs and the leftover row/column indices.
//
#include <vector>

namespace rtp {

struct Assignment {
    std::vector<std::pair<int, int>> matches;     // (row, col)
    std::vector<int> unmatched_rows;
    std::vector<int> unmatched_cols;
};

// `cost[i][j]` is the cost of matching row i to col j. Pairs with cost greater
// than `thresh` are rejected (left unmatched). Empty input is handled.
Assignment linearAssignment(const std::vector<std::vector<float>>& cost,
                            float thresh);

} // namespace rtp
