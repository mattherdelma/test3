#include "LinearAssignment.h"

#include <algorithm>
#include <limits>
#include <vector>

namespace rtp {

namespace {

constexpr float kLarge = 1e6f;   // padding cost for the square cost matrix

// Core Jonker-Volgenant solver on a dense square cost matrix `cost` (n x n).
// Fills `rowsol[i]` = column assigned to row i. Returns total cost.
// Compact, allocation-light port of the classic dense LAPJV.
float lapjvInternal(int n, const std::vector<std::vector<float>>& cost,
                    std::vector<int>& rowsol, std::vector<int>& colsol) {
    std::vector<float> v(n, 0.f);            // column potentials
    std::vector<int>   colsol_local(n, -1);
    rowsol.assign(n, -1);
    std::vector<int>   matches(n, 0);

    // --- Column reduction ---------------------------------------------------
    for (int j = n - 1; j >= 0; --j) {
        float min_v = cost[0][j];
        int   imin  = 0;
        for (int i = 1; i < n; ++i) {
            if (cost[i][j] < min_v) { min_v = cost[i][j]; imin = i; }
        }
        v[j] = min_v;
        if (++matches[imin] == 1) { rowsol[imin] = j; colsol_local[j] = imin; }
        else                       { colsol_local[j] = -1; }
    }

    // --- Reduction transfer for unassigned rows -----------------------------
    std::vector<int> free_rows;
    for (int i = 0; i < n; ++i) {
        if (matches[i] == 0) {
            free_rows.push_back(i);
        } else if (matches[i] == 1) {
            const int j1 = rowsol[i];
            float min_d = std::numeric_limits<float>::infinity();
            for (int j = 0; j < n; ++j) {
                if (j != j1 && cost[i][j] - v[j] < min_d)
                    min_d = cost[i][j] - v[j];
            }
            v[j1] -= min_d;
        }
    }

    // --- Augmenting path search (Dijkstra-like) for each free row -----------
    std::vector<float> d(n);
    std::vector<int>   pred(n);
    std::vector<char>  collist_done(n);

    for (int free_i : free_rows) {
        for (int j = 0; j < n; ++j) {
            d[j]            = cost[free_i][j] - v[j];
            pred[j]         = free_i;
            collist_done[j] = 0;
        }

        int endofpath = -1;
        while (true) {
            // Find the cheapest not-yet-scanned column.
            float min_d = std::numeric_limits<float>::infinity();
            int   j = -1;
            for (int jj = 0; jj < n; ++jj) {
                if (!collist_done[jj] && d[jj] < min_d) { min_d = d[jj]; j = jj; }
            }
            if (j == -1) break;
            collist_done[j] = 1;

            const int i = colsol_local[j];
            if (i == -1) { endofpath = j; break; }      // free column found

            // Relax through the row currently owning column j.
            for (int jj = 0; jj < n; ++jj) {
                if (collist_done[jj]) continue;
                const float h = cost[i][jj] - v[jj] - min_d;
                if (h < d[jj]) { d[jj] = h; pred[jj] = i; }
            }
        }
        if (endofpath == -1) continue;  // degenerate; should not happen if n>0

        // Update potentials for scanned columns.
        const float dmin = d[endofpath];
        for (int j = 0; j < n; ++j) {
            if (collist_done[j]) v[j] += d[j] - dmin;
        }

        // Augment along the path.
        int j = endofpath;
        while (true) {
            const int i = pred[j];
            colsol_local[j] = i;
            const int prev = rowsol[i];
            rowsol[i] = j;
            if (i == free_i) break;
            j = prev;
        }
    }

    colsol = colsol_local;
    float total = 0.f;
    for (int i = 0; i < n; ++i) total += cost[i][rowsol[i]];
    return total;
}

} // namespace

Assignment linearAssignment(const std::vector<std::vector<float>>& cost_in,
                            float thresh) {
    Assignment out;
    const int rows = static_cast<int>(cost_in.size());
    const int cols = rows ? static_cast<int>(cost_in[0].size()) : 0;

    if (rows == 0 || cols == 0) {
        for (int i = 0; i < rows; ++i) out.unmatched_rows.push_back(i);
        for (int j = 0; j < cols; ++j) out.unmatched_cols.push_back(j);
        return out;
    }

    // Pad to a square matrix; padded cells get a prohibitively large cost so a
    // real assignment is always preferred when one is admissible.
    const int n = std::max(rows, cols);
    std::vector<std::vector<float>> cost(n, std::vector<float>(n, kLarge));
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j) cost[i][j] = cost_in[i][j];

    std::vector<int> rowsol, colsol;
    lapjvInternal(n, cost, rowsol, colsol);

    std::vector<char> row_matched(rows, 0), col_matched(cols, 0);
    for (int i = 0; i < rows; ++i) {
        const int j = rowsol[i];
        if (j < cols && cost_in[i][j] <= thresh) {
            out.matches.emplace_back(i, j);
            row_matched[i] = 1;
            col_matched[j] = 1;
        }
    }
    for (int i = 0; i < rows; ++i) if (!row_matched[i]) out.unmatched_rows.push_back(i);
    for (int j = 0; j < cols; ++j) if (!col_matched[j]) out.unmatched_cols.push_back(j);
    return out;
}

} // namespace rtp
