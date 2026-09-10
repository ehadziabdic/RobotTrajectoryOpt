#pragma once
#include <cstddef>
#include <utility>
#include <vector>
#include "MpcLayout.h"

namespace mpc {

struct IneqRow {
    std::vector<std::pair<td::INT4, double>> coeffs;
    double rhs = 0.0;
    int tag = 0;
};

// Tag encoding: (kind << 24) | (t << 8) | idx
// kind: 0=delta_min, 1=delta_max, 2=a_min, 3=a_max, 4=v_min, 5=v_max
inline std::vector<IneqRow> buildBoundRows(const MpcLayout& layout, double steerLimit, double accelLimit, double vMax) {
    std::vector<IneqRow> rows;
    const std::size_t N = layout.N();
    for (std::size_t t = 0; t < N - 1; ++t) {
        IneqRow r;

        // delta >= -steerLimit
        r.coeffs.clear();
        r.coeffs.push_back({static_cast<td::INT4>(layout.idxDelta(t)), 1.0});
        r.rhs = -steerLimit;
        r.tag = (0 << 24) | (static_cast<int>(t) << 8) | 0;
        rows.push_back(r);

        // -delta >= -steerLimit  (i.e. delta <= steerLimit)
        r.coeffs.clear();
        r.coeffs.push_back({static_cast<td::INT4>(layout.idxDelta(t)), -1.0});
        r.rhs = -steerLimit;
        r.tag = (1 << 24) | (static_cast<int>(t) << 8) | 0;
        rows.push_back(r);

        // a >= -accelLimit
        r.coeffs.clear();
        r.coeffs.push_back({static_cast<td::INT4>(layout.idxA(t)), 1.0});
        r.rhs = -accelLimit;
        r.tag = (2 << 24) | (static_cast<int>(t) << 8) | 0;
        rows.push_back(r);

        // -a >= -accelLimit  (i.e. a <= accelLimit)
        r.coeffs.clear();
        r.coeffs.push_back({static_cast<td::INT4>(layout.idxA(t)), -1.0});
        r.rhs = -accelLimit;
        r.tag = (3 << 24) | (static_cast<int>(t) << 8) | 0;
        rows.push_back(r);
    }

    // Velocity bounds for all timesteps (proposal: "cannot exceed maximum speed")
    for (std::size_t t = 0; t < N; ++t) {
        IneqRow r;

        // v >= 0
        r.coeffs.clear();
        r.coeffs.push_back({static_cast<td::INT4>(layout.idxV(t)), 1.0});
        r.rhs = 0.0;
        r.tag = (4 << 24) | (static_cast<int>(t) << 8) | 0;
        rows.push_back(r);

        // -v >= -vMax  (i.e. v <= vMax)
        r.coeffs.clear();
        r.coeffs.push_back({static_cast<td::INT4>(layout.idxV(t)), -1.0});
        r.rhs = -vMax;
        r.tag = (5 << 24) | (static_cast<int>(t) << 8) | 0;
        rows.push_back(r);
    }
    return rows;
}

} // namespace mpc
