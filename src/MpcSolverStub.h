#pragma once
#include <dense/Matrix.h>
#include <iostream>
#include "MpcLayout.h"
#include "MpcCost.h"
#include "MpcConstraints.h"

namespace mpc {

class MpcSolverStub {
public:
    explicit MpcSolverStub(std::size_t horizon = 20, double dt = 0.1, double lf = 0.5)
        : _layout(horizon, dt, lf) {}

    bool Solve(const dense::DblMatrix& initial_state, dense::DblMatrix& optimal_trajectory) {
        if (initial_state.getNoOfRows() * initial_state.getNoOfCols() < 4) {
            std::cout << "MPC stub: invalid initial_state size" << std::endl;
            return false;
        }

        std::cout << "MPC stub: building constraints" << std::endl;
        MpcCost cost(_layout);
        (void)cost;
        MpcConstraints constraints(_layout);
        constraints.setInitialState(initial_state);
        std::cout << "MPC stub: constraints built" << std::endl;

        const std::size_t N = _layout.N();
        optimal_trajectory = dense::DblMatrix(static_cast<td::UINT4>(N), 2, nullptr, true);

        auto init = initial_state.getColumnManipulator();
        double x0 = init(0);
        double y0 = init(1);
        double x1 = x0 + 1.0;
        double y1 = y0;

        auto traj = optimal_trajectory.getManipulator();
        for (std::size_t i = 0; i < N; ++i) {
            double alpha = (N == 1) ? 0.0 : static_cast<double>(i) / static_cast<double>(N - 1);
            traj(static_cast<td::UINT4>(i), 0) = x0 + (x1 - x0) * alpha;
            traj(static_cast<td::UINT4>(i), 1) = y0 + (y1 - y0) * alpha;
        }

        std::cout << "MPC stub: trajectory points=" << N << std::endl;
        return true;
    }

private:
    MpcLayout _layout;
};

} // namespace mpc
