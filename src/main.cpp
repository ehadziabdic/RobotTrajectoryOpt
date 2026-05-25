#include <mu/Application.h>
#include <dense/Matrix.h>
#include <iostream>
#include "MpcCost.h"
#include "MpcConstraints.h"
#include "MpcKkt.h"
#include "MpcKktSolver.h"
#include "MpcSqp.h"
#include "MpcSolverStub.h"

int main(int argc, const char* argv[]) {
    mu::Application app(argc, argv);

    std::cout << "MpcCore start" << std::endl;

    mpc::MpcLayout layout(20, 0.1, 0.5);

    dense::DblMatrix coeffs(4, 1, nullptr, true);
    auto c = coeffs.getColumnManipulator();
    c(0) = 0.0;
    c(1) = 0.0;
    c(2) = 0.0;
    c(3) = 0.0;

    dense::DblMatrix initial(4, 1, nullptr, true);
    auto init = initial.getColumnManipulator();
    init(0) = 0.0; // x
    init(1) = 0.0; // y
    init(2) = 0.0; // psi
    init(3) = 1.0; // v

    mpc::MpcSqp sqp(layout);
    mpc::MpcSqp::Settings cfg;
    cfg.maxIter = 5;
    cfg.tol = 1e-3;
    cfg.alpha = 1.0;

    sqp.Solve(coeffs, 1.0, 0.0, layout.dt(), cfg);

    mpc::MpcSolverStub solver;
    dense::DblMatrix trajectory;
    solver.Solve(initial, trajectory);

    std::cout << "MpcCore end" << std::endl;

    return 0;
}
