#pragma once
#include <sparse/ISolver.h>
#include <mem/PointerReleaser.h>
#include <td/Types.h>
#include <iostream>
#include <vector>
#include "MpcKkt.h"

namespace mpc {

class MpcKktSolver {
public:
    struct Result {
        bool ok = false;
        std::vector<double> z;
        std::vector<double> lambda;
    };

    Result Solve(const MpcKkt& kkt) {
        Result out;
        const sparse::IDblMatrix* kktMat = kkt.matrix();
        if (!kktMat) {
            std::cout << "MPC Solver: KKT matrix null" << std::endl;
            return out;
        }

        const td::UINT4 n = static_cast<td::UINT4>(kktMat->getNoOfRows());
        const int nz = static_cast<int>(kktMat->getNoOfNonZero());
        const td::UINT4 nZ = kkt.primalSize();

        sparse::DblSolverReleaser solver(sparse::createDblSolver(
            static_cast<int>(n),
            nz,
            sparse::Symmetry::SymmetricIndef,
            sparse::SolverType::LDLT,
            sparse::Pivoting::DiagonalSinglePass));

        if (!solver.ptr()) {
            std::cout << "MPC Solver: createDblSolver failed" << std::endl;
            return out;
        }

        const auto& trips = kkt.triplets();
        for (const auto& t : trips) {
            solver->addTriple(t.row, t.col, t.val);
        }

        auto rhs = kkt.rhs().getColumnManipulator();
        for (td::UINT4 i = 0; i < n; ++i) {
            solver->setRHS(static_cast<td::INT4>(i), rhs(i));
        }

        if (!solver->factorize()) {
            std::cout << "MPC Solver: factorize failed" << std::endl;
            return out;
        }
        if (!solver->solve()) {
            std::cout << "MPC Solver: solve failed" << std::endl;
            return out;
        }

        out.z.resize(nZ);
        for (td::UINT4 i = 0; i < nZ; ++i) {
            out.z[i] = solver->x(static_cast<td::INT4>(i));
        }
        const td::UINT4 nLambda = n - nZ;
        out.lambda.resize(static_cast<std::size_t>(nLambda));
        for (td::UINT4 i = 0; i < nLambda; ++i) {
            out.lambda[i] = solver->x(static_cast<td::INT4>(nZ + i));
        }
        out.ok = true;
        return out;
    }
};

} // namespace mpc
