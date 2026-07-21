#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>
#include <sparse/ISolver.h>
#include <mem/PointerReleaser.h>
#include <dense/Matrix.h>
#include "MpcLayout.h"
#include "MpcCost.h"
#include "MpcConstraints.h"
#include "MpcInequality.h"

namespace mpc {

struct ActiveSetResult {
    bool ok = false;
    std::vector<double> z;
    std::vector<IneqRow> workingSet;
};

inline ActiveSetResult SolveActiveSetQP(
    const MpcLayout& layout,
    const MpcCost& cost,
    const MpcConstraints& constraints,
    const std::vector<IneqRow>& allIneqRows,
    const std::vector<IneqRow>& warmStartSet,
    int maxInnerIter,
    bool verbose,
    const std::vector<double>& zInitial) {

    const td::UINT4 nZ = static_cast<td::UINT4>(layout.totalSize());
    const td::UINT4 nCBase = static_cast<td::UINT4>(constraints.rowCount());

    const auto& baseTriplets = constraints.triplets();
    auto bBase = constraints.rhs().getColumnManipulator();
    auto hDiag = cost.H().getColumnManipulator();
    auto gDiag = cost.G().getColumnManipulator();

    // Warm-start active set from previous solve: match by tag, use current coefficients
    std::vector<IneqRow> activeSet;
    for (const auto& ws : warmStartSet) {
        auto it = std::find_if(allIneqRows.begin(), allIneqRows.end(),
            [&ws](const IneqRow& r) { return r.tag == ws.tag; });
        if (it != allIneqRows.end()) {
            activeSet.push_back(*it);
        }
    }

    if (verbose) {
        std::cout << "ActiveSet: warm-start active rows=" << activeSet.size()
                  << " total_ineq=" << allIneqRows.size() << std::endl;
    }

    std::vector<double> zCurrent = zInitial;
    int innerIter = 0;

    for (; innerIter < maxInnerIter; ++innerIter) {
        td::UINT4 nW = static_cast<td::UINT4>(activeSet.size());
        td::UINT4 kktSize = nZ + nCBase + nW;

        td::INT4 baseTc = 0;
        for (std::size_t i = 0; i < baseTriplets.size(); ++i) ++baseTc;
        td::INT4 activeCc = 0;
        for (const auto& row : activeSet) activeCc += static_cast<td::INT4>(row.coeffs.size());
        td::INT4 nnzEst = static_cast<td::INT4>(nZ) + 2 * baseTc + 2 * activeCc + 64;

        sparse::DblSolverReleaser solver(sparse::createDblSolver(
            static_cast<int>(kktSize),
            nnzEst,
            sparse::Symmetry::SymmetricIndef,
            sparse::SolverType::LDLT,
            sparse::Pivoting::DiagonalSinglePass));

        if (!solver.ptr()) {
            if (verbose) std::cout << "ActiveSet: createDblSolver failed" << std::endl;
            return {false, zCurrent, activeSet};
        }

        for (td::UINT4 i = 0; i < nZ; ++i) {
            double h = hDiag(static_cast<td::UINT4>(i));
            // Tikhonov regularization (like MpcKkt) to prevent singular KKT
            if (h < 1e-8) h = 1e-8;
            solver->addTriple(static_cast<td::INT4>(i), static_cast<td::INT4>(i), h);
        }

        for (const auto& t : baseTriplets) {
            td::INT4 rowInKKT = static_cast<td::INT4>(nZ) + t.row;
            solver->addTriple(rowInKKT, t.col, t.val);
            solver->addTriple(t.col, rowInKKT, t.val);
        }

        for (td::UINT4 w = 0; w < nW; ++w) {
            td::INT4 rowInKKT = static_cast<td::INT4>(nZ + nCBase + w);
            for (std::size_t ci = 0; ci < activeSet[w].coeffs.size(); ++ci) {
                td::INT4 col = activeSet[w].coeffs[ci].first;
                double coeff = activeSet[w].coeffs[ci].second;
                solver->addTriple(rowInKKT, col, coeff);
                solver->addTriple(col, rowInKKT, coeff);
            }
        }

        for (td::UINT4 i = 0; i < nZ; ++i) {
            solver->setRHS(static_cast<td::INT4>(i), -gDiag(static_cast<td::UINT4>(i)));
        }
        for (td::UINT4 i = 0; i < nCBase; ++i) {
            solver->setRHS(static_cast<td::INT4>(nZ + i), bBase(static_cast<td::UINT4>(i)));
        }
        for (td::UINT4 w = 0; w < nW; ++w) {
            solver->setRHS(static_cast<td::INT4>(nZ + nCBase + w), activeSet[w].rhs);
        }

        if (!solver->factorize()) {
            if (verbose) std::cout << "ActiveSet: factorize failed iter=" << innerIter << std::endl;
            return {false, zCurrent, activeSet};
        }
        if (!solver->solve()) {
            if (verbose) std::cout << "ActiveSet: solve failed iter=" << innerIter << std::endl;
            return {false, zCurrent, activeSet};
        }

        std::vector<double> zCandidate(nZ);
        for (td::UINT4 i = 0; i < nZ; ++i) {
            zCandidate[i] = solver->x(static_cast<td::INT4>(i));
        }

        std::vector<double> lambda(nW);
        for (td::UINT4 w = 0; w < nW; ++w) {
            lambda[w] = solver->x(static_cast<td::INT4>(nZ + nCBase + w));
        }

        // --- Feasibility check against inactive inequalities ---
        double alpha = 1.0;
        const IneqRow* blocking = nullptr;

        for (const auto& ineq : allIneqRows) {
            bool isActive = false;
            for (const auto& act : activeSet) {
                if (act.tag == ineq.tag) { isActive = true; break; }
            }
            if (isActive) continue;

            double aTdir = 0.0;
            for (std::size_t ci = 0; ci < ineq.coeffs.size(); ++ci) {
                td::INT4 col = ineq.coeffs[ci].first;
                double coeff = ineq.coeffs[ci].second;
                aTdir += coeff * zCandidate[static_cast<std::size_t>(col)];
                aTdir -= coeff * zCurrent[static_cast<std::size_t>(col)];
            }

            if (aTdir >= 0.0) continue;

            double aTzCurr = 0.0;
            for (std::size_t ci = 0; ci < ineq.coeffs.size(); ++ci) {
                td::INT4 col = ineq.coeffs[ci].first;
                double coeff = ineq.coeffs[ci].second;
                aTzCurr += coeff * zCurrent[static_cast<std::size_t>(col)];
            }

            double room = (aTzCurr - ineq.rhs) / (-aTdir);
            if (room < alpha && room >= -1e-12) {
                alpha = room;
                blocking = &ineq;
            }
        }

        if (blocking != nullptr) {
            // Prevent adding near-duplicate rows (distance + slack-nonneg for same (t,obs))
            bool isDuplicate = false;
            for (const auto& act : activeSet) {
                if (act.coeffs.size() == blocking->coeffs.size()) {
                    double diff = 0.0;
                    for (std::size_t ci = 0; ci < blocking->coeffs.size(); ++ci) {
                        double a = blocking->coeffs[ci].second;
                        double b = act.coeffs[ci].second;
                        diff += (a - b) * (a - b);
                    }
                    if (diff < 1e-12) { isDuplicate = true; break; }
                }
            }
            if (isDuplicate) {
                if (verbose) std::cout << "ActiveSet: blocked duplicate tag=" << blocking->tag << std::endl;
            } else {
                for (td::UINT4 i = 0; i < nZ; ++i) {
                    zCurrent[i] += alpha * (zCandidate[i] - zCurrent[i]);
                }
                activeSet.push_back(*blocking);
                if (verbose) {
                    std::cout << "ActiveSet: added tag=" << blocking->tag << " alpha=" << alpha << std::endl;
                }
            }
            continue;
        }

        // Full step is feasible
        zCurrent = zCandidate;

        // --- Optimality check via dual variables ---
        if (nW == 0) {
            if (verbose) std::cout << "ActiveSet: converged " << (innerIter + 1) << " iters (unconstrained)" << std::endl;
            return {true, zCurrent, activeSet};
        }

        td::INT4 mostNegativeIdx = -1;
        double mostNegativeVal = std::numeric_limits<double>::max();
        for (td::UINT4 w = 0; w < nW; ++w) {
            if (lambda[w] < mostNegativeVal) {
                mostNegativeVal = lambda[w];
                mostNegativeIdx = static_cast<td::INT4>(w);
            }
        }

        constexpr double kDualTol = -1e-8;
        if (mostNegativeIdx >= 0 && mostNegativeVal < kDualTol) {
            int droppedTag = activeSet[mostNegativeIdx].tag;
            activeSet.erase(activeSet.begin() + mostNegativeIdx);
            if (verbose) {
                std::cout << "ActiveSet: dropped tag=" << droppedTag << " lambda=" << mostNegativeVal << std::endl;
            }
            continue;
        }

        if (verbose) std::cout << "ActiveSet: converged " << (innerIter + 1) << " iters" << std::endl;
        return {true, zCurrent, activeSet};
    }

    if (verbose) {
        std::cout << "ActiveSet: maxInnerIter=" << maxInnerIter << " reached, returning best feasible" << std::endl;
    }
    return {true, zCurrent, activeSet};
}

} // namespace mpc
