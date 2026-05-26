#pragma once
#include <dense/Matrix.h>
#include <td/Types.h>
#include <cmath>
#include <iostream>
#include "MpcLayout.h"
#include "MpcCost.h"
#include "MpcConstraints.h"
#include "MpcKkt.h"
#include "MpcKktSolver.h"

namespace mpc {

class MpcSqp {
public:
    struct Settings {
        int maxIter = 10;
        double tol = 1e-3;
        double alpha = 1.0;
        bool log = true;
    };

    explicit MpcSqp(const MpcLayout& layout)
        : _layout(layout) {}

    bool Solve(const dense::DblMatrix& coeffs, double target_v, double initial_x, double dt, const Settings& cfg) {
        const td::UINT4 nZ = static_cast<td::UINT4>(_layout.totalSize());
        dense::DblMatrix zNom(nZ, 1, nullptr, true);

        MpcCost cost(_layout);
        cost.UpdateReferenceTrajectory(coeffs, target_v, initial_x, dt);
        auto zref = cost.Ref().getColumnManipulator();
        auto zn = zNom.getColumnManipulator();
        for (td::UINT4 i = 0; i < nZ; ++i) {
            zn(i) = zref(i);
        }

        for (int iter = 0; iter < cfg.maxIter; ++iter) {
            cost.UpdateReferenceTrajectory(coeffs, target_v, initial_x, dt);
            MpcConstraints constraints(_layout);
            constraints.UpdateNominalTrajectory(zNom);

            dense::DblMatrix init(4, 1, nullptr, true);
            auto initv = init.getColumnManipulator();
            initv(0) = zn(static_cast<td::UINT4>(_layout.idxX(0)));
            initv(1) = zn(static_cast<td::UINT4>(_layout.idxY(0)));
            initv(2) = zn(static_cast<td::UINT4>(_layout.idxPsi(0)));
            initv(3) = zn(static_cast<td::UINT4>(_layout.idxV(0)));
            constraints.setInitialState(init);

            MpcKkt kkt(_layout, cost, constraints);
            kkt.Assemble();

            MpcKktSolver solver;
            auto res = solver.Solve(kkt);
            if (!res.ok) {
                if (cfg.log) {
                    std::cout << "MpcSqp: solver.Solve returned !ok" << std::endl;
                    if (kkt.matrix()) {
                        std::cout << "  KKT nnz=" << kkt.matrix()->getNoOfNonZero() << std::endl;
                    } else {
                        std::cout << "  KKT matrix is null" << std::endl;
                    }
                }
                return false;
            }

            if (cfg.log) {
                std::cout << "MpcSqp: solver returned ok; res.z.size=" << res.z.size() << " expected=" << nZ << std::endl;
            }

            if (res.z.size() != static_cast<std::size_t>(nZ)) {
                if (cfg.log) {
                    std::cout << "MpcSqp: solver produced invalid z size=" << res.z.size() << " expected=" << nZ << std::endl;
                }
                return false;
            }

            double maxAbs = 0.0;
            for (td::UINT4 i = 0; i < nZ; ++i) {
                const double dz = res.z[i] - zn(i);
                if (std::fabs(dz) > maxAbs) {
                    maxAbs = std::fabs(dz);
                }
                zn(i) = zn(i) + cfg.alpha * dz;
            }

            if (cfg.log) {
                std::cout << "SQP iter=" << iter << " max|dZ|=" << maxAbs << std::endl;
            }
            if (maxAbs < cfg.tol) {
                return true;
            }
        }

        return false;
    }

    bool Solve(const dense::DblMatrix& coeffs,
               double target_v,
               double initial_x,
               double dt,
               const Settings& cfg,
               dense::DblMatrix& zNom,
               double init_x,
               double init_y,
               double init_psi,
               double init_v) {
        const td::UINT4 nZ = static_cast<td::UINT4>(_layout.totalSize());
        if (zNom.getNoOfRows() != nZ || zNom.getNoOfCols() != 1) {
            return false;
        }
        auto zn = zNom.getColumnManipulator();
        for (td::UINT4 i = 0; i < nZ; ++i) {
            if (!std::isfinite(zn(i))) {
                return false;
            }
        }

        MpcCost cost(_layout);

        for (int iter = 0; iter < cfg.maxIter; ++iter) {
            cost.UpdateReferenceTrajectory(coeffs, target_v, initial_x, dt);
            MpcConstraints constraints(_layout);
            constraints.UpdateNominalTrajectory(zNom);

            dense::DblMatrix init(4, 1, nullptr, true);
            auto initv = init.getColumnManipulator();
            initv(0) = init_x;
            initv(1) = init_y;
            initv(2) = init_psi;
            initv(3) = init_v;
            constraints.setInitialState(init);

            MpcKkt kkt(_layout, cost, constraints);
            kkt.Assemble();

            MpcKktSolver solver;
            auto res = solver.Solve(kkt);
            if (!res.ok) {
                if (cfg.log) {
                    std::cout << "MpcSqp: solver.Solve returned !ok (hot-start)" << std::endl;
                    if (kkt.matrix()) {
                        std::cout << "  KKT nnz=" << kkt.matrix()->getNoOfNonZero() << std::endl;
                    } else {
                        std::cout << "  KKT matrix is null" << std::endl;
                    }
                }
                return false;
            }

            if (cfg.log) {
                std::cout << "MpcSqp: solver returned ok (hot-start); res.z.size=" << res.z.size() << " expected=" << nZ << std::endl;
            }

            if (res.z.size() != static_cast<std::size_t>(nZ)) {
                if (cfg.log) {
                    std::cout << "MpcSqp: solver produced invalid z size (hot-start)=" << res.z.size() << " expected=" << nZ << std::endl;
                }
                return false;
            }

            double maxAbs = 0.0;
            for (td::UINT4 i = 0; i < nZ; ++i) {
                const double dz = res.z[i] - zn(i);
                if (std::fabs(dz) > maxAbs) {
                    maxAbs = std::fabs(dz);
                }
                zn(i) = zn(i) + cfg.alpha * dz;
            }

            if (cfg.log) {
                std::cout << "SQP iter=" << iter << " max|dZ|=" << maxAbs << std::endl;
            }
            if (maxAbs < cfg.tol) {
                return true;
            }
        }

        return false;
    }

private:
    const MpcLayout& _layout;
};

} // namespace mpc
