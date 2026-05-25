#pragma once
#include <dense/Matrix.h>
#include <td/Types.h>
#include <vector>
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

            std::cout << "SQP iter=" << iter << " max|dZ|=" << maxAbs << std::endl;
            if (maxAbs < cfg.tol) {
                return true;
            }
        }

        return true;
    }

private:
    const MpcLayout& _layout;
};

} // namespace mpc
