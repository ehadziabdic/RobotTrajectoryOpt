#pragma once
#include <algorithm>
#include <dense/Matrix.h>
#include <td/Types.h>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>
#include "MpcLayout.h"
#include "MpcCost.h"
#include "MpcConstraints.h"
#include "MpcKkt.h"
#include "MpcKktSolver.h"

namespace mpc {

class MpcSqp {
public:
    struct Settings {
        int maxIter = 30;
        double tol = 2e-3;
        double alpha = 0.12;
        double steerLimit = 0.6;
        double accelLimit = 3.0;
        bool verbose = false;
    };

    explicit MpcSqp(const MpcLayout& layout)
        : _layout(layout) {}

    int lastIterations() const { return _lastIterations; }
    double lastMaxAbs() const { return _lastMaxAbs; }
    bool lastConverged() const { return _lastConverged; }

    bool Solve(const dense::DblMatrix& coeffs,
               double target_v,
               double initial_x,
               double initial_y,
               double dt,
               const Settings& cfg,
               const std::vector<Obstacle>& obstacles,
               double initial_psi,
               bool freezeAtPeak = false,
               double maxLookahead = 15.0) {
        _lastIterations = 0;
        _lastMaxAbs = 0.0;
        _lastConverged = false;

        const td::UINT4 nZ = static_cast<td::UINT4>(_layout.totalSize());
        dense::DblMatrix zNom(nZ, 1, nullptr, true);

        MpcCost solverCost(_layout);
        solverCost.setFreezeAtPeak(freezeAtPeak);
        solverCost.setMaxLookahead(maxLookahead);
        solverCost.UpdateReferenceTrajectory(coeffs, target_v, initial_x, initial_y, dt, initial_psi);
        auto zref = solverCost.Ref().getColumnManipulator();
        auto zn = zNom.getColumnManipulator();
        for (td::UINT4 i = 0; i < nZ; ++i) {
            zn(i) = zref(i);
        }
        sanitizeTrajectory(zNom, cfg);

        dense::DblMatrix bestZ = zNom.makeCopy();
        double bestMaxAbs = std::numeric_limits<double>::max();

        for (int iter = 0; iter < cfg.maxIter; ++iter) {
            solverCost.UpdateReferenceTrajectory(coeffs, target_v, initial_x, initial_y, dt, initial_psi);
            MpcConstraints constraints(_layout);
            constraints.setVerbose(cfg.verbose);
            constraints.setObstacles(obstacles);
            constraints.UpdateNominalTrajectory(zNom);

            dense::DblMatrix init(4, 1, nullptr, true);
            auto initv = init.getColumnManipulator();
            initv(0) = zn(static_cast<td::UINT4>(_layout.idxX(0)));
            initv(1) = zn(static_cast<td::UINT4>(_layout.idxY(0)));
            initv(2) = initial_psi;
            initv(3) = zn(static_cast<td::UINT4>(_layout.idxV(0)));
            constraints.setInitialState(init);

            MpcKkt kkt(_layout, solverCost, constraints);
            kkt.Assemble();

            MpcKktSolver solver;
            auto res = solver.Solve(kkt);
            if (!res.ok) {
                if (cfg.verbose) {
                    std::cout << "MpcSqp: solver.Solve returned !ok" << std::endl;
                    if (kkt.matrix()) {
                        std::cout << "  KKT nnz=" << kkt.matrix()->getNoOfNonZero() << std::endl;
                    } else {
                        std::cout << "  KKT matrix is null" << std::endl;
                    }
                }
                return false;
            }

            if (cfg.verbose) {
                std::cout << "MpcSqp: solver returned ok; res.z.size=" << res.z.size() << " expected=" << nZ << std::endl;
            }

            if (res.z.size() != static_cast<std::size_t>(nZ)) {
                if (cfg.verbose) {
                    std::cout << "MpcSqp: solver produced invalid z size=" << res.z.size() << " expected=" << nZ << std::endl;
                }
                return false;
            }

            sanitizeTrajectoryVector(res.z, cfg);

            double maxAbs = 0.0;
            for (td::UINT4 i = 0; i < nZ; ++i) {
                double dz = res.z[i] - zn(i);
                if (isPsiIndex(i)) {
                    dz = wrapAngle(dz);
                }
                if (std::fabs(dz) > maxAbs) {
                    maxAbs = std::fabs(dz);
                }
                if (isPsiIndex(i)) {
                    zn(i) = wrapAngle(zn(i) + cfg.alpha * dz);
                } else {
                    zn(i) = zn(i) + cfg.alpha * dz;
                }
            }

            sanitizeTrajectory(zNom, cfg);

            if (cfg.verbose) {
                std::cout << "SQP iter=" << iter << " max|dZ|=" << maxAbs << std::endl;
            }
            _lastIterations = iter + 1;
            _lastMaxAbs = maxAbs;
            if (maxAbs < bestMaxAbs) {
                bestMaxAbs = maxAbs;
                bestZ = zNom.makeCopy();
            }
            if (maxAbs < cfg.tol) {
                _lastConverged = true;
                return true;
            }
        }

        _lastIterations = cfg.maxIter;
        zNom = bestZ.makeCopy();
        sanitizeTrajectory(zNom, cfg);
        _lastMaxAbs = bestMaxAbs;
        if (cfg.verbose) {
            std::cout << "[MPC Failsafe] maxIter reached. Rolling back to iteration with lowest objective; min max|dZ| = " << bestMaxAbs << "." << std::endl;
        }
        _lastConverged = false;
        return true;
    }

    bool Solve(const dense::DblMatrix& coeffs,
               double target_v,
               double initial_x,
               double initial_y,
               double dt,
               const Settings& cfg,
               const std::vector<Obstacle>& obstacles,
               dense::DblMatrix& zNom,
               double init_x,
               double init_y,
               double init_psi,
               double init_v,
               double initial_psi,
               bool freezeAtPeak = false,
               double maxLookahead = 15.0) {
        _lastIterations = 0;
        _lastMaxAbs = 0.0;
        _lastConverged = false;

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
        sanitizeTrajectory(zNom, cfg);

        MpcCost solverCost(_layout);
        solverCost.setFreezeAtPeak(freezeAtPeak);
        solverCost.setMaxLookahead(maxLookahead);
        dense::DblMatrix bestZ = zNom.makeCopy();
        double bestMaxAbs = std::numeric_limits<double>::max();

        for (int iter = 0; iter < cfg.maxIter; ++iter) {
            solverCost.UpdateReferenceTrajectory(coeffs, target_v, initial_x, initial_y, dt, initial_psi);
            MpcConstraints constraints(_layout);
            constraints.setVerbose(cfg.verbose);
            constraints.setObstacles(obstacles);
            constraints.UpdateNominalTrajectory(zNom);

            dense::DblMatrix init(4, 1, nullptr, true);
            auto initv = init.getColumnManipulator();
            initv(0) = init_x;
            initv(1) = init_y;
            initv(2) = init_psi;
            initv(3) = init_v;
            constraints.setInitialState(init);

            MpcKkt kkt(_layout, solverCost, constraints);
            kkt.Assemble();

            MpcKktSolver solver;
            auto res = solver.Solve(kkt);
            if (!res.ok) {
                if (cfg.verbose) {
                    std::cout << "MpcSqp: solver.Solve returned !ok (hot-start)" << std::endl;
                    if (kkt.matrix()) {
                        std::cout << "  KKT nnz=" << kkt.matrix()->getNoOfNonZero() << std::endl;
                    } else {
                        std::cout << "  KKT matrix is null" << std::endl;
                    }
                }
                return false;
            }

            if (cfg.verbose) {
                std::cout << "MpcSqp: solver returned ok (hot-start); res.z.size=" << res.z.size() << " expected=" << nZ << std::endl;
            }

            if (res.z.size() != static_cast<std::size_t>(nZ)) {
                if (cfg.verbose) {
                    std::cout << "MpcSqp: solver produced invalid z size (hot-start)=" << res.z.size() << " expected=" << nZ << std::endl;
                }
                return false;
            }

            sanitizeTrajectoryVector(res.z, cfg);

            double maxAbs = 0.0;
            for (td::UINT4 i = 0; i < nZ; ++i) {
                double dz = res.z[i] - zn(i);
                if (isPsiIndex(i)) {
                    dz = wrapAngle(dz);
                }
                if (std::fabs(dz) > maxAbs) {
                    maxAbs = std::fabs(dz);
                }
                if (isPsiIndex(i)) {
                    zn(i) = wrapAngle(zn(i) + cfg.alpha * dz);
                } else {
                    zn(i) = zn(i) + cfg.alpha * dz;
                }
            }

            sanitizeTrajectory(zNom, cfg);

            if (cfg.verbose) {
                std::cout << "SQP iter=" << iter << " max|dZ|=" << maxAbs << std::endl;
            }
            _lastIterations = iter + 1;
            _lastMaxAbs = maxAbs;
            if (maxAbs < bestMaxAbs) {
                bestMaxAbs = maxAbs;
                bestZ = zNom.makeCopy();
            }
            if (maxAbs < cfg.tol) {
                _lastConverged = true;
                return true;
            }
        }

        _lastIterations = cfg.maxIter;
        zNom = bestZ.makeCopy();
        sanitizeTrajectory(zNom, cfg);
        _lastMaxAbs = bestMaxAbs;
        if (cfg.verbose) {
            std::cout << "[MPC Failsafe] maxIter reached. Rolling back to iteration with lowest objective; min max|dZ| = " << bestMaxAbs << "." << std::endl;
        }
        _lastConverged = false;
        return true;
    }

private:
    static double wrapAngle(double angle) {
        constexpr double kPi = 3.1415926535897932384626433832795;
        constexpr double kTwoPi = 6.2831853071795864769252867665590;
        while (angle > kPi) {
            angle -= kTwoPi;
        }
        while (angle < -kPi) {
            angle += kTwoPi;
        }
        return angle;
    }

    bool isPsiIndex(td::UINT4 idx) const {
        const td::UINT4 start = static_cast<td::UINT4>(_layout.psiStart());
        const td::UINT4 end = static_cast<td::UINT4>(_layout.vStart());
        return idx >= start && idx < end;
    }

    void sanitizeTrajectoryVector(std::vector<double>& z, const Settings& cfg) const {
        const td::UINT4 nZ = static_cast<td::UINT4>(_layout.totalSize());
        const td::UINT4 deltaStart = static_cast<td::UINT4>(_layout.deltaStart());
        const td::UINT4 aStart = static_cast<td::UINT4>(_layout.aStart());
        const td::UINT4 slackStart = static_cast<td::UINT4>(_layout.slackStart());
        for (td::UINT4 i = 0; i < nZ; ++i) {
            if (isPsiIndex(i)) {
                z[i] = wrapAngle(z[i]);
            } else if (i >= slackStart) {
                z[i] = std::max(0.0, z[i]);
            } else if (i >= deltaStart && i < aStart) {
                z[i] = std::clamp(z[i], -cfg.steerLimit, cfg.steerLimit);
            } else if (i >= aStart) {
                z[i] = std::clamp(z[i], -cfg.accelLimit, cfg.accelLimit);
            }
        }
    }

    void sanitizeTrajectory(dense::DblMatrix& z, const Settings& cfg) const {
        if (z.getNoOfRows() * z.getNoOfCols() == 0) {
            return;
        }
        auto v = z.getColumnManipulator();
        const td::UINT4 nZ = static_cast<td::UINT4>(_layout.totalSize());
        const td::UINT4 deltaStart = static_cast<td::UINT4>(_layout.deltaStart());
        const td::UINT4 aStart = static_cast<td::UINT4>(_layout.aStart());
        const td::UINT4 slackStart = static_cast<td::UINT4>(_layout.slackStart());
        for (td::UINT4 i = 0; i < nZ; ++i) {
            if (isPsiIndex(i)) {
                v(i) = wrapAngle(v(i));
            } else if (i >= slackStart) {
                v(i) = std::max(0.0, v(i));
            } else if (i >= deltaStart && i < aStart) {
                v(i) = std::clamp(v(i), -cfg.steerLimit, cfg.steerLimit);
            } else if (i >= aStart) {
                v(i) = std::clamp(v(i), -cfg.accelLimit, cfg.accelLimit);
            }
        }
    }

    double evaluateObjective(const MpcCost& cost, const dense::DblMatrix& z) const {
        auto h = cost.H().getColumnManipulator();
        auto g = cost.G().getColumnManipulator();
        auto v = z.getColumnManipulator();
        const td::UINT4 total = static_cast<td::UINT4>(_layout.totalSize());
        double objective = 0.0;
        for (td::UINT4 i = 0; i < total; ++i) {
            const double zi = v(i);
            objective += 0.5 * h(i) * zi * zi + g(i) * zi;
        }
        return objective;
    }

    const MpcLayout& _layout;
    int _lastIterations = 0;
    double _lastMaxAbs = 0.0;
    bool _lastConverged = false;
};

} // namespace mpc
