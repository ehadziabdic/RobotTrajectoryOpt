#pragma once
#include <dense/Matrix.h>
#include <td/Types.h>
#include <cstddef>
#include <iostream>
#include <limits>
#include <vector>
#include "MpcLayout.h"
#include "MpcCost.h"
#include "MpcSqp.h"
#include "MpcObstacle.h"

namespace mpc {

struct Telemetry {
    double x = 0.0;
    double y = 0.0;
    double psi = 0.0;
    double v = 0.0;
};

struct Trajectory {
    dense::DblMatrix states;   // 4 x N
    dense::DblMatrix controls; // 2 x (N - 1)
    std::size_t N = 0;
};

class MpcEngine {
public:
    struct Diagnostics {
        bool ok = false;
        bool converged = false;
        int iterations = 0;
        double maxAbs = 0.0;
    };

    explicit MpcEngine(const MpcLayout& layout)
        : _layout(layout),
          _sqp(_layout),
          _zNom(static_cast<td::UINT4>(_layout.totalSize()), 1, nullptr, true) {}

    void setSettings(const MpcSqp::Settings& settings) { _settings = settings; }

    void reset() {
        const td::UINT4 totalSize = static_cast<td::UINT4>(_layout.totalSize());
        _zNom = dense::DblMatrix(totalSize, 1, nullptr, true);
        _zNomInitialized = false;
        _sqp.reset();
        _diag = Diagnostics{};
    }

    const Diagnostics& diagnostics() const { return _diag; }

    bool Solve(const Telemetry& current,
               const dense::DblMatrix& coeffs,
               double target_v,
               double dt,
               const std::vector<Obstacle>& obstacles,
               Trajectory& out,
               bool freezeAtPeak = false,
               double maxLookahead = 15.0,
               double trackLength = std::numeric_limits<double>::max()) {
        if (dt <= 0.0) {
            std::cout << "MpcEngine: invalid dt" << std::endl;
            _diag = Diagnostics{};
            return false;
        }
        if (coeffs.getNoOfRows() == 0 || coeffs.getNoOfCols() == 0) {
            std::cout << "MpcEngine: empty coeffs" << std::endl;
            _diag = Diagnostics{};
            return false;
        }

        // Force fresh reference regeneration every step.  Carrying over the
        // previous step's SQP output as warm-start causes x-position drift:
        // z(1..N-1) have x-positions from the old step, making z(0)≈z(1)
        // which violates the dynamics constraint.  The SQP then needs many
        // iterations to push the trajectory forward, and eventually fails.
        // Always regenerating from the polynomial at the current vehicle
        // position keeps the warm-start aligned and the SQP converges fast.
        _zNomInitialized = false;

        if (!ensureNominalInitialized(coeffs, target_v, current.x, current.y, current.psi, dt, freezeAtPeak, maxLookahead, trackLength, current.v)) {
            std::cout << "MpcEngine: nominal init failed" << std::endl;
            _diag = Diagnostics{};
            return false;
        }

        dense::DblMatrix zWork = _zNom.makeCopy();

        // Snap warm-start t=0 to the current vehicle state so the SQP
        // always starts from the correct position.
        // z(1..N-1) stays on the polynomial reference — this is the correct
        // warm-start since the reference is what the solver should track.
        {
            auto zw = zWork.getColumnManipulator();
            zw(static_cast<td::UINT4>(_layout.idxX(0)))   = current.x;
            zw(static_cast<td::UINT4>(_layout.idxY(0)))   = current.y;
            zw(static_cast<td::UINT4>(_layout.idxPsi(0))) = current.psi;
            zw(static_cast<td::UINT4>(_layout.idxV(0)))   = current.v;
        }

        if (_settings.verbose) {
            const td::UINT4 nZ = static_cast<td::UINT4>(zWork.getNoOfRows());
            auto zw = zWork.getColumnManipulator();
            std::cout << "MpcEngine: launching SQP with zWork size=" << nZ << " first_vals=[";
            const td::UINT4 dump = (nZ > 8) ? 8u : nZ;
            for (td::UINT4 i = 0; i < dump; ++i) {
                double v = zw(i);
                if (!std::isfinite(v)) std::cout << "nan";
                else std::cout << v;
                if (i + 1 < dump) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }

        const bool ok = _sqp.Solve(coeffs,
                       target_v,
                       current.x,
                       current.y,
                       dt,
                       _settings,
                       obstacles,
                       zWork,
                       current.x,
                       current.y,
                       current.psi,
                       current.v,
                       current.psi,
                       freezeAtPeak,
                       maxLookahead,
                       trackLength);
        _diag.ok = ok;
        _diag.converged = _sqp.lastConverged();
        _diag.iterations = _sqp.lastIterations();
        _diag.maxAbs = _sqp.lastMaxAbs();

        if (!ok) {
            // SQP failed (KKT factorization issue).  Rather than freezing
            // the vehicle, fall back to the reference trajectory feedforward
            // controls so the vehicle keeps moving along the polynomial.
            _zNomInitialized = false;
            extractTrajectory(out);
            return true;
        }

        // Accept the converged warm-start if the SQP update was reasonable.
        // If maxAbs is very large, the SQP diverged; fall back to a fresh
        // reference-trajectory init on the next step to escape the bad basin.
        if (_sqp.lastMaxAbs() < 5.0) {
            _zNom = zWork;
        } else {
            _zNomInitialized = false;
        }

        // Detect stagnation: SQP converged in very few iterations with
        // near-zero update. This means the warm-start trajectory has become
        // a fixed point of the SQP — the dynamics constraints are satisfied
        // by the stale trajectory so the QP returns it unchanged, even though
        // it no longer tracks the current reference.  Force a fresh
        // reference-trajectory init on the next step to break the cycle.
        if (_diag.converged && _diag.iterations <= 3 && _diag.maxAbs < 1e-6) {
            _zNomInitialized = false;
        }

        extractTrajectory(out);
        return true;
    }

private:
    bool ensureNominalInitialized(const dense::DblMatrix& coeffs,
                                  double target_v,
                                  double initial_x,
                                  double initial_y,
                                  double initial_psi,
                                  double dt,
                                  bool freezeAtPeak,
                                  double maxLookahead,
                                  double trackLength,
                                  double projection_v = -1.0) {
        if (_zNom.getNoOfRows() == 0) {
            return false;
        }
        const td::UINT4 totalSize = static_cast<td::UINT4>(_layout.totalSize());
        if (_zNom.getNoOfRows() != totalSize || _zNom.getNoOfCols() != 1) {
            _zNom = dense::DblMatrix(totalSize, 1, nullptr, true);
            _zNomInitialized = false;
        }
        if (!_zNomInitialized) {
            MpcCost cost(_layout);
            cost.setFreezeAtPeak(freezeAtPeak);
            cost.setMaxLookahead(maxLookahead);
            cost.setTrackLength(trackLength);
            // Pass actual vehicle velocity for spatial projection so warm-start
            // reference already accounts for the true travel speed
            cost.UpdateReferenceTrajectory(coeffs, target_v, initial_x, initial_y, dt, initial_psi, projection_v);
            _zNom = cost.Ref().makeCopy();
            _zNomInitialized = true;
        }
        return true;
    }

    void extractTrajectory(Trajectory& out) const {
        const std::size_t N = _layout.N();
        out.N = N;
        const std::size_t controlCols = (N > 0) ? (N - 1) : 0;
        out.states = dense::DblMatrix(4, static_cast<td::UINT4>(N), nullptr, true);
        out.controls = dense::DblMatrix(2, static_cast<td::UINT4>(controlCols), nullptr, true);

        auto z = _zNom.getColumnManipulator();
        auto s = out.states.getManipulator();
        auto u = out.controls.getManipulator();

        for (std::size_t t = 0; t < N; ++t) {
            s(0, static_cast<td::UINT4>(t)) = z(static_cast<td::UINT4>(_layout.idxX(t)));
            s(1, static_cast<td::UINT4>(t)) = z(static_cast<td::UINT4>(_layout.idxY(t)));
            s(2, static_cast<td::UINT4>(t)) = z(static_cast<td::UINT4>(_layout.idxPsi(t)));
            s(3, static_cast<td::UINT4>(t)) = z(static_cast<td::UINT4>(_layout.idxV(t)));
        }

        for (std::size_t t = 0; t < controlCols; ++t) {
            u(0, static_cast<td::UINT4>(t)) = z(static_cast<td::UINT4>(_layout.idxDelta(t)));
            u(1, static_cast<td::UINT4>(t)) = z(static_cast<td::UINT4>(_layout.idxA(t)));
        }
    }

    const MpcLayout& _layout;
    MpcSqp _sqp;
    MpcSqp::Settings _settings{};
    dense::DblMatrix _zNom;
    bool _zNomInitialized = false;
    Diagnostics _diag{};
};

} // namespace mpc
