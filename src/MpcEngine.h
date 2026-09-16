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
            std::cout << "MpcEngine: invalid dt\n";
            _diag = Diagnostics{};
            return false;
        }
        if (coeffs.getNoOfRows() == 0 || coeffs.getNoOfCols() == 0) {
            std::cout << "MpcEngine: empty coeffs\n";
            _diag = Diagnostics{};
            return false;
        }

        // Force fresh reference regeneration every step to prevent warm-start drift.
        _zNomInitialized = false;

        if (!ensureNominalInitialized(coeffs, target_v, current.x, current.y, current.psi, dt, freezeAtPeak, maxLookahead, trackLength, current.v)) {
            std::cout << "MpcEngine: nominal init failed\n";
            _diag = Diagnostics{};
            return false;
        }

        dense::DblMatrix zWork = _zNom.makeCopy();

        // Snap warm-start t=0 to the current vehicle state.
        {
            auto zw = zWork.getColumnManipulator();
            zw(static_cast<td::UINT4>(_layout.idxX(0)))   = current.x;
            zw(static_cast<td::UINT4>(_layout.idxY(0)))   = current.y;
            zw(static_cast<td::UINT4>(_layout.idxPsi(0))) = current.psi;
            zw(static_cast<td::UINT4>(_layout.idxV(0)))   = current.v;
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
            // SQP failed — fall back to reference feedforward controls.
            _zNomInitialized = false;
            extractTrajectory(out);
            return true;
        }

        // Accept the converged warm-start if the SQP update was reasonable.
        if (_sqp.lastMaxAbs() < 5.0) {
            _zNom = zWork;
        } else {
            _zNomInitialized = false;
        }

        // Detect stagnation: SQP converged in very few iterations with
        // near-zero update. Force fresh reference on next step.
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
