#pragma once
#include <dense/Matrix.h>
#include <td/Types.h>
#include <cstddef>
#include <iostream>
#include "MpcLayout.h"
#include "MpcCost.h"
#include "MpcSqp.h"

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
    explicit MpcEngine(const MpcLayout& layout)
        : _layout(layout),
          _sqp(_layout),
          _zNom(static_cast<td::UINT4>(_layout.totalSize()), 1, nullptr, true) {}

    void setSettings(const MpcSqp::Settings& settings) { _settings = settings; }

    bool Solve(const Telemetry& current,
               const dense::DblMatrix& coeffs,
               double target_v,
               double dt,
               Trajectory& out) {
        if (dt <= 0.0) {
            std::cout << "MpcEngine: invalid dt" << std::endl;
            return false;
        }
        if (coeffs.getNoOfRows() == 0 || coeffs.getNoOfCols() == 0) {
            std::cout << "MpcEngine: empty coeffs" << std::endl;
            return false;
        }
        if (!ensureNominalInitialized(coeffs, target_v, current.x, dt)) {
            std::cout << "MpcEngine: nominal init failed" << std::endl;
            return false;
        }

        dense::DblMatrix zWork = _zNom.makeCopy();
        if (_settings.log) {
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

        if (!_sqp.Solve(coeffs,
                        target_v,
                        current.x,
                        dt,
                        _settings,
                        zWork,
                        current.x,
                        current.y,
                        current.psi,
                        current.v)) {
            std::cout << "MpcEngine: SQP solve failed" << std::endl;
            return false;
        }

        _zNom = zWork;

        extractTrajectory(out);
        return true;
    }

private:
    bool ensureNominalInitialized(const dense::DblMatrix& coeffs,
                                  double target_v,
                                  double initial_x,
                                  double dt) {
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
            cost.UpdateReferenceTrajectory(coeffs, target_v, initial_x, dt);
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
};

} // namespace mpc
