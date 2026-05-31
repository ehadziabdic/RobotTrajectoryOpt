#pragma once
#include <dense/Matrix.h>
#include <sparse/IMatrix.h>
#include <mem/PointerReleaser.h>
#include <td/Types.h>
#include <iostream>
#include <vector>
#include <cmath>
#include "MpcLayout.h"
#include "MpcObstacle.h"

namespace mpc {

class MpcConstraints {
public:
    struct Triplet {
        td::INT4 row;
        td::INT4 col;
        double val;
    };

    explicit MpcConstraints(const MpcLayout& layout)
        : _layout(layout),
          _initial(4, 1, nullptr, true),
          _nominal(static_cast<td::UINT4>(4 * layout.N()), 1, nullptr, true) {
        assemble();
    }

        void setVerbose(bool verbose) { _verbose = verbose; }
        void setObstacles(const std::vector<Obstacle>& obstacles) { _obstacles = obstacles; }

    const MpcLayout& layout() const { return _layout; }
    sparse::IDblMatrix* matrix() { return _matrix.ptr(); }
    const sparse::IDblMatrix* matrix() const { return _matrix.ptr(); }
    const dense::DblMatrix& rhs() const { return _rhs; }
    const std::vector<Triplet>& triplets() const { return _triplets; }
    std::size_t rowCount() const { return _rowCount; }

    void setInitialState(const dense::DblMatrix& initial_state) {
        if (initial_state.getNoOfRows() * initial_state.getNoOfCols() < 4) {
            return;
        }
        auto init = initial_state.getColumnManipulator();
        auto dst = _initial.getColumnManipulator();
        dst(0) = init(0);
        dst(1) = init(1);
        dst(2) = init(2);
        dst(3) = init(3);

        if (_rhs.getNoOfRows() >= 4) {
            auto b = _rhs.getColumnManipulator();
            b(0) = dst(0);
            b(1) = dst(1);
            b(2) = dst(2);
            b(3) = dst(3);
        }
    }

    void UpdateNominalTrajectory(const dense::DblMatrix& nominal_state) {
        const std::size_t expected = 4 * _layout.N();
        if (nominal_state.getNoOfRows() * nominal_state.getNoOfCols() < expected) {
            return;
        }
        _nominal = nominal_state.makeCopy();
        assemble();
    }

private:
    void addTriplet(td::INT4 row, td::INT4 col, double val) {
        _matrix->addTriple(row, col, val);
        _triplets.push_back({row, col, val});
    }

    void assemble() {
        const std::size_t N = _layout.N();
        const std::size_t obstacleSlots = _layout.obstacleSlackPerStep();
        const std::size_t activeObstacles = std::min<std::size_t>(obstacleSlots, _obstacles.size());
        const std::size_t obstacleRowsPerStep = activeObstacles;
        const std::size_t obstacleRowCount = obstacleRowsPerStep * (N - 1);
        const std::size_t rows = 4 * N + obstacleRowCount;
        const std::size_t cols = _layout.totalSize();

        _rowCount = rows;
        _triplets.clear();

        const std::size_t nzEstimate = 4 + (N - 1) * 14 + obstacleRowCount * 3;

        _matrix = sparse::createDblMatrix(
            static_cast<int>(rows),
            static_cast<int>(cols),
            static_cast<int>(nzEstimate),
            sparse::Symmetry::NonSymmetric);

        if (!_matrix.ptr()) {
            if (_verbose) {
                std::cout << "MPC constraints: createDblMatrix returned null" << std::endl;
            }
            return;
        }

        _rhs = dense::DblMatrix(static_cast<td::UINT4>(rows), 1, nullptr, true);
        auto b = _rhs.getColumnManipulator();
        auto init = _initial.getColumnManipulator();
        auto nom = _nominal.getColumnManipulator();

        // Initial state lock (rows 0..3)
        addTriplet(0, static_cast<td::INT4>(_layout.idxX(0)), 1.0);
        addTriplet(1, static_cast<td::INT4>(_layout.idxY(0)), 1.0);
        addTriplet(2, static_cast<td::INT4>(_layout.idxPsi(0)), 1.0);
        addTriplet(3, static_cast<td::INT4>(_layout.idxV(0)), 1.0);

        b(0) = init(0);
        b(1) = init(1);
        b(2) = init(2);
        b(3) = init(3);

        std::size_t row = 4;
        for (std::size_t t = 0; t < N - 1; ++t) {
            const double psi = nom(static_cast<td::UINT4>(_layout.idxPsi(t)));
            const double vnom = nom(static_cast<td::UINT4>(_layout.idxV(t)));
            const double cPsi = std::cos(psi);
            const double sPsi = std::sin(psi);

            // x_{t+1} - x_t - v_t * cos(psi_t) * dt = 0
            addTriplet(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxX(t + 1)), 1.0);
            addTriplet(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxX(t)), -1.0);
            addTriplet(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxV(t)), -_layout.dt() * cPsi);
            addTriplet(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxPsi(t)), _layout.dt() * vnom * sPsi);
            b(static_cast<td::UINT4>(row)) = vnom * sPsi * _layout.dt() * psi;
            ++row;

            // y_{t+1} - y_t - v_t * sin(psi_t) * dt = 0
            addTriplet(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxY(t + 1)), 1.0);
            addTriplet(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxY(t)), -1.0);
            addTriplet(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxV(t)), -_layout.dt() * sPsi);
            addTriplet(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxPsi(t)), -_layout.dt() * vnom * cPsi);
            b(static_cast<td::UINT4>(row)) = -vnom * cPsi * _layout.dt() * psi;
            ++row;

            // psi_{t+1} - psi_t - (dt/Lf) * delta_t = 0
            addTriplet(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxPsi(t + 1)), 1.0);
            addTriplet(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxPsi(t)), -1.0);
            addTriplet(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxDelta(t)), -_layout.dt() / _layout.Lf());
            b(static_cast<td::UINT4>(row)) = 0.0;
            ++row;

            // v_{t+1} - v_t - a_t * dt = 0
            addTriplet(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxV(t + 1)), 1.0);
            addTriplet(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxV(t)), -1.0);
            addTriplet(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxA(t)), -_layout.dt());
            b(static_cast<td::UINT4>(row)) = 0.0;
            ++row;

            for (std::size_t obsIdx = 0; obsIdx < obstacleSlots && obsIdx < _obstacles.size(); ++obsIdx) {
                const auto& obstacle = _obstacles[obsIdx];
                const double safeRadius = obstacle.r + 0.15;
                const double xNom = nom(static_cast<td::UINT4>(_layout.idxX(t + 1)));
                const double yNom = nom(static_cast<td::UINT4>(_layout.idxY(t + 1)));
                const double dx = xNom - obstacle.x;
                const double dy = yNom - obstacle.y;
                const double dist2 = dx * dx + dy * dy;
                const double violation = safeRadius * safeRadius - dist2;

                const double gradX = 2.0 * dx;
                const double gradY = 2.0 * dy;
                const td::UINT4 slackIdx = static_cast<td::UINT4>(_layout.idxSlack(t + 1, obsIdx));

                if (violation <= 0.0) {
                    addTriplet(static_cast<td::INT4>(row), static_cast<td::INT4>(slackIdx), 1.0);
                    b(static_cast<td::UINT4>(row)) = 0.0;
                    ++row;
                    continue;
                }

                // Soft obstacle row: linearized violation with positive slack.
                addTriplet(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxX(t + 1)), gradX);
                addTriplet(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxY(t + 1)), gradY);
                addTriplet(static_cast<td::INT4>(row), static_cast<td::INT4>(slackIdx), 1.0);

                b(static_cast<td::UINT4>(row)) = violation + gradX * xNom + gradY * yNom;
                ++row;
            }
        }

        if (_verbose) {
            std::cout << "MPC constraints: rows=" << rows
                      << " cols=" << cols
                      << " nz_est=" << nzEstimate
                      << " nz_actual=" << _matrix->getNoOfNonZero()
                      << "\n";
        }
    }

    MpcLayout _layout;
    dense::DblMatrix _initial;
    dense::DblMatrix _nominal;
    std::vector<Triplet> _triplets;
    sparse::DblMatrixReleaser _matrix;
    dense::DblMatrix _rhs;
    std::vector<Obstacle> _obstacles;
    std::size_t _rowCount = 0;
    bool _verbose = false;
};

} // namespace mpc
