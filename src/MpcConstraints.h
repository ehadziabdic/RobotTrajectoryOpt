#pragma once
#include <dense/Matrix.h>
#include <sparse/IMatrix.h>
#include <mem/PointerReleaser.h>
#include <td/Types.h>
#include <iostream>
#include "MpcLayout.h"

namespace mpc {

class MpcConstraints {
public:
    explicit MpcConstraints(const MpcLayout& layout)
        : _layout(layout) {
        assemble();
    }

    const MpcLayout& layout() const { return _layout; }
    sparse::IDblMatrix* matrix() { return _matrix.ptr(); }
    const sparse::IDblMatrix* matrix() const { return _matrix.ptr(); }
    const dense::DblMatrix& rhs() const { return _rhs; }

    void setInitialState(const dense::DblMatrix& initial_state) {
        if (initial_state.getNoOfRows() * initial_state.getNoOfCols() < 4) {
            return;
        }
        auto init = initial_state.getColumnManipulator();
        auto b = _rhs.getColumnManipulator();
        b(0) = init(0);
        b(1) = init(1);
        b(2) = init(2);
        b(3) = init(3);
    }

private:
    void assemble() {
        const std::size_t N = _layout.N();
        const std::size_t rows = 4 * N;
        const std::size_t cols = _layout.totalSize();

        const std::size_t nzEstimate = 4 + (N - 1) * 12;

        _matrix = sparse::createDblMatrix(
            static_cast<int>(rows),
            static_cast<int>(cols),
            static_cast<int>(nzEstimate),
            sparse::Symmetry::NonSymmetric);

        if (!_matrix.ptr()) {
            std::cout << "MPC constraints: createDblMatrix returned null" << std::endl;
            return;
        }

        std::cout << "MPC constraints: addTriples start" << std::endl;

        _rhs = dense::DblMatrix(static_cast<td::UINT4>(rows), 1, nullptr, true);
        auto b = _rhs.getColumnManipulator();

        // Initial state lock (rows 0..3)
        _matrix->addTriple(0, static_cast<td::INT4>(_layout.idxX(0)), 1.0);
        _matrix->addTriple(1, static_cast<td::INT4>(_layout.idxY(0)), 1.0);
        _matrix->addTriple(2, static_cast<td::INT4>(_layout.idxPsi(0)), 1.0);
        _matrix->addTriple(3, static_cast<td::INT4>(_layout.idxV(0)), 1.0);

        b(0) = 0.0;
        b(1) = 0.0;
        b(2) = 0.0;
        b(3) = 0.0;

        const double cPsi = 1.0; // cos(0)
        const double sPsi = 0.0; // sin(0)
        std::size_t row = 4;
        for (std::size_t t = 0; t < N - 1; ++t) {
            // x_{t+1} - x_t - v_t * cos(psi) * dt = 0
            _matrix->addTriple(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxX(t + 1)), 1.0);
            _matrix->addTriple(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxX(t)), -1.0);
            _matrix->addTriple(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxV(t)), -_layout.dt() * cPsi);
            b(static_cast<td::UINT4>(row)) = 0.0;
            ++row;

            // y_{t+1} - y_t - v_t * sin(psi) * dt = 0
            _matrix->addTriple(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxY(t + 1)), 1.0);
            _matrix->addTriple(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxY(t)), -1.0);
            _matrix->addTriple(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxV(t)), -_layout.dt() * sPsi);
            b(static_cast<td::UINT4>(row)) = 0.0;
            ++row;

            // psi_{t+1} - psi_t - (dt/Lf) * delta_t = 0 (linearized v=1)
            _matrix->addTriple(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxPsi(t + 1)), 1.0);
            _matrix->addTriple(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxPsi(t)), -1.0);
            _matrix->addTriple(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxDelta(t)), -_layout.dt() / _layout.Lf());
            b(static_cast<td::UINT4>(row)) = 0.0;
            ++row;

            // v_{t+1} - v_t - a_t * dt = 0
            _matrix->addTriple(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxV(t + 1)), 1.0);
            _matrix->addTriple(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxV(t)), -1.0);
            _matrix->addTriple(static_cast<td::INT4>(row), static_cast<td::INT4>(_layout.idxA(t)), -_layout.dt());
            b(static_cast<td::UINT4>(row)) = 0.0;
            ++row;
        }

        std::cout << "MPC constraints: rows=" << rows
                  << " cols=" << cols
                  << " nz_est=" << nzEstimate
                  << " nz_actual=" << _matrix->getNoOfNonZero()
                  << "\n";
    }

    MpcLayout _layout;
    sparse::DblMatrixReleaser _matrix;
    dense::DblMatrix _rhs;
};

} // namespace mpc
