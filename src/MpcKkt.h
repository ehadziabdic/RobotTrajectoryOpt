#pragma once
#include <sparse/IMatrix.h>
#include <mem/PointerReleaser.h>
#include <dense/Matrix.h>
#include <td/Types.h>
#include <iostream>
#include <vector>
#include "MpcLayout.h"
#include "MpcCost.h"
#include "MpcConstraints.h"

namespace mpc {

class MpcKkt {
public:
    struct Triplet {
        td::INT4 row;
        td::INT4 col;
        double val;
    };

    MpcKkt(const MpcLayout& layout, const MpcCost& cost, const MpcConstraints& constraints)
        : _layout(layout), _cost(cost), _constraints(constraints) {}

    void Assemble() {
        const std::size_t nZ = _layout.totalSize();
        _nZ = static_cast<td::UINT4>(nZ);
        const std::size_t nC = _constraints.rowCount();
        const std::size_t kktSize = nZ + nC;

        const std::size_t nnzEstimate = 658;
        _kkt = sparse::createDblMatrix(
            static_cast<int>(kktSize),
            static_cast<int>(kktSize),
            static_cast<int>(nnzEstimate),
            sparse::Symmetry::NonSymmetric);

        if (!_kkt.ptr()) {
            std::cout << "MPC KKT: createDblMatrix returned null" << std::endl;
            return;
        }

        auto h = _cost.H().getColumnManipulator();
        auto g = _cost.G().getColumnManipulator();

        _rhs = dense::DblMatrix(static_cast<td::UINT4>(kktSize), 1, nullptr, true);
        auto rhs = _rhs.getColumnManipulator();

        _triplets.clear();

        for (std::size_t i = 0; i < nZ; ++i) {
            const td::INT4 idx = static_cast<td::INT4>(i);
            const double val = h(static_cast<td::UINT4>(i));
            _kkt->addTriple(idx, idx, val);
            _triplets.push_back({idx, idx, val});
            rhs(static_cast<td::UINT4>(i)) = -g(static_cast<td::UINT4>(i));
        }

        const sparse::IDblMatrix* A = _constraints.matrix();
        if (!A) {
            std::cout << "MPC KKT: constraints matrix is null" << std::endl;
            return;
        }

        const auto& trips = _constraints.triplets();
        for (const auto& t : trips) {
            const td::INT4 rowLower = static_cast<td::INT4>(nZ) + t.row;
            const td::INT4 colLower = t.col;
            _kkt->addTriple(rowLower, colLower, t.val);
            _triplets.push_back({rowLower, colLower, t.val});

            const td::INT4 rowUpper = t.col;
            const td::INT4 colUpper = static_cast<td::INT4>(nZ) + t.row;
            _kkt->addTriple(rowUpper, colUpper, t.val);
            _triplets.push_back({rowUpper, colUpper, t.val});
        }

        auto b = _constraints.rhs().getColumnManipulator();
        for (std::size_t i = 0; i < nC; ++i) {
            rhs(static_cast<td::UINT4>(nZ + i)) = b(static_cast<td::UINT4>(i));
        }
    }

    // Number of primal variables (z) contained in the KKT system
    td::UINT4 primalSize() const { return _nZ; }

    sparse::IDblMatrix* matrix() { return _kkt.ptr(); }
    const sparse::IDblMatrix* matrix() const { return _kkt.ptr(); }
    const dense::DblMatrix& rhs() const { return _rhs; }
    const std::vector<Triplet>& triplets() const { return _triplets; }

private:
    const MpcLayout& _layout;
    const MpcCost& _cost;
    const MpcConstraints& _constraints;
    std::vector<Triplet> _triplets;
    sparse::DblMatrixReleaser _kkt;
    dense::DblMatrix _rhs;
    td::UINT4 _nZ = 0;
};

} // namespace mpc
