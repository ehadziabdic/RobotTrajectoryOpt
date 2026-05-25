#pragma once
#include <dense/Matrix.h>
#include <td/Types.h>
#include <cmath>
#include "MpcLayout.h"

namespace mpc {

class MpcCost {
public:
    explicit MpcCost(const MpcLayout& layout)
        : _layout(layout),
          Q(4, 1, nullptr, true),
          R(2, 1, nullptr, true),
          Hdiag(static_cast<td::UINT4>(_layout.totalSize()), 1, nullptr, true),
          g(static_cast<td::UINT4>(_layout.totalSize()), 1, nullptr, true),
          Zref(static_cast<td::UINT4>(_layout.totalSize()), 1, nullptr, true) {
        auto q = Q.getColumnManipulator();
        q(0) = 1.0; // qx
        q(1) = 1.0; // qy
        q(2) = 1.0; // qpsi
        q(3) = 1.0; // qv

        auto r = R.getColumnManipulator();
        r(0) = 1.0; // r_delta
        r(1) = 1.0; // r_a

        buildHdiag();
    }

    const MpcLayout& layout() const { return _layout; }

    void UpdateReferenceTrajectory(const dense::DblMatrix& coeffs, double target_v, double initial_x, double dt) {
        double c0 = 0.0;
        double c1 = 0.0;
        double c2 = 0.0;
        double c3 = 0.0;

        auto cmat = coeffs.getManipulator();
        const td::UINT4 r = coeffs.getNoOfRows();
        const td::UINT4 c = coeffs.getNoOfCols();
        if (r >= 4) {
            c0 = cmat(0, 0);
            c1 = cmat(1, 0);
            c2 = cmat(2, 0);
            c3 = cmat(3, 0);
        } else if (c >= 4) {
            c0 = cmat(0, 0);
            c1 = cmat(0, 1);
            c2 = cmat(0, 2);
            c3 = cmat(0, 3);
        }

        auto z = Zref.getColumnManipulator();
        auto h = Hdiag.getColumnManipulator();
        auto gg = g.getColumnManipulator();

        const std::size_t N = _layout.N();
        for (std::size_t t = 0; t < N; ++t) {
            const double xref = initial_x + target_v * static_cast<double>(t) * dt;
            const double yref = c0 + c1 * xref + c2 * xref * xref + c3 * xref * xref * xref;
            const double psiref = std::atan(c1 + 2.0 * c2 * xref + 3.0 * c3 * xref * xref);
            const double vref = target_v;

            z(static_cast<td::UINT4>(_layout.idxX(t))) = xref;
            z(static_cast<td::UINT4>(_layout.idxY(t))) = yref;
            z(static_cast<td::UINT4>(_layout.idxPsi(t))) = psiref;
            z(static_cast<td::UINT4>(_layout.idxV(t))) = vref;
        }

        for (std::size_t t = 0; t < N - 1; ++t) {
            z(static_cast<td::UINT4>(_layout.idxDelta(t))) = 0.0;
            z(static_cast<td::UINT4>(_layout.idxA(t))) = 0.0;
        }

        const td::UINT4 total = static_cast<td::UINT4>(_layout.totalSize());
        for (td::UINT4 i = 0; i < total; ++i) {
            gg(i) = -h(i) * z(i);
        }
    }

    const dense::DblMatrix& H() const { return Hdiag; }
    const dense::DblMatrix& G() const { return g; }
    const dense::DblMatrix& Ref() const { return Zref; }

    dense::DblMatrix Q;
    dense::DblMatrix R;

private:
    void buildHdiag() {
        auto h = Hdiag.getColumnManipulator();
        auto q = Q.getColumnManipulator();
        auto r = R.getColumnManipulator();

        const std::size_t N = _layout.N();
        for (std::size_t t = 0; t < N; ++t) {
            h(static_cast<td::UINT4>(_layout.idxX(t))) = q(0);
            h(static_cast<td::UINT4>(_layout.idxY(t))) = q(1);
            h(static_cast<td::UINT4>(_layout.idxPsi(t))) = q(2);
            h(static_cast<td::UINT4>(_layout.idxV(t))) = q(3);
        }
        for (std::size_t t = 0; t < N - 1; ++t) {
            h(static_cast<td::UINT4>(_layout.idxDelta(t))) = r(0);
            h(static_cast<td::UINT4>(_layout.idxA(t))) = r(1);
        }
    }

    MpcLayout _layout;
    dense::DblMatrix Hdiag;
    dense::DblMatrix g;
    dense::DblMatrix Zref;
};

} // namespace mpc
