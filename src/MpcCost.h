#pragma once
#include <dense/Matrix.h>
#include <td/Types.h>
#include <cmath>
#include "MpcLayout.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace mpc {

class MpcCost {
public:
    explicit MpcCost(const MpcLayout& layout)
        : _layout(layout),
          Q(4, 1, nullptr, true),
          R(2, 1, nullptr, true),
          slackWeight(8000.0),
          Hdiag(static_cast<td::UINT4>(_layout.totalSize()), 1, nullptr, true),
          g(static_cast<td::UINT4>(_layout.totalSize()), 1, nullptr, true),
          Zref(static_cast<td::UINT4>(_layout.totalSize()), 1, nullptr, true) {
        auto q = Q.getColumnManipulator();
        q(0) = 0.5; // qx
        q(1) = 8.0; // qy
        q(2) = 5.0; // qpsi
        q(3) = 1.0; // qv

        auto r = R.getColumnManipulator();
        r(0) = 0.5; // r_delta
        r(1) = 0.5; // r_a

        buildHdiag();
    }

    void setFreezeAtPeak(bool v) { _freezeAtPeak = v; }
    void setMaxLookahead(double v) { _maxLookahead = v; }
    bool freezeAtPeak() const { return _freezeAtPeak; }
    double maxLookahead() const { return _maxLookahead; }

    const MpcLayout& layout() const { return _layout; }

    void UpdateReferenceTrajectory(const dense::DblMatrix& coeffs, double target_v, double initial_x, double initial_y, double dt, double initial_psi) {
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

        // Find closest x on polynomial y(x) to current (initial_x, initial_y)
        auto y_at = [&](double x) {
            return c0 + c1 * x + c2 * x * x + c3 * x * x * x;
        };
        auto dy_at = [&](double x) {
            return c1 + 2.0 * c2 * x + 3.0 * c3 * x * x;
        };
        auto ddy_at = [&](double x) {
            return 2.0 * c2 + 6.0 * c3 * x;
        };

        double x_closest = initial_x;
        // Newton iterations to find root of f(x) = (x - x0) + (y(x)-y0)*y'(x)
        for (int it = 0; it < 12; ++it) {
            const double yx = y_at(x_closest);
            const double dy = dy_at(x_closest);
            const double ddy = ddy_at(x_closest);
            const double f = (x_closest - initial_x) + (yx - initial_y) * dy;
            const double fp = 1.0 + dy * dy + (yx - initial_y) * ddy;
            if (std::fabs(fp) < 1e-8) break;
            const double dx = f / fp;
            x_closest -= dx;
            if (std::fabs(dx) < 1e-8) break;
        }

        // Ensure k=0 (initial) reference values preserve vehicle telemetry
        const double y0 = initial_y;
        const double x0 = initial_x;
        const double psi0 = std::atan(dy_at(x_closest));
        const double v0 = target_v;

        z(static_cast<td::UINT4>(_layout.idxX(0))) = x0;
        z(static_cast<td::UINT4>(_layout.idxY(0))) = y0;
        z(static_cast<td::UINT4>(_layout.idxPsi(0))) = psi0;
        z(static_cast<td::UINT4>(_layout.idxV(0))) = v0;

        // Step C2: Replace any existing max_path_x computation with:
        const double max_path_x = x_closest + _maxLookahead;

        // Step C3: Before the reference trajectory loop, initialize psiref_prev and normalize
        double psiref_prev = std::atan2(dy_at(x_closest), 1.0);
        while (psiref_prev - initial_psi >  M_PI) psiref_prev -= 2.0 * M_PI;
        while (psiref_prev - initial_psi < -M_PI) psiref_prev += 2.0 * M_PI;

        // Step C4: If _freezeAtPeak is true and c3 < 0 and c2 > 0, compute before the loop:
        double x_peak = 0.0;
        if (_freezeAtPeak && c3 < 0.0 && c2 > 0.0) {
            x_peak = -2.0 * c2 / (3.0 * c3);
        }

        // Build forward reference trajectory from the closest point, but keep k=0 locked to telemetry
        double xref = x_closest;
        double psiref = std::atan2(dy_at(xref), 1.0);
        const double vrefConst = target_v;

        for (std::size_t t = 1; t < N; ++t) {
            // Step forward using heading-aware increments
            xref += vrefConst * std::cos(psiref) * dt;

            // Step C6: Inside the loop, after advancing xref, add freeze logic
            if (_freezeAtPeak && c3 < 0.0 && c2 > 0.0 && xref >= x_peak) {
                xref = x_peak;
            }

            if (xref > max_path_x) {
                xref = max_path_x;
            }

            // Recompute reference heading from polynomial slope at new x
            psiref = std::atan2(dy_at(xref), 1.0);

            // Step C5: Inside the loop, after computing psiref = std::atan2(dy_at(xref), 1.0):
            while (psiref - psiref_prev >  M_PI) psiref -= 2.0 * M_PI;
            while (psiref - psiref_prev < -M_PI) psiref += 2.0 * M_PI;
            psiref_prev = psiref;

            // Step C7: Apply y-floor on yref:
            const double yref = y_at(xref);

            z(static_cast<td::UINT4>(_layout.idxX(t))) = xref;
            z(static_cast<td::UINT4>(_layout.idxY(t))) = yref;
            z(static_cast<td::UINT4>(_layout.idxPsi(t))) = psiref;
            z(static_cast<td::UINT4>(_layout.idxV(t))) = vrefConst;
        }

        for (std::size_t t = 0; t < N - 1; ++t) {
            z(static_cast<td::UINT4>(_layout.idxDelta(t))) = 0.0;
            z(static_cast<td::UINT4>(_layout.idxA(t))) = 0.0;
        }

        for (std::size_t t = 0; t < N; ++t) {
            for (std::size_t obs = 0; obs < _layout.obstacleSlackPerStep(); ++obs) {
                z(static_cast<td::UINT4>(_layout.idxSlack(t, obs))) = 0.0;
            }
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

        for (std::size_t t = 0; t < N; ++t) {
            for (std::size_t obs = 0; obs < _layout.obstacleSlackPerStep(); ++obs) {
                h(static_cast<td::UINT4>(_layout.idxSlack(t, obs))) = slackWeight;
            }
        }
    }

    MpcLayout _layout;
    dense::DblMatrix Hdiag;
    dense::DblMatrix g;
    dense::DblMatrix Zref;
    double slackWeight;
    bool _freezeAtPeak = false;
    double _maxLookahead = 15.0;
};

} // namespace mpc
