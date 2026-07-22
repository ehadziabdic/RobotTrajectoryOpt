#pragma once
#include <dense/Matrix.h>
#include <td/Types.h>
#include <cmath>
#include <limits>
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
        q(0) = 0.5;  // qx
        q(1) = 50.0; // qy - strong lateral tracking (was drowning under qv domination)
        q(2) = 25.0; // qpsi - strong heading alignment to prevent overshoot
        q(3) = 1.0;  // qv - low: let solver use velocity for tracking rather than rigid vref

        auto r = R.getColumnManipulator();
        r(0) = 0.5; // r_delta
        r(1) = 0.5; // r_a

        buildHdiag();
    }

    void setFreezeAtPeak(bool v) { _freezeAtPeak = v; }
    void setMaxLookahead(double v) { _maxLookahead = v; }
    void setTrackLength(double v) { _trackLength = v; }
    bool freezeAtPeak() const { return _freezeAtPeak; }
    double maxLookahead() const { return _maxLookahead; }
    double trackLength() const { return _trackLength; }

    const MpcLayout& layout() const { return _layout; }

    // projection_v: velocity used to advance xref along the curve.
    // When < 0, target_v is used (default).
    void UpdateReferenceTrajectory(const dense::DblMatrix& coeffs, double target_v, double initial_x, double initial_y, double dt, double initial_psi, double projection_v = -1.0) {
        // Read up to 6 coefficients (quintic).
        double c0 = 0.0, c1 = 0.0, c2 = 0.0, c3 = 0.0, c4 = 0.0, c5 = 0.0;

        auto cmat = coeffs.getManipulator();
        const td::UINT4 r = coeffs.getNoOfRows();
        const td::UINT4 nc = coeffs.getNoOfCols();
        // Support both 6x1 column vector and 1x6 row vector
        const td::UINT4 maxC = (r > 1) ? r : nc;
        auto read = [&](td::UINT4 i) -> double {
            return (r > 1) ? cmat(i, 0) : cmat(0, i);
        };
        if (maxC >= 1) c0 = read(0);
        if (maxC >= 2) c1 = read(1);
        if (maxC >= 3) c2 = read(2);
        if (maxC >= 4) c3 = read(3);
        if (maxC >= 5) c4 = read(4);
        if (maxC >= 6) c5 = read(5);

        auto z = Zref.getColumnManipulator();
        auto h = Hdiag.getColumnManipulator();
        auto gg = g.getColumnManipulator();

        const std::size_t N = _layout.N();

        // Quintic polynomial (gracefully degrades to cubic when c4=c5=0).
        auto y_poly = [&](double x) {
            return c0 + c1 * x + c2 * x * x + c3 * x * x * x
                 + c4 * x * x * x * x + c5 * x * x * x * x * x;
        };
        auto dy_poly = [&](double x) {
            return c1 + 2.0 * c2 * x + 3.0 * c3 * x * x
                 + 4.0 * c4 * x * x * x + 5.0 * c5 * x * x * x * x;
        };

        // Beyond _trackLength the polynomial continues as a straight line
        // tangent to the curve at x = _trackLength.
        const bool hasTrackLimit = _trackLength < 1.0e6;
        const double xEndSat = hasTrackLimit ? _trackLength : 0.0;
        const double yEndSat = hasTrackLimit ? y_poly(xEndSat) : 0.0;
        const double dyEndSat = hasTrackLimit ? dy_poly(xEndSat) : 0.0;

        auto y_at = [&](double x) {
            if (!hasTrackLimit || x <= xEndSat) {
                return y_poly(x);
            }
            return yEndSat + dyEndSat * (x - xEndSat);
        };
        auto dy_at = [&](double x) {
            if (!hasTrackLimit || x <= xEndSat) {
                return dy_poly(x);
            }
            return dyEndSat;
        };
        auto ddy_at = [&](double x) {
            if (!hasTrackLimit || x <= xEndSat) {
                return 2.0 * c2 + 6.0 * c3 * x + 12.0 * c4 * x * x + 20.0 * c5 * x * x * x;
            }
            return 0.0;
        };

        // Reference trajectory starts from the vehicle's current x position.
        // The initial-state constraint locks z(0) to the actual vehicle state;
        // the cost gradient pushes z(t) toward the polynomial for t >= 1.
        const double x0 = initial_x;
        const double y0 = y_at(x0);
        const double psi0 = std::atan2(dy_at(x0), 1.0);
        const double v0 = target_v;

        z(static_cast<td::UINT4>(_layout.idxX(0))) = x0;
        z(static_cast<td::UINT4>(_layout.idxY(0))) = y0;
        z(static_cast<td::UINT4>(_layout.idxPsi(0))) = psi0;
        z(static_cast<td::UINT4>(_layout.idxV(0))) = v0;

        // max_path_x: lookahead from the vehicle's current position
        const double max_path_x = x0 + _maxLookahead;

        // Initialize psiref_prev from the polynomial heading at the vehicle position
        double psiref_prev = psi0;

        // If _freezeAtPeak is true and c3 < 0 and c2 > 0, compute peak before loop.
        double x_peak = 0.0;
        if (_freezeAtPeak && c3 < 0.0 && c2 > 0.0) {
            x_peak = -2.0 * c2 / (3.0 * c3);
        }

        // Build forward reference trajectory from the vehicle's current x position.
        double xref = x0;
        double psiref = psi0;
        // Use projection_v for spatial advancement (matches actual vehicle speed),
        // but target_v for the velocity reference in the cost.
        const double adv_v = (projection_v >= 0.0) ? projection_v : target_v;
        const double vrefConst = target_v;

        for (std::size_t t = 1; t < N; ++t) {
            // Advance xref using the vehicle's heading (initial_psi).
            xref += adv_v * std::cos(initial_psi) * dt;

            // Freeze logic: clamp xref at peak if enabled
            if (_freezeAtPeak && c3 < 0.0 && c2 > 0.0 && xref >= x_peak) {
                xref = x_peak;
            }

            if (xref > max_path_x) {
                xref = max_path_x;
            }

            // Recompute reference heading from polynomial slope at new x
            psiref = std::atan2(dy_at(xref), 1.0);

            // Wrap reference heading to [-pi, pi]
            while (psiref - psiref_prev >  M_PI) psiref -= 2.0 * M_PI;
            while (psiref - psiref_prev < -M_PI) psiref += 2.0 * M_PI;
            psiref_prev = psiref;

            const double yref = y_at(xref);

            z(static_cast<td::UINT4>(_layout.idxX(t))) = xref;
            z(static_cast<td::UINT4>(_layout.idxY(t))) = yref;
            z(static_cast<td::UINT4>(_layout.idxPsi(t))) = psiref;
            z(static_cast<td::UINT4>(_layout.idxV(t))) = vrefConst;
        }

        const double Lf = 0.5;
        const double steerLimit = 0.6;
        for (std::size_t t = 0; t < N - 1; ++t) {
            const double psi_t = z(static_cast<td::UINT4>(_layout.idxPsi(t)));
            const double psi_next = z(static_cast<td::UINT4>(_layout.idxPsi(t + 1)));
            const double v_t = z(static_cast<td::UINT4>(_layout.idxV(t)));
            double dpsi = psi_next - psi_t;
            while (dpsi >  M_PI) dpsi -= 2.0 * M_PI;
            while (dpsi < -M_PI) dpsi += 2.0 * M_PI;
            double delta_ff = 0.0;
            if (std::fabs(v_t) > 0.1 && dt > 0.0) {
                delta_ff = std::atan(Lf * dpsi / (v_t * dt));
                if (delta_ff >  steerLimit) delta_ff =  steerLimit;
                if (delta_ff < -steerLimit) delta_ff = -steerLimit;
            }
            z(static_cast<td::UINT4>(_layout.idxDelta(t))) = delta_ff;
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
    double _trackLength = std::numeric_limits<double>::max();
};

} // namespace mpc
