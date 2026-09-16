#pragma once

#include <vector>
#include <cstddef>
#include <limits>

#include <dense/Matrix.h>

#include "MpcEngine.h"
#include "MpcScenario.h"

namespace mpc {

struct PlotPoint {
    float x = 0.0f;
    float y = 0.0f;
};

struct MpcVizFrame {
    std::vector<PlotPoint> historyPath;
    std::vector<PlotPoint> predictedPath;
    std::vector<PlotPoint> referencePath;
    std::vector<Obstacle> obstacles;

    std::vector<float> predictedPsi;
    std::vector<float> predictedV;

    std::vector<float> timeS;
    std::vector<float> delta;
    std::vector<float> accel;

    float deltaMin = -0.5f;
    float deltaMax = 0.5f;
    float accelMin = -1.0f;
    float accelMax = 1.0f;
};

class MpcVizAdapter {
public:
    static MpcVizFrame BuildFrame(const std::vector<PlotPoint>& history,
                                  double initialX,
                                  const dense::DblMatrix& coeffs,
                                  const Trajectory& traj,
                                  const std::vector<Obstacle>& obstacles,
                                  double dt,
                                  float deltaMin,
                                  float deltaMax,
                                  float accelMin,
                                  float accelMax,
                                  double trackLength = std::numeric_limits<double>::max(),
                                  bool freezeAtPeak = false) {
        MpcVizFrame frame;
        frame.historyPath = history;
        frame.obstacles = obstacles;
        frame.deltaMin = deltaMin;
        frame.deltaMax = deltaMax;
        frame.accelMin = accelMin;
        frame.accelMax = accelMax;

        const std::size_t n = traj.N;
        if (n == 0) {
            return frame;
        }

        frame.predictedPath.reserve(n);
        frame.referencePath.reserve(n);
        frame.predictedPsi.resize(n);
        frame.predictedV.resize(n);
        frame.timeS.resize(n);

        const std::size_t controlCount = (n > 0) ? (n - 1) : 0;
        frame.delta.resize(controlCount);
        frame.accel.resize(controlCount);

        auto states = traj.states.getManipulator();
        auto controls = traj.controls.getManipulator();

        double c0 = 0.0, c1 = 0.0, c2 = 0.0, c3 = 0.0, c4 = 0.0, c5 = 0.0;
        const td::UINT4 maxC = std::max(coeffs.getNoOfRows(), coeffs.getNoOfCols());
        if (maxC >= 1) {
            auto cm = coeffs.getManipulator();
            auto read = [&](td::UINT4 i) -> double {
                return (coeffs.getNoOfRows() > 1) ? cm(i, 0) : cm(0, i);
            };
            c0 = read(0);
            if (maxC >= 2) c1 = read(1);
            if (maxC >= 3) c2 = read(2);
            if (maxC >= 4) c3 = read(3);
            if (maxC >= 5) c4 = read(4);
            if (maxC >= 6) c5 = read(5);
        }

        for (std::size_t t = 0; t < n; ++t) {
            const double x = states(0, static_cast<td::UINT4>(t));
            const double y = states(1, static_cast<td::UINT4>(t));
            const double psi = states(2, static_cast<td::UINT4>(t));
            const double v = states(3, static_cast<td::UINT4>(t));

            frame.predictedPath.push_back({static_cast<float>(x), static_cast<float>(y)});
            frame.predictedPsi[t] = static_cast<float>(psi);
            frame.predictedV[t] = static_cast<float>(v);
            frame.timeS[t] = static_cast<float>(static_cast<double>(t) * dt);
        }

        // Generate reference path independently spanning the full track ahead.
        // Mirrors MpcCost::UpdateReferenceTrajectory's saturation: past
        // trackLength the polynomial is replaced with its tangent line so the
        // drawn road matches what the solver is actually tracking (otherwise
        // the visualization shows the raw unbounded polynomial shooting off
        // while the solver has already flattened its target).
        // freezeAtPeak is a legacy flag only enabled for single-hump cubic
        // polynomials that do not return to zero at trackLength.
        {
            // Quintic helpers (gracefully handles cubic by c4=c5=0)
            auto y_poly = [&](double x) {
                return c0 + c1 * x + c2 * x * x + c3 * x * x * x
                     + c4 * x * x * x * x + c5 * x * x * x * x * x;
            };
            auto dy_poly = [&](double x) {
                return c1 + 2.0 * c2 * x + 3.0 * c3 * x * x
                     + 4.0 * c4 * x * x * x + 5.0 * c5 * x * x * x * x;
            };

            const bool doFreeze = freezeAtPeak && (c3 < 0.0 && c2 > 0.0);
            double x_peak_viz = std::numeric_limits<double>::max();
            double y_peak_viz = 0.0;
            if (doFreeze) {
                // For a cubic with c3<0, c2>0, the peak is at -2*c2/(3*c3).
                // For higher-degree polynomials we'd need a root-finder, but
                // freezeAtPeak is only used with simple cubic shapes.
                x_peak_viz = -2.0 * c2 / (3.0 * c3);
                y_peak_viz = y_poly(x_peak_viz);
            }
            const bool hasTrackLimit = trackLength < 1.0e6;
            const double xEndSat = hasTrackLimit ? trackLength : 0.0;
            const double yEndSat = hasTrackLimit ? y_poly(xEndSat) : 0.0;
            const double dyEndSat = hasTrackLimit ? dy_poly(xEndSat) : 0.0;

            const double xStart = initialX - 5.0;
            const double xEnd   = initialX + 80.0;   // wider window to show full S
            const int nRef = 300;
            frame.referencePath.reserve(nRef);
            for (int i = 0; i < nRef; ++i) {
                const double x = xStart + (xEnd - xStart) * i / (nRef - 1);
                double y;
                if (hasTrackLimit && x > xEndSat) {
                    y = yEndSat + dyEndSat * (x - xEndSat);
                } else {
                    y = y_poly(x);
                }
                if (doFreeze && x >= x_peak_viz)
                    y = y_peak_viz;   // clamp Y, X keeps advancing — line stays visible
                frame.referencePath.push_back({static_cast<float>(x), static_cast<float>(y)});
            }
        }

        for (std::size_t t = 0; t < controlCount; ++t) {
            frame.delta[t] = static_cast<float>(controls(0, static_cast<td::UINT4>(t)));
            frame.accel[t] = static_cast<float>(controls(1, static_cast<td::UINT4>(t)));
        }

        return frame;
    }
};

} // namespace mpc