#pragma once

#include <vector>
#include <cstddef>

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
                                  float accelMax) {
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

        double c0 = 0.0;
        double c1 = 0.0;
        double c2 = 0.0;
        double c3 = 0.0;
        if (coeffs.getNoOfRows() >= 4 || coeffs.getNoOfCols() >= 4) {
            auto c = coeffs.getColumnManipulator();
            c0 = c(0);
            c1 = c(1);
            c2 = c(2);
            c3 = c(3);
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

        // Generate reference path independently spanning the full track ahead
        {
            const bool doFreeze = (c3 < 0.0 && c2 > 0.0);
            const double x_peak_viz = doFreeze ? (-2.0 * c2 / (3.0 * c3))
                                                : std::numeric_limits<double>::max();
            const double y_peak_viz = doFreeze
                ? (c2 * x_peak_viz * x_peak_viz + c3 * x_peak_viz * x_peak_viz * x_peak_viz)
                : 0.0;

            const double xStart = initialX - 5.0;
            const double xEnd   = initialX + 80.0;   // wider window to show full S
            const int nRef = 300;
            frame.referencePath.reserve(nRef);
            for (int i = 0; i < nRef; ++i) {
                const double x = xStart + (xEnd - xStart) * i / (nRef - 1);
                double y = c0 + c1 * x + c2 * x * x + c3 * x * x * x;
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