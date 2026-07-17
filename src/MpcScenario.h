#pragma once

#include <limits>
#include <vector>

#include <dense/Matrix.h>

#include "MpcEngine.h"
#include "MpcObstacle.h"

namespace mpc {

enum class SimScenario {
    StraightLine = 0,
    LaneChange = 1,
    SCurve = 2,
};

struct ScenarioConfig {
    SimScenario scenario = SimScenario::StraightLine;
    Telemetry initialTelemetry{};
    dense::DblMatrix coeffs;
    std::vector<Obstacle> obstacles;
    bool freezeAtPeak = false;
    double maxLookahead = 15.0;
    // x-coordinate where the designed maneuver (S-weave / S-curve) ends.
    // Beyond this point the reference polynomial is unbounded (cubic term
    // dominates), so the reference generator must switch to a straight-line
    // continuation using the tangent at trackLength. Default = "never
    // saturate" for scenarios whose polynomial is already a straight line.
    double trackLength = std::numeric_limits<double>::max();

    ScenarioConfig()
        : coeffs(6, 1, nullptr, true) {}
};

inline const char* scenarioKey(SimScenario scenario) {
    switch (scenario) {
    case SimScenario::StraightLine:
        return "straight_line";
    case SimScenario::LaneChange:
        return "lane_change";
    case SimScenario::SCurve:
        return "s_curve";
    }
    return "straight_line";
}

inline ScenarioConfig makeStraightLineScenarioConfig() {
    ScenarioConfig config;
    config.scenario = SimScenario::StraightLine;
    config.initialTelemetry = Telemetry{0.0, 0.0, 0.0, 1.0};
    config.freezeAtPeak = false;
    config.maxLookahead = 15.0;

    auto coeffs = config.coeffs.getColumnManipulator();
    coeffs(0) = 0.0;
    coeffs(1) = 0.0;
    coeffs(2) = 0.0;
    coeffs(3) = 0.0;
    coeffs(4) = 0.0;
    coeffs(5) = 0.0;
    return config;
}

inline ScenarioConfig makeLaneChangeScenarioConfig() {
    ScenarioConfig config;
    config.scenario = SimScenario::LaneChange;
    config.initialTelemetry = Telemetry{0.0, 0.0, 0.0, 1.5};
    config.obstacles = {{8.0, 0.3, 0.5}, {32.0, -0.3, 0.5}};
    config.freezeAtPeak = false;
    config.maxLookahead = 20.0;
    config.trackLength = 40.0; // = 2L: end of the S-weave (see HANDOFF derivation)

    // Quintic S-curve with zero initial/terminal slope (matches vehicle heading).
    // y = c5*(-32000*x^2 + 3200*x^3 - 100*x^4 + x^5)
    // Roots at x=0 (double, slope 0), x=20, x=40 (double, slope 0).
    // Amplitude ±1.5, obstacles at (8,0.3) and (32,-0.3) are cleared.
    const double ampl = 1.5;
    const double c5 = ampl / 915849.0;
    auto coeffs = config.coeffs.getColumnManipulator();
    coeffs(0) = 0.0;
    coeffs(1) = 0.0;
    coeffs(2) = -32000.0 * c5;
    coeffs(3) = 3200.0 * c5;
    coeffs(4) = -100.0 * c5;
    coeffs(5) = c5;
    return config;
}

inline ScenarioConfig makeSCurveScenarioConfig() {
    ScenarioConfig config;
    config.scenario = SimScenario::SCurve;
    config.initialTelemetry = Telemetry{0.0, 0.0, 0.0, 1.5};
    config.obstacles = {};
    config.freezeAtPeak = false;
    config.maxLookahead = 30.0;
    config.trackLength = 40.0;

    // Inverted quintic S-curve with zero initial/terminal slope.
    // Same shape as LaneChange but negated (starts going UP, crosses at x=20, goes DOWN).
    const double ampl = 1.5;
    const double c5 = ampl / 915849.0;
    auto coeffs = config.coeffs.getColumnManipulator();
    coeffs(0) = 0.0;
    coeffs(1) = 0.0;
    coeffs(2) = 32000.0 * c5;
    coeffs(3) = -3200.0 * c5;
    coeffs(4) = 100.0 * c5;
    coeffs(5) = -c5;
    return config;
}

inline ScenarioConfig makeScenarioConfig(SimScenario scenario) {
    switch (scenario) {
    case SimScenario::StraightLine:
        return makeStraightLineScenarioConfig();
    case SimScenario::LaneChange:
        return makeLaneChangeScenarioConfig();
    case SimScenario::SCurve:
        return makeSCurveScenarioConfig();
    }
    return makeStraightLineScenarioConfig();
}

} // namespace mpc