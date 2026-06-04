#pragma once

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

    ScenarioConfig()
        : coeffs(4, 1, nullptr, true) {}
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
    return config;
}

inline ScenarioConfig makeLaneChangeScenarioConfig() {
    ScenarioConfig config;
    config.scenario = SimScenario::LaneChange;
    config.initialTelemetry = Telemetry{0.0, 0.0, 0.0, 1.5};
    config.obstacles = {{8.0, 0.3, 0.5}, {32.0, -0.3, 0.5}};
    config.freezeAtPeak = false;
    config.maxLookahead = 20.0;

    auto coeffs = config.coeffs.getColumnManipulator();
    coeffs(0) = 0.0;
    coeffs(1) = 0.311769;
    coeffs(2) = -0.023383;
    coeffs(3) = 0.000390;
    return config;
}

inline ScenarioConfig makeSCurveScenarioConfig() {
    ScenarioConfig config;
    config.scenario = SimScenario::SCurve;
    config.initialTelemetry = Telemetry{0.0, 0.0, 0.0, 1.5};
    config.obstacles = {};
    config.freezeAtPeak = false;
    config.maxLookahead = 25.0;

    auto coeffs = config.coeffs.getColumnManipulator();
    coeffs(0) = 0.0;
    coeffs(1) = 0.22269225;
    coeffs(2) = -0.00954395;
    coeffs(3) = 0.00009089;
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