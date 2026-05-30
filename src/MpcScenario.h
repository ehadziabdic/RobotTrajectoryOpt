#pragma once

#include <vector>

#include <dense/Matrix.h>

#include "MpcEngine.h"

namespace mpc {

enum class SimScenario {
    StraightLine = 0,
    LaneChange = 1,
    SCurve = 2,
};

struct Obstacle {
    double x = 0.0;
    double y = 0.0;
    double r = 0.5;
};

struct ScenarioConfig {
    SimScenario scenario = SimScenario::StraightLine;
    Telemetry initialTelemetry{};
    dense::DblMatrix coeffs;
    std::vector<Obstacle> obstacles;

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
    config.initialTelemetry = Telemetry{0.0, 0.0, 0.0, 1.0};
    config.obstacles.push_back({18.0, 1.25, 0.8});
    config.obstacles.push_back({24.0, 2.55, 0.8});

    auto coeffs = config.coeffs.getColumnManipulator();
    coeffs(0) = 0.0;
    coeffs(1) = 0.0;
    coeffs(2) = 0.0140;
    coeffs(3) = -0.00035;
    return config;
}

inline ScenarioConfig makeSCurveScenarioConfig() {
    ScenarioConfig config;
    config.scenario = SimScenario::SCurve;
    config.initialTelemetry = Telemetry{0.0, 0.0, 0.0, 1.0};

    auto coeffs = config.coeffs.getColumnManipulator();
    coeffs(0) = 0.0;
    coeffs(1) = 0.18;
    coeffs(2) = -0.018;
    coeffs(3) = 0.00045;
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