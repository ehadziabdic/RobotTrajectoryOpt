#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <dense/Matrix.h>
#include <gui/SplitterLayout.h>
#include <gui/Timer.h>
#include <gui/View.h>

#include "MpcActuationCanvas.h"
#include "MpcEngine.h"
#include "MpcPathCanvas.h"
#include "MpcSidebarView.h"
#include "MpcVizAdapter.h"

class MainView : public gui::View {
public:
    MainView();

    void startSimulation();
    void stopSimulation();

protected:
    bool onTimer(gui::Timer* pTimer) override;

private:
    void advanceOneStep();
    void refreshVizFrame();
    void updateSidebar();

private:
    gui::SplitterLayout _mainSplit;
    gui::SplitterLayout _rightSplit;

    MpcSidebarView _sidebar;
    MpcPathCanvas _pathCanvas;
    MpcActuationCanvas _actuationCanvas;
    gui::Timer _timer;

    mpc::MpcLayout _layout;
    mpc::MpcEngine _engine;
    mpc::MpcSqp::Settings _settings;
    dense::DblMatrix _coeffs;
    mpc::Telemetry _telemetry;
    mpc::Trajectory _trajectory;

    std::vector<mpc::PlotPoint> _history;
    mpc::MpcVizFrame _frame;

    double _targetVelocity = 1.0;
    bool _running = false;
    bool _followVehicle = true;
    double _trackingError = 0.0;
};

inline MainView::MainView()
    : _mainSplit(gui::SplitterLayout::Orientation::Horizontal, gui::SplitterLayout::AuxiliaryCell::Second)
    , _rightSplit(gui::SplitterLayout::Orientation::Vertical, gui::SplitterLayout::AuxiliaryCell::Second)
    , _sidebar()
    , _pathCanvas()
    , _actuationCanvas()
    , _timer(this, 0.05f, false)
    , _layout(20, 0.1, 0.5)
    , _engine(_layout)
    , _coeffs(4, 1, nullptr, true)
{
    setMargins(0, 0, 0, 0);

    auto coeffs = _coeffs.getColumnManipulator();
    coeffs(0) = 0.0;
    coeffs(1) = 0.0;
    coeffs(2) = 0.0;
    coeffs(3) = 0.0;

    _telemetry.x = 0.0;
    _telemetry.y = 0.0;
    _telemetry.psi = 0.0;
    _telemetry.v = 1.0;

    _settings.maxIter = 6;
    _settings.tol = 1e-3;
    _settings.alpha = 1.0;
    _settings.verbose = false;
    _engine.setSettings(_settings);

    _history.push_back({static_cast<float>(_telemetry.x), static_cast<float>(_telemetry.y)});

    _sidebar.setPlayHandler([this]() { startSimulation(); });
    _sidebar.setPauseHandler([this]() { stopSimulation(); });
    _sidebar.setStepHandler([this]() { advanceOneStep(); });
    _sidebar.setFollowHandler([this](bool follow) {
        _followVehicle = follow;
        _pathCanvas.setFollowVehicle(_followVehicle);
    });

    _mainSplit.setContent(_sidebar, _rightSplit);
    _rightSplit.setContent(_pathCanvas, _actuationCanvas);
    setLayout(&_mainSplit);

    _pathCanvas.setFrame(&_frame);
    _pathCanvas.setTelemetry(&_telemetry);
    _pathCanvas.setFollowVehicle(_followVehicle);
    _pathCanvas.setTrackingError(_trackingError);
    _actuationCanvas.setFrame(&_frame);

    refreshVizFrame();
    updateSidebar();
}

inline void MainView::startSimulation() {
    _running = true;
    if (!_timer.isRunning()) {
        _timer.start();
    }
}

inline void MainView::stopSimulation() {
    _running = false;
    if (_timer.isRunning()) {
        _timer.stop();
    }
}

inline bool MainView::onTimer(gui::Timer* pTimer) {
    if (pTimer == &_timer) {
        if (!_running) {
            return true;
        }
        advanceOneStep();
        return true;
    }
    return false;
}

inline void MainView::advanceOneStep() {
    if (!_engine.Solve(_telemetry, _coeffs, _targetVelocity, _layout.dt(), _trajectory)) {
        updateSidebar();
        return;
    }

    if (_trajectory.N > 0) {
        auto states = _trajectory.states.getManipulator();
        const double refX = states(0, 0);
        const double refY = states(1, 0);
        const double dx = _telemetry.x - refX;
        const double dy = _telemetry.y - refY;
        _trackingError = std::sqrt(dx * dx + dy * dy);

        double delta = 0.0;
        double accel = 0.0;
        if (_trajectory.controls.getNoOfCols() > 0) {
            auto controls = _trajectory.controls.getManipulator();
            delta = controls(0, 0);
            accel = controls(1, 0);
        }

        const double dt = _layout.dt();
        const double lf = _layout.Lf();
        const double x = _telemetry.x;
        const double y = _telemetry.y;
        const double psi = _telemetry.psi;
        const double v = _telemetry.v;

        _telemetry.x = x + v * std::cos(psi) * dt;
        _telemetry.y = y + v * std::sin(psi) * dt;
        _telemetry.psi = psi + (v / lf) * delta * dt;
        _telemetry.v = v + accel * dt;

        _history.push_back({static_cast<float>(_telemetry.x), static_cast<float>(_telemetry.y)});
    }

    refreshVizFrame();
    updateSidebar();
}

inline void MainView::refreshVizFrame() {
    _frame = mpc::MpcVizAdapter::BuildFrame(
        _history,
        _coeffs,
        _trajectory,
        _layout.dt(),
        -0.436332f,
        0.436332f,
        -1.0f,
        1.0f);

    _pathCanvas.setFrame(&_frame);
    _pathCanvas.setTelemetry(&_telemetry);
    _pathCanvas.setFollowVehicle(_followVehicle);
    _pathCanvas.setTrackingError(_trackingError);
    _actuationCanvas.setFrame(&_frame);
}

inline void MainView::updateSidebar() {
    const auto& diag = _engine.diagnostics();
    _sidebar.setTelemetry(_telemetry.x, _telemetry.y, _telemetry.psi, _telemetry.v, _trackingError);
    _sidebar.setSolverStatus(diag.ok, diag.converged, diag.iterations, diag.maxAbs);
}

