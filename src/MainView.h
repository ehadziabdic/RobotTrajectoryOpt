#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

#include <dense/Matrix.h>
#include <gui/Alert.h>
#include <gui/Application.h>
#include <gui/SplitterLayout.h>
#include <gui/HorizontalLayout.h>
#include <gui/Timer.h>
#include <gui/VerticalLayout.h>
#include <gui/View.h>

#include "DialogSettings.h"
#include "MpcActuationCanvas.h"
#include "MpcEngine.h"
#include "MpcScenario.h"
#include "MpcPathCanvas.h"
#include "MpcSidebarView.h"
#include "MpcVizAdapter.h"

class MainView : public gui::View {
public:
    MainView();
    ~MainView();

    void startSimulation();
    void stopSimulation();
    void resetSimulation();
    void stepBackward();
    void stepForward();
    void openSettingsDialog();
    bool isRunning() const { return _running; }
    void setToolbarStateHandler(const std::function<void(bool, bool, bool)>& fn) { _onToolbarState = fn; }

protected:
    bool onTimer(gui::Timer* pTimer) override;

private:
    static double normalizeAngle(double angle) {
        constexpr double kPi = 3.1415926535897932384626433832795;
        constexpr double kTwoPi = 6.2831853071795864769252867665590;
        while (angle > kPi) {
            angle -= kTwoPi;
        }
        while (angle < -kPi) {
            angle += kTwoPi;
        }
        return angle;
    }

    void advanceOneStep();
    void refreshVizFrame();
    void updateSidebar();
    double evaluateReferenceY(double x) const;
    double computeTrackingError(const mpc::Telemetry& telem, const std::vector<mpc::PlotPoint>& referencePath) const;
    void setPlaybackSpeed(float sliderValue);
    void updatePlaybackButtons();
    void clearHistoryRedo();
    void notifyToolbarState();
    void finalizeStepOnGuiThread();

    struct StepSnapshot {
        mpc::Telemetry telemetry;
        mpc::Trajectory trajectory;
        std::vector<mpc::PlotPoint> history;
        mpc::MpcVizFrame frame;
        mpc::MpcEngine::Diagnostics diag;
        double trackingError = 0.0;
    };

    StepSnapshot captureSnapshot() const;
    void restoreSnapshot(const StepSnapshot& snapshot);

private:
    gui::SplitterLayout _splitter;
    gui::VerticalLayout _contentLayout;
    gui::View _contentView;

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
    mpc::SimScenario _scenario = mpc::SimScenario::StraightLine;
    mpc::ScenarioConfig _scenarioConfig;

    double _targetVelocity = 1.0;
    bool _running = false;
    bool _followVehicle = true;
    double _trackingError = 0.0;
    float _baseTimerInterval = 0.05f;
    float _speedPercent = 100.0f;
    std::vector<StepSnapshot> _undoSnapshots;
    std::vector<StepSnapshot> _redoSnapshots;
    std::function<void(bool, bool, bool)> _onToolbarState;
    std::mutex _simMutex;
    std::atomic<bool> _solverRunning{false};
    std::thread _solverThread;
};

inline MainView::MainView()
    : _splitter(gui::SplitterLayout::Orientation::Horizontal, gui::SplitterLayout::AuxiliaryCell::Second)
    , _contentLayout(2)
    , _sidebar()
    , _pathCanvas()
    , _actuationCanvas()
    , _timer(this, 0.05f, false)
    , _layout(20, 0.1, 0.5)
    , _engine(_layout)
    , _coeffs(4, 1, nullptr, true)
{
    setMargins(0, 0, 0, 0);

    const auto straightLine = mpc::makeScenarioConfig(mpc::SimScenario::StraightLine);
    _scenarioConfig = straightLine;
    auto coeffs = _coeffs.getColumnManipulator();
    auto scenarioCoeffs = _scenarioConfig.coeffs.getColumnManipulator();
    coeffs(0) = scenarioCoeffs(0);
    coeffs(1) = scenarioCoeffs(1);
    coeffs(2) = scenarioCoeffs(2);
    coeffs(3) = scenarioCoeffs(3);

    _telemetry = _scenarioConfig.initialTelemetry;
    _telemetry.psi = normalizeAngle(_telemetry.psi);
    _targetVelocity = _telemetry.v;

    _settings.maxIter = 60;
    _settings.tol = 2e-3;
    _settings.alpha = 0.12;
    _settings.verbose = true;
    _engine.setSettings(_settings);

    _history.push_back({static_cast<float>(_telemetry.x), static_cast<float>(_telemetry.y)});

    _sidebar.setScenarioHandler([this](mpc::SimScenario scenario) {
        _scenario = scenario;
        resetSimulation();
    });
    _sidebar.setFollowHandler([this](bool follow) {
        _followVehicle = follow;
        _pathCanvas.setFollowVehicle(_followVehicle);
    });
    _sidebar.setSpeedHandler([this](float value) { setPlaybackSpeed(value); });

    _contentLayout.setSpaceBetweenCells(6);
    _contentLayout.setMargins(0, 0);
    _contentLayout.append(_pathCanvas);
    _contentLayout.append(_actuationCanvas);

    // host the content layout inside a View so Splitter receives two Controls/Views
    _contentView.setLayout(&_contentLayout);
    // Make main content the primary (first) view and sidebar the auxiliary (second)
    _splitter.setContent(_contentView, _sidebar);
    // enforce a reasonable default sidebar width so app opens with left column visible
    _sidebar.setSizeLimits(320, gui::Control::Limit::Fixed);
    setLayout(&_splitter);

    _pathCanvas.setFrame(&_frame);
    _pathCanvas.setTelemetry(&_telemetry);
    _pathCanvas.setFollowVehicle(_followVehicle);
    _pathCanvas.setTrackingError(_trackingError);
    _actuationCanvas.setFrame(&_frame);

    refreshVizFrame();
    updateSidebar();
    updatePlaybackButtons();
    _undoSnapshots.push_back(captureSnapshot());
}

inline MainView::~MainView() {
    _running = false;
    if (_solverThread.joinable()) {
        _solverThread.join();
    }
}

inline void MainView::startSimulation() {
    _running = true;
    setPlaybackSpeed(_speedPercent);
    if (!_timer.isRunning()) {
        _timer.start();
    }
    updatePlaybackButtons();
}

inline void MainView::stopSimulation() {
    _running = false;
    if (_timer.isRunning()) {
        _timer.stop();
    }
    if (_solverThread.joinable()) {
        _solverThread.join();
    }
    updatePlaybackButtons();
}

inline void MainView::resetSimulation() {
    if (_solverThread.joinable()) {
        _solverThread.join();
    }
    stopSimulation();

    _scenarioConfig = mpc::makeScenarioConfig(_scenario);
    auto coeffs = _coeffs.getColumnManipulator();
    auto scenarioCoeffs = _scenarioConfig.coeffs.getColumnManipulator();
    coeffs(0) = scenarioCoeffs(0);
    coeffs(1) = scenarioCoeffs(1);
    coeffs(2) = scenarioCoeffs(2);
    coeffs(3) = scenarioCoeffs(3);

    _telemetry = _scenarioConfig.initialTelemetry;
    _telemetry.psi = normalizeAngle(_telemetry.psi);

    _trajectory = mpc::Trajectory{};
    _engine.reset();
    _history.clear();
    _history.push_back({static_cast<float>(_telemetry.x), static_cast<float>(_telemetry.y)});

    _trackingError = 0.0;

    refreshVizFrame();
    updateSidebar();
    clearHistoryRedo();
    _undoSnapshots.clear();
    _undoSnapshots.push_back(captureSnapshot());
    updatePlaybackButtons();
}

inline bool MainView::onTimer(gui::Timer* pTimer) {
    if (pTimer == &_timer) {
        if (!_running) {
            return true;
        }

        if (_solverThread.joinable() && !_solverRunning.load()) {
            _solverThread.join();
            finalizeStepOnGuiThread();
        }

        if (_solverRunning.load()) {
            return true;
        }

        _solverRunning.store(true);
        _solverThread = std::thread([this]() {
            {
                std::lock_guard<std::mutex> lock(_simMutex);
                advanceOneStep();
            }
            _solverRunning.store(false);
        });
        return true;
    }
    return false;
}

inline void MainView::advanceOneStep() {
    const double dt = _layout.dt();

    const bool ok = _engine.Solve(_telemetry, _coeffs, _targetVelocity, dt, _scenarioConfig.obstacles, _trajectory, _scenarioConfig.maxLookahead);
    const auto d = _engine.diagnostics();

    // Print detailed per-step diagnostics to console for debugging / offline analysis
    std::fprintf(stderr, "[MPC] Step: ok=%d converged=%d iter=%d maxAbs=%.6f telemetry_before(x,y,psi,v)=%.3f,%.3f,%.3f,%.3f trackingErr=%.6f trajN=%d\n",
        ok ? 1 : 0,
        d.converged ? 1 : 0,
        d.iterations,
        d.maxAbs,
        _telemetry.x,
        _telemetry.y,
        _telemetry.psi,
        _telemetry.v,
        _trackingError,
        static_cast<int>(_trajectory.N));

    if (!ok) {
        std::fprintf(stderr, "[MPC] Solve FAILED at this step (see diagnostics above)\n");
        return;
    }

    if (_trajectory.N > 0) {
        double delta = 0.0;
        double accel = 0.0;
        if (_trajectory.controls.getNoOfCols() > 0) {
            auto controls = _trajectory.controls.getManipulator();
            delta = controls(0, 0);
            accel = controls(1, 0);
            std::fprintf(stderr, "[MPC] First control: delta=%.6f accel=%.6f\n", delta, accel);
        }

        const double lf = _layout.Lf();
        const double x = _telemetry.x;
        const double y = _telemetry.y;
        const double psi = _telemetry.psi;
        const double v = _telemetry.v;

        _telemetry.x = x + v * std::cos(psi) * dt;
        _telemetry.y = y + v * std::sin(psi) * dt;
        _telemetry.psi = normalizeAngle(psi + (v / lf) * delta * dt);
        _telemetry.v = std::clamp(v + accel * dt, 0.1, _targetVelocity * 2.0);

        _history.push_back({static_cast<float>(_telemetry.x), static_cast<float>(_telemetry.y)});
    } else {
        std::fprintf(stderr, "[MPC] Trajectory returned N=0 (no predicted controls)\n");
    }

    // Print telemetry after update
    std::fprintf(stderr, "[MPC] Telemetry after step: x=%.3f y=%.3f psi=%.3f v=%.3f trackingErr=%.6f\n",
        _telemetry.x, _telemetry.y, _telemetry.psi, _telemetry.v, _trackingError);

}

inline void MainView::finalizeStepOnGuiThread() {
    refreshVizFrame();
    updateSidebar();
    clearHistoryRedo();
    _undoSnapshots.push_back(captureSnapshot());
    updatePlaybackButtons();
}

inline void MainView::refreshVizFrame() {
    _frame = mpc::MpcVizAdapter::BuildFrame(
        _history,
        _telemetry.x,
        _coeffs,
        _trajectory,
        _scenarioConfig.obstacles,
        _layout.dt(),
        -0.436332f,
        0.436332f,
        -1.0f,
        1.0f);

    _trackingError = computeTrackingError(_telemetry, _frame.referencePath);

    _pathCanvas.setFrame(&_frame);
    _pathCanvas.setTelemetry(&_telemetry);
    _pathCanvas.setFollowVehicle(_followVehicle);
    _pathCanvas.setTrackingError(_trackingError);
    _actuationCanvas.setFrame(&_frame);
}

inline MainView::StepSnapshot MainView::captureSnapshot() const {
    StepSnapshot snapshot;
    snapshot.telemetry = _telemetry;
    snapshot.trajectory = _trajectory;
    snapshot.history = _history;
    snapshot.frame = _frame;
    snapshot.diag = _engine.diagnostics();
    snapshot.trackingError = _trackingError;
    return snapshot;
}

inline void MainView::restoreSnapshot(const StepSnapshot& snapshot) {
    _telemetry = snapshot.telemetry;
    _trajectory = snapshot.trajectory;
    _history = snapshot.history;
    _frame = snapshot.frame;
    _trackingError = snapshot.trackingError;

    _pathCanvas.setFrame(&_frame);
    _pathCanvas.setTelemetry(&_telemetry);
    _pathCanvas.setFollowVehicle(_followVehicle);
    _pathCanvas.setTrackingError(_trackingError);
    _actuationCanvas.setFrame(&_frame);
    _sidebar.setTelemetry(_telemetry.x, _telemetry.y, _telemetry.psi, _telemetry.v, _trackingError);
    _sidebar.setSolverStatus(snapshot.diag.ok, snapshot.diag.converged, snapshot.diag.iterations, snapshot.diag.maxAbs);
}

inline void MainView::clearHistoryRedo() {
    _redoSnapshots.clear();
}

inline void MainView::updatePlaybackButtons() {
    notifyToolbarState();
}

inline void MainView::setPlaybackSpeed(float sliderValue) {
    _speedPercent = std::max(1.0f, sliderValue);
    const float factor = _speedPercent / 100.0f;
    const float interval = std::clamp(_baseTimerInterval / factor, 0.015f, 0.2f);
    _timer.setInterval(interval);
}

inline void MainView::stepBackward() {
    if (_running || _undoSnapshots.size() < 2) {
        return;
    }

    _redoSnapshots.push_back(_undoSnapshots.back());
    _undoSnapshots.pop_back();
    restoreSnapshot(_undoSnapshots.back());
    updatePlaybackButtons();
}

inline void MainView::stepForward() {
    if (_running) {
        return;
    }

    if (!_redoSnapshots.empty()) {
        _undoSnapshots.push_back(_redoSnapshots.back());
        restoreSnapshot(_redoSnapshots.back());
        _redoSnapshots.pop_back();
        updatePlaybackButtons();
        return;
    }

    {
        std::lock_guard<std::mutex> lock(_simMutex);
        advanceOneStep();
    }
    finalizeStepOnGuiThread();
}

inline void MainView::openSettingsDialog() {
    auto* app = gui::getApplication();
    auto* props = app ? app->getProperties() : nullptr;
    const td::String currentLanguage = props ? props->getValue("translation", td::String("EN")) : td::String("EN");

    auto* dlg = new DialogSettings(this);
    dlg->syncValues(_settings.maxIter, _settings.tol, currentLanguage);
    dlg->setApplyHandler([this, app, props](int maxIter, double tolerance, const td::String& langExt, bool languageChanged) {
        _settings.maxIter = std::max(1, maxIter);
        _settings.tol = tolerance;
        _engine.setSettings(_settings);

        if (props) {
            props->setValue("translation", langExt);
        }

        if (languageChanged) {
            gui::Alert::showYesNoQuestion(
                tr("RestartRequired"),
                tr("RestartRequiredInfo"),
                tr("Restart"),
                tr("DoNoRestart"),
                [app](gui::Alert::Answer answer) {
                    if (answer == gui::Alert::Answer::Yes && app) {
                        app->restart();
                    }
                });
        }
    });
    dlg->openNonModal();
}

inline void MainView::notifyToolbarState() {
    if (!_onToolbarState) {
        return;
    }

    const bool canBack = (!_running && _undoSnapshots.size() > 1);
    const bool canForward = !_running;
    _onToolbarState(_running, canBack, canForward);
}

inline double MainView::evaluateReferenceY(double x) const {
    auto coeffs = _coeffs.getColumnManipulator();
    const double c0 = coeffs(0);
    const double c1 = coeffs(1);
    const double c2 = coeffs(2);
    const double c3 = coeffs(3);
    return c0 + c1 * x + c2 * x * x + c3 * x * x * x;
}

inline double MainView::computeTrackingError(const mpc::Telemetry& telem, const std::vector<mpc::PlotPoint>& referencePath) const {
    if (!referencePath.empty()) {
        double minDist2 = std::numeric_limits<double>::max();
        for (const auto& point : referencePath) {
            const double dx = telem.x - static_cast<double>(point.x);
            const double dy = telem.y - static_cast<double>(point.y);
            const double dist2 = dx * dx + dy * dy;
            if (dist2 < minDist2) {
                minDist2 = dist2;
            }
        }

        if (std::isfinite(minDist2)) {
            return std::sqrt(minDist2);
        }
    }

    const double refY = evaluateReferenceY(telem.x);
    return std::fabs(telem.y - refY);
}

inline void MainView::updateSidebar() {
    const auto& diag = _engine.diagnostics();
    _sidebar.setTelemetry(_telemetry.x, _telemetry.y, _telemetry.psi, _telemetry.v, _trackingError);
    _sidebar.setSolverStatus(diag.ok, diag.converged, diag.iterations, diag.maxAbs);
}
