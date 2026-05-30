#pragma once

#include <cstdio>
#include <functional>

#include <gui/Button.h>
#include <gui/HorizontalLayout.h>
#include <gui/CheckBox.h>
#include <gui/Label.h>
#include <gui/VerticalLayout.h>
#include <gui/View.h>

class MpcSidebarView : public gui::View {
public:
    MpcSidebarView();

    void setPlayHandler(const std::function<void()>& fn) { _onPlay = fn; }
    void setPauseHandler(const std::function<void()>& fn) { _onPause = fn; }
    void setStepHandler(const std::function<void()>& fn) { _onStep = fn; }
    void setFollowHandler(const std::function<void(bool)>& fn) { _onFollow = fn; }

    void setTelemetry(double x, double y, double psi, double v, double err);
    void setSolverStatus(bool ok, bool converged, int iterations, double maxAbs);

    bool followVehicle() const { return _chkFollow.isChecked(); }

private:
    gui::VerticalLayout _layout;
    gui::HorizontalLayout _controls;
    gui::HorizontalLayout _rowTitle;
    gui::HorizontalLayout _rowFollow;
    gui::HorizontalLayout _rowTelemetry;
    gui::HorizontalLayout _rowPosX;
    gui::HorizontalLayout _rowPosY;
    gui::HorizontalLayout _rowPsi;
    gui::HorizontalLayout _rowVel;
    gui::HorizontalLayout _rowErr;
    gui::HorizontalLayout _rowSolver;
    gui::HorizontalLayout _rowIters;
    gui::HorizontalLayout _rowMaxAbs;

    gui::Label _lblTitle;
    gui::Button _btnPlay;
    gui::Button _btnPause;
    gui::Button _btnStep;
    gui::CheckBox _chkFollow;

    gui::Label _lblTelemetry;
    gui::Label _lblPosX;
    gui::Label _lblPosY;
    gui::Label _lblPsi;
    gui::Label _lblVel;
    gui::Label _lblErr;
    gui::Label _valPosX;
    gui::Label _valPosY;
    gui::Label _valPsi;
    gui::Label _valVel;
    gui::Label _valErr;

    gui::Label _lblSolver;
    gui::Label _valSolver;
    gui::Label _lblIter;
    gui::Label _valIter;
    gui::Label _lblMaxAbs;
    gui::Label _valMaxAbs;

    std::function<void()> _onPlay;
    std::function<void()> _onPause;
    std::function<void()> _onStep;
    std::function<void(bool)> _onFollow;
};

inline MpcSidebarView::MpcSidebarView()
    : _layout(16)
    , _controls(3)
    , _rowTitle(1)
    , _rowFollow(1)
    , _rowTelemetry(1)
    , _rowPosX(2)
    , _rowPosY(2)
    , _rowPsi(2)
    , _rowVel(2)
    , _rowErr(2)
    , _rowSolver(2)
    , _rowIters(2)
    , _rowMaxAbs(2)
    , _lblTitle(tr("sidebarTitle"))
    , _btnPlay(tr("play"))
    , _btnPause(tr("pause"))
    , _btnStep(tr("step"))
    , _chkFollow(tr("followVehicle"))
    , _lblTelemetry(tr("liveMetrics"))
    , _lblPosX(tr("lblPosX"))
    , _lblPosY(tr("lblPosY"))
    , _lblPsi(tr("lblHeading"))
    , _lblVel(tr("lblVelocity"))
    , _lblErr(tr("lblTrackingErr"))
    , _valPosX("-")
    , _valPosY("-")
    , _valPsi("-")
    , _valVel("-")
    , _valErr("-")
    , _lblSolver(tr("solverStatus"))
    , _valSolver("-")
    , _lblIter(tr("lblSqpIter"))
    , _valIter("-")
    , _lblMaxAbs(tr("lblMaxAbs"))
    , _valMaxAbs("-")
{
    setMargins(8, 8, 8, 8);

    _controls.append(_btnPlay);
    _controls.append(_btnPause);
    _controls.append(_btnStep);

    _rowTitle.append(_lblTitle);
    _rowFollow.append(_chkFollow);
    _rowTelemetry.append(_lblTelemetry);

    _rowPosX.append(_lblPosX);
    _rowPosX.append(_valPosX);
    _rowPosY.append(_lblPosY);
    _rowPosY.append(_valPosY);
    _rowPsi.append(_lblPsi);
    _rowPsi.append(_valPsi);
    _rowVel.append(_lblVel);
    _rowVel.append(_valVel);
    _rowErr.append(_lblErr);
    _rowErr.append(_valErr);

    _rowSolver.append(_lblSolver);
    _rowSolver.append(_valSolver);
    _rowIters.append(_lblIter);
    _rowIters.append(_valIter);
    _rowMaxAbs.append(_lblMaxAbs);
    _rowMaxAbs.append(_valMaxAbs);

    _layout << _rowTitle;
    _layout << _controls;
    _layout << _rowFollow;
    _layout << _rowTelemetry;
    _layout << _rowPosX;
    _layout << _rowPosY;
    _layout << _rowPsi;
    _layout << _rowVel;
    _layout << _rowErr;
    _layout << _rowSolver;
    _layout << _rowIters;
    _layout << _rowMaxAbs;

    _btnPlay.onClick([this]() {
        if (_onPlay) {
            _onPlay();
        }
    });
    _btnPause.onClick([this]() {
        if (_onPause) {
            _onPause();
        }
    });
    _btnStep.onClick([this]() {
        if (_onStep) {
            _onStep();
        }
    });
    _chkFollow.onClick([this]() {
        if (_onFollow) {
            _onFollow(_chkFollow.isChecked());
        }
    });
    _chkFollow.setChecked(true, false);

    setLayout(&_layout);
}

inline void MpcSidebarView::setTelemetry(double x, double y, double psi, double v, double err) {
    char buffer[64];

    std::snprintf(buffer, sizeof(buffer), "%.3f", x);
    _valPosX.setTitle(buffer);
    std::snprintf(buffer, sizeof(buffer), "%.3f", y);
    _valPosY.setTitle(buffer);
    std::snprintf(buffer, sizeof(buffer), "%.3f", psi);
    _valPsi.setTitle(buffer);
    std::snprintf(buffer, sizeof(buffer), "%.3f", v);
    _valVel.setTitle(buffer);
    std::snprintf(buffer, sizeof(buffer), "%.3f", err);
    _valErr.setTitle(buffer);
}

inline void MpcSidebarView::setSolverStatus(bool ok, bool converged, int iterations, double maxAbs) {
    if (!ok) {
        _valSolver.setTitle("Failed");
    } else if (converged) {
        _valSolver.setTitle("Converged");
    } else {
        _valSolver.setTitle("Running");
    }

    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%d", iterations);
    _valIter.setTitle(buffer);
    std::snprintf(buffer, sizeof(buffer), "%.4f", maxAbs);
    _valMaxAbs.setTitle(buffer);
}