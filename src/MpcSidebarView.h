#pragma once

#include <cstdio>
#include <functional>

#include <gui/ComboBox.h>
#include <gui/CheckBox.h>
#include <gui/HorizontalLayout.h>
#include <gui/Label.h>
#include <gui/Slider.h>
#include <gui/VerticalLayout.h>
#include <gui/View.h>

#include "MpcScenario.h"

class MpcSidebarView : public gui::View {
public:
    MpcSidebarView();

    void setFollowHandler(const std::function<void(bool)>& fn) { _onFollow = fn; }
    void setScenarioHandler(const std::function<void(mpc::SimScenario)>& fn) { _onScenario = fn; }
    void setSpeedHandler(const std::function<void(float)>& fn) { _onSpeed = fn; }

    void setTelemetry(double x, double y, double psi, double v, double err);
    void setSolverStatus(bool ok, bool converged, int iterations, double maxAbs);

    bool followVehicle() const { return _chkFollow.isChecked(); }

private:
    gui::VerticalLayout _layout;
    gui::HorizontalLayout _rowTitle;
    gui::HorizontalLayout _rowFollow;
    gui::HorizontalLayout _rowTelemetry;
    gui::HorizontalLayout _rowScenario;
    gui::HorizontalLayout _rowSpeed;
    gui::HorizontalLayout _rowPosX;
    gui::HorizontalLayout _rowPosY;
    gui::HorizontalLayout _rowPsi;
    gui::HorizontalLayout _rowVel;
    gui::HorizontalLayout _rowErr;
    gui::HorizontalLayout _rowSolver;
    gui::HorizontalLayout _rowIters;
    gui::HorizontalLayout _rowMaxAbs;

    gui::Label _lblTitle;
    gui::CheckBox _chkFollow;
    gui::ComboBox _cmbScenario;
    gui::Slider _slSpeed;

    gui::Label _lblTelemetry;
    gui::Label _lblScenario;
    gui::Label _lblSpeed;
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

    std::function<void(bool)> _onFollow;
    std::function<void(mpc::SimScenario)> _onScenario;
    std::function<void(float)> _onSpeed;
};

inline MpcSidebarView::MpcSidebarView()
    : _layout(13)
    , _rowTitle(1)
    , _rowFollow(1)
    , _rowTelemetry(1)
    , _rowScenario(2)
    , _rowSpeed(2)
    , _rowPosX(2)
    , _rowPosY(2)
    , _rowPsi(2)
    , _rowVel(2)
    , _rowErr(2)
    , _rowSolver(2)
    , _rowIters(2)
    , _rowMaxAbs(2)
    , _lblTitle(tr("sidebarTitle"))
    , _chkFollow(tr("followVehicle"))
    , _cmbScenario()
    , _slSpeed()
    , _lblTelemetry(tr("liveMetrics"))
    , _lblScenario(tr("scenarioLabel"))
    , _lblSpeed(tr("speedLabel"))
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

    _rowTitle.append(_lblTitle);
    _rowFollow.append(_chkFollow);
    _rowTelemetry.append(_lblTelemetry);
    _rowScenario.append(_lblScenario);
    _rowScenario.append(_cmbScenario);
    _rowSpeed.append(_lblSpeed);
    _rowSpeed.append(_slSpeed);

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
    _layout << _rowFollow;
    _layout << _rowTelemetry;
    _layout << _rowScenario;
    _layout << _rowSpeed;
    _layout << _rowPosX;
    _layout << _rowPosY;
    _layout << _rowPsi;
    _layout << _rowVel;
    _layout << _rowErr;
    _layout << _rowSolver;
    _layout << _rowIters;
    _layout << _rowMaxAbs;

    _chkFollow.onClick([this]() {
        if (_onFollow) {
            _onFollow(_chkFollow.isChecked());
        }
    });

    _cmbScenario.addItem(tr("scenarioStraightLine"));
    _cmbScenario.addItem(tr("scenarioLaneChange"));
    _cmbScenario.addItem(tr("scenarioSCurve"));
    _cmbScenario.selectIndex(0);
    _cmbScenario.onChangedSelection([this]() {
        if (_onScenario) {
            const int index = _cmbScenario.getSelectedIndex();
            _onScenario(static_cast<mpc::SimScenario>(index));
        }
    });

    _slSpeed.setRange(0, 200);
    _slSpeed.setValue(100);
    _slSpeed.onChangedValue([this]() {
        if (_onSpeed) {
            _onSpeed(static_cast<float>(_slSpeed.getValue()));
        }
    });

    // limit slider width so it doesn't extend to the very border
    _slSpeed.setSizeLimits(200, gui::Control::Limit::Fixed);

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