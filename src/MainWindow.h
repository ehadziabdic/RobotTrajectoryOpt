#pragma once
#include <gui/Window.h>
#include "MainView.h"
#include "MpcToolBar.h"

class MainWindow : public gui::Window {
private:
    MainView _view;
    MpcToolBar _toolBar;

public:
    MainWindow()
        : gui::Window(gui::Geometry(50, 50, 1440, 900))
        , _view()
        , _toolBar()
    {
        setTitle(tr("appTitle"));
        setToolBar(_toolBar);

        setCentralView(&_view);
    }

    void onInitialAppearance() override {
        _view.stopSimulation();
        _toolBar.setRunningState(false);
    }

    bool onActionItem(gui::ActionItemDescriptor& aiDesc) override {
        auto [menuID, firstSubMenuID, lastSubMenuID, actionID] = aiDesc.getIDs();

        if (menuID == 10 && actionID == 10) {
            _view.openSettingsDialog();
            return true;
        }

        if (menuID == 20 && firstSubMenuID == 0 && lastSubMenuID == 0) {
            switch (actionID) {
                case 10:
                    _view.startSimulation();
                    _toolBar.setRunningState(true);
                    return true;
                case 11:
                    if (!_view.isRunning()) {
                        return true;
                    }
                    _view.stopSimulation();
                    _toolBar.setRunningState(false);
                    return true;
                case 12:
                    _view.resetSimulation();
                    _toolBar.setRunningState(false);
                    return true;
                case 13:
                    _view.stepBackward();
                    return true;
                case 14:
                    _view.stepForward();
                    return true;
            }
        }

        return false;
    }
};

