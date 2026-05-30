#pragma once
#include <gui/Window.h>
#include "MainView.h"

class MainWindow : public gui::Window {
private:
    MainView _view;

public:
    MainWindow()
        : gui::Window(gui::Geometry(50, 50, 1440, 900))
        , _view()
    {
        setTitle(tr("appTitle"));
        setCentralView(&_view);
    }

    void onInitialAppearance() override {
        _view.startSimulation();
    }
};

