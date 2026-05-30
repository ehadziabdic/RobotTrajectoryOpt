#pragma once

#include <functional>

#include <gui/Dialog.h>

#include "MpcSettingsPopup.h"

class DialogSettings : public gui::Dialog {
private:
    MpcSettingsPopup _settingsView;
    std::function<void(int, double, const td::String&, bool)> _applyHandler;

protected:
    bool onClick(gui::Dialog::Button::ID btnID, gui::Button* /*pButton*/) override {
        if (btnID == gui::Dialog::Button::ID::OK && _applyHandler) {
            _applyHandler(
                _settingsView.selectedMaxIter(),
                _settingsView.selectedTolerance(),
                _settingsView.selectedLanguageExtension(),
                _settingsView.languageChanged());
        }
        return true;
    }

public:
    DialogSettings(gui::Frame* pFrame, td::UINT4 wndID = 0)
        : gui::Dialog(
              pFrame,
              {
                  {gui::Dialog::Button::ID::OK, tr("Ok"), gui::Button::Type::Default},
                  {gui::Dialog::Button::ID::Cancel, tr("Cancel")},
              },
              gui::Size(460, 220),
              wndID)
    {
        setTitle(tr("settings"));
        setCentralView(&_settingsView);
    }

    void syncValues(int maxIter, double tolerance, const td::String& languageExtension) {
        _settingsView.syncValues(maxIter, tolerance, languageExtension);
    }

    void setApplyHandler(const std::function<void(int, double, const td::String&, bool)>& handler) {
        _applyHandler = handler;
    }
};
