#pragma once

#include <algorithm>
#include <array>
#include <cstdio>
#include <cmath>

#include <gui/ComboBox.h>
#include <gui/GridComposer.h>
#include <gui/GridLayout.h>
#include <gui/Label.h>
#include <gui/PopupView.h>
#include <gui/NumericEdit.h>
#include <td/String.h>

class MpcSettingsPopup : public gui::PopupView {
private:
    struct LanguageOption {
        const char* extension;
        const char* labelKey;
    };

    gui::Label _lblLanguage;
    gui::ComboBox _cmbLanguage;
    gui::Label _lblMaxIter;
    gui::NumericEdit _edMaxIter;
    gui::Label _lblTolerance;
    gui::ComboBox _cmbTolerance;
    gui::GridLayout _layout;

    int _initialLanguageIndex = 0;

    inline static const std::array<LanguageOption, 6> kLanguages{{
        {"EN", "langEnglish"},
        {"BA", "langBosnian"},
        {"DE", "langGerman"},
        {"ES", "langSpanish"},
        {"FR", "langFrench"},
        {"JP", "langJapanese"}
    }};

    static int toleranceToIndex(double tolerance) {
        if (tolerance <= 1e-5) {
            return 3;
        }
        if (tolerance <= 1e-4) {
            return 2;
        }
        if (tolerance <= 1e-3) {
            return 1;
        }
        return 0;
    }

    void updateValueLabels() {
        _edMaxIter.setMinValue(1.0);
        _edMaxIter.setMaxValue(1000000.0);
        _edMaxIter.setNumberOfDigitsAfterDecimalPoint(0);
        _edMaxIter.showThSep(false);
    }

public:
    MpcSettingsPopup()
        : _lblLanguage(tr("language"))
        , _lblMaxIter(tr("lblMaxIter"))
        , _edMaxIter(td::int4)
        , _lblTolerance(tr("lblTolerance"))
        , _layout(3, 2)
    {
        gui::GridComposer gc(_layout);
        gc.appendRow(_lblLanguage) << _cmbLanguage;
        gc.appendRow(_lblMaxIter) << _edMaxIter;
        gc.appendRow(_lblTolerance) << _cmbTolerance;
        setLayout(&_layout);

        for (std::size_t i = 0; i < kLanguages.size(); ++i) {
            _cmbLanguage.addItem(tr(kLanguages[i].labelKey));
        }

        _cmbTolerance.addItem("1e-2");
        _cmbTolerance.addItem("1e-3");
        _cmbTolerance.addItem("1e-4");
        _cmbTolerance.addItem("1e-5");

        _cmbTolerance.selectIndex(1);
        _cmbLanguage.selectIndex(_initialLanguageIndex);

        updateValueLabels();
    }

    void syncValues(int maxIter, double tolerance, const td::String& languageExtension) {
        char iterBuffer[64];
        std::snprintf(iterBuffer, sizeof(iterBuffer), "%d", maxIter);
        _edMaxIter.setText(iterBuffer, false);
        _cmbTolerance.selectIndex(toleranceToIndex(tolerance));

        _initialLanguageIndex = 0;
        for (std::size_t i = 0; i < kLanguages.size(); ++i) {
            if (languageExtension == td::String(kLanguages[i].extension)) {
                _initialLanguageIndex = static_cast<int>(i);
                break;
            }
        }
        _cmbLanguage.selectIndex(_initialLanguageIndex);
        updateValueLabels();
    }

    int selectedMaxIter() const {
        int value = 1;
        _edMaxIter.getValue(value);
        return value;
    }

    double selectedTolerance() const {
        switch (_cmbTolerance.getSelectedIndex()) {
            case 0: return 1e-2;
            case 1: return 1e-3;
            case 2: return 1e-4;
            case 3: return 1e-5;
            default: return 1e-3;
        }
    }

    td::String selectedLanguageExtension() const {
        const int idx = _cmbLanguage.getSelectedIndex();
        if (idx < 0 || idx >= static_cast<int>(kLanguages.size())) {
            return td::String("EN");
        }
        return td::String(kLanguages[static_cast<std::size_t>(idx)].extension);
    }

    bool languageChanged() const {
        return _cmbLanguage.getSelectedIndex() != _initialLanguageIndex;
    }
};
