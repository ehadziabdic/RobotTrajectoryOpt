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
    gui::NumericEdit _edTolerance;
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

    void updateValueLabels() {
        _edMaxIter.setMinValue(1.0);
        _edMaxIter.setMaxValue(1000000.0);
        _edMaxIter.setNumberOfDigitsAfterDecimalPoint(0);
        _edMaxIter.showThSep(false);
        _edTolerance.setMinValue(1e-8);
        _edTolerance.setMaxValue(1.0);
        _edTolerance.setNumberOfDigitsAfterDecimalPoint(6);
        _edTolerance.showThSep(false);
    }

public:
    MpcSettingsPopup()
        : _lblLanguage(tr("language"))
        , _lblMaxIter(tr("lblMaxIter"))
        , _edMaxIter(td::int4)
        , _lblTolerance(tr("lblTolerance"))
        , _edTolerance(td::real8)
        , _layout(3, 2)
    {
        gui::GridComposer gc(_layout);
        gc.appendRow(_lblLanguage) << _cmbLanguage;
        gc.appendRow(_lblMaxIter) << _edMaxIter;
        gc.appendRow(_lblTolerance) << _edTolerance;
        setLayout(&_layout);

        for (std::size_t i = 0; i < kLanguages.size(); ++i) {
            _cmbLanguage.addItem(tr(kLanguages[i].labelKey));
        }

        // default tolerance value
        _edTolerance.setText("0.002", false);
        _cmbLanguage.selectIndex(_initialLanguageIndex);

        updateValueLabels();
    }

    void syncValues(int maxIter, double tolerance, const td::String& languageExtension) {
        char iterBuffer[64];
        std::snprintf(iterBuffer, sizeof(iterBuffer), "%d", maxIter);
        _edMaxIter.setText(iterBuffer, false);
        char tolBuf[64];
        std::snprintf(tolBuf, sizeof(tolBuf), "%.6f", tolerance);
        _edTolerance.setText(tolBuf, false);

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
        double value = 2e-3;
        _edTolerance.getValue(value);
        return value;
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
