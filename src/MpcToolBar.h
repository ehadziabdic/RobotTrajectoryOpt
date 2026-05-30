#pragma once

#include <gui/Image.h>
#include <gui/ToolBar.h>

class MpcToolBar : public gui::ToolBar {
private:
    gui::Image _imgSettings;
    gui::Image _imgStart;
    gui::Image _imgStop;
    gui::Image _imgReset;
    gui::Image _imgBack;
    gui::Image _imgForward;

    void setActionEnabled(td::BYTE actionID, bool enabled) {
        gui::ToolBarItem* pItem = getItem(20, 0, 0, actionID);
        if (pItem) {
            pItem->enable(enabled);
        }
    }

public:
    MpcToolBar()
        : gui::ToolBar("mainTB", 9)
        , _imgSettings(":settings")
        , _imgStart(":start")
        , _imgStop(":stop")
        , _imgReset(":reset")
        , _imgBack(":bck")
        , _imgForward(":fwd")
    {
        addItem(tr("settings"), &_imgSettings, tr("settingsTT"), 10, 0, 0, 10);
        addSpaceItem();
        addItem(tr("start"), &_imgStart, tr("start"), 20, 0, 0, 10);
        addItem(tr("stop"), &_imgStop, tr("stop"), 20, 0, 0, 11);
        addSpaceItem();
        addItem(tr("reset"), &_imgReset, tr("reset"), 20, 0, 0, 12);
        addSpaceItem();
        addItem(tr("back"), &_imgBack, tr("back"), 20, 0, 0, 13);
        addItem(tr("forward"), &_imgForward, tr("forward"), 20, 0, 0, 14);
    }

    void setRunningState(bool running) {
        setActionEnabled(10, !running);
        setActionEnabled(11, running);
    }
};
