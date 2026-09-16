#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <gui/Canvas.h>
#include <gui/DrawableString.h>
#include <gui/Shape.h>

#include "MpcVizAdapter.h"

class MpcActuationCanvas : public gui::Canvas {
public:
    MpcActuationCanvas();

    void setFrame(const mpc::MpcVizFrame* frame);

protected:
    void onDraw(const gui::Rect& rect) override;

private:
    const mpc::MpcVizFrame* _frame = nullptr;

    void drawStepPlot(const gui::Rect& rect,
                      const std::vector<float>& timeS,
                      const std::vector<float>& values,
                      float minLine,
                      float maxLine,
                      const char* title,
                      td::ColorID color) const;

    void drawStepPlot(const gui::Rect& rect,
                      const std::vector<float>& timeS,
                      const std::vector<float>& values,
                      float minLine,
                      float maxLine,
                      const td::String& title,
                      td::ColorID color) const;
};

inline MpcActuationCanvas::MpcActuationCanvas()
    : gui::Canvas()
{
    enableResizeEvent(true);
}

inline void MpcActuationCanvas::setFrame(const mpc::MpcVizFrame* frame) {
    _frame = frame;
    reDraw();
}

inline void MpcActuationCanvas::drawStepPlot(const gui::Rect& rect,
                                             const std::vector<float>& timeS,
                                             const std::vector<float>& values,
                                             float minLine,
                                             float maxLine,
                                             const char* title,
                                             td::ColorID color) const {
    drawStepPlot(rect, timeS, values, minLine, maxLine, td::String(title), color);
}

inline void MpcActuationCanvas::drawStepPlot(const gui::Rect& rect,
                                             const std::vector<float>& timeS,
                                             const std::vector<float>& values,
                                             float minLine,
                                             float maxLine,
                                             const td::String& title,
                                             td::ColorID color) const {
    gui::Shape::drawRect(rect, td::ColorID::Black);

    gui::DrawableString titleText(title);
    gui::Point titlePoint;
    titlePoint.x = rect.left + 10;
    titlePoint.y = rect.top + 8;
    titleText.draw(titlePoint, gui::Font::ID::SystemNormal, td::ColorID::SysText);

    const double left = rect.left + 12;
    const double right = rect.right - 12;
    const double top = rect.top + 28;
    const double bottom = rect.bottom - 14;

    gui::Shape::drawLine(gui::Point(static_cast<gui::CoordType>(left), static_cast<gui::CoordType>(top)),
                         gui::Point(static_cast<gui::CoordType>(right), static_cast<gui::CoordType>(top)),
                         td::ColorID::Gray, 1.0f);
    gui::Shape::drawLine(gui::Point(static_cast<gui::CoordType>(left), static_cast<gui::CoordType>(bottom)),
                         gui::Point(static_cast<gui::CoordType>(right), static_cast<gui::CoordType>(bottom)),
                         td::ColorID::Gray, 1.0f);

    if (timeS.size() < 2 || values.empty()) {
        gui::DrawableString empty(tr("actuationCanvasEmpty"));
        gui::Point msgPoint;
        msgPoint.x = rect.left + 10;
        msgPoint.y = rect.top + 28;
        empty.draw(msgPoint, gui::Font::ID::SystemNormal, td::ColorID::SysText);

        const gui::Point centerLeft(static_cast<gui::CoordType>(left), static_cast<gui::CoordType>((top + bottom) * 0.5));
        const gui::Point centerRight(static_cast<gui::CoordType>(right), static_cast<gui::CoordType>((top + bottom) * 0.5));
        gui::Shape::drawLine(centerLeft, centerRight, td::ColorID::DarkBlue, 2.0f);
        return;
    }

    double minValue = minLine;
    double maxValue = maxLine;
    for (float value : values) {
        minValue = std::min(minValue, static_cast<double>(value));
        maxValue = std::max(maxValue, static_cast<double>(value));
    }

    double range = maxValue - minValue;
    if (range < 1e-6) {
        range = 1.0;
        minValue -= 0.5;
        maxValue += 0.5;
    }

    const double timeStart = timeS.front();
    const double timeEnd = timeS[timeS.size() - 1];
    const double timeRange = std::max(1e-6, timeEnd - timeStart);

    auto toScreen = [&](double time, double value) {
        const double x = left + (time - timeStart) / timeRange * (right - left);
        const double y = bottom - (value - minValue) / range * (bottom - top);
        gui::Point point;
        point.x = static_cast<gui::CoordType>(x);
        point.y = static_cast<gui::CoordType>(y);
        return point;
    };

    const gui::Point minA = toScreen(timeStart, minLine);
    const gui::Point minB = toScreen(timeEnd, minLine);
    const gui::Point maxA = toScreen(timeStart, maxLine);
    const gui::Point maxB = toScreen(timeEnd, maxLine);
    gui::Shape::drawLine(minA, minB, td::ColorID::DarkRed, 1.0f);
    gui::Shape::drawLine(maxA, maxB, td::ColorID::DarkRed, 1.0f);

    for (std::size_t i = 0; i < values.size(); ++i) {
        const double t0 = timeS[std::min(i, timeS.size() - 1)];
        const double t1 = timeS[std::min(i + 1, timeS.size() - 1)];
        const gui::Point p0 = toScreen(t0, values[i]);
        const gui::Point p1 = toScreen(t1, values[i]);
        gui::Shape::drawLine(p0, p1, color, 2.0f);

        if (i + 1 < values.size()) {
            const gui::Point p2 = toScreen(t1, values[i + 1]);
            gui::Shape::drawLine(p1, p2, color, 2.0f);
        }
    }
}

inline void MpcActuationCanvas::onDraw(const gui::Rect& rect) {
    gui::Shape::drawRect(rect, td::ColorID::DarkGray);

    gui::Rect top = rect;
    gui::Rect bottom = rect;
    const gui::CoordType gap = 5;
    const gui::CoordType half = static_cast<gui::CoordType>((rect.bottom - rect.top - gap) * 0.5);
    top.bottom = top.top + half;
    bottom.top = top.bottom + gap;

    const gui::Rect separator(
        gui::Point(rect.left, top.bottom),
        gui::Size(rect.right - rect.left, gap));
    gui::Shape::drawRect(separator, td::ColorID::Transparent);

    const std::vector<float> emptyTime;
    const std::vector<float> emptyValues;

    if (!_frame) {
        drawStepPlot(top, emptyTime, emptyValues, -0.5f, 0.5f, tr("plotSteering").c_str(), td::ColorID::DarkBlue);
        drawStepPlot(bottom, emptyTime, emptyValues, -1.0f, 1.0f, tr("plotAcceleration").c_str(), td::ColorID::Yellow);
        return;
    }

    drawStepPlot(top, _frame->timeS, _frame->delta, _frame->deltaMin, _frame->deltaMax, tr("plotSteering").c_str(), td::ColorID::DarkBlue);
    drawStepPlot(bottom, _frame->timeS, _frame->accel, _frame->accelMin, _frame->accelMax, tr("plotAcceleration").c_str(), td::ColorID::Yellow);
}