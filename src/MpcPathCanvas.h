#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <gui/Canvas.h>
#include <gui/DrawableString.h>
#include <gui/Shape.h>

#include "MpcEngine.h"
#include "MpcVizAdapter.h"

class MpcPathCanvas : public gui::Canvas {
public:
    MpcPathCanvas();

    void setFrame(const mpc::MpcVizFrame* frame);
    void setTelemetry(const mpc::Telemetry* telem);
    void setFollowVehicle(bool follow);
    void setTrackingError(double err);

protected:
    void onDraw(const gui::Rect& rect) override;

private:
    struct Viewport {
        double centerX = 0.0;
        double centerY = 0.0;
        double scale = 30.0;
    };

    const mpc::MpcVizFrame* _frame = nullptr;
    const mpc::Telemetry* _telem = nullptr;
    bool _follow = true;
    double _trackingErr = 0.0;

    gui::Point worldToScreen(const gui::Rect& rect, double x, double y, const Viewport& view) const;
    Viewport computeViewport(const gui::Rect& rect) const;
    void drawPolyline(const std::vector<mpc::PlotPoint>& pts,
                      const gui::Rect& rect,
                      const Viewport& view,
                      td::ColorID color,
                      float lineWidth) const;
    void drawVehicle(const gui::Rect& rect, const Viewport& view) const;
    void drawPreviewLine(const gui::Rect& rect, const Viewport& view) const;
};

inline MpcPathCanvas::MpcPathCanvas()
    : gui::Canvas()
{
    enableResizeEvent(true);
}

inline void MpcPathCanvas::setFrame(const mpc::MpcVizFrame* frame) {
    _frame = frame;
    reDraw();
}

inline void MpcPathCanvas::setTelemetry(const mpc::Telemetry* telem) {
    _telem = telem;
    reDraw();
}

inline void MpcPathCanvas::setFollowVehicle(bool follow) {
    _follow = follow;
    reDraw();
}

inline void MpcPathCanvas::setTrackingError(double err) {
    _trackingErr = err;
    reDraw();
}

inline gui::Point MpcPathCanvas::worldToScreen(const gui::Rect& rect, double x, double y, const Viewport& view) const {
    const double width = rect.right - rect.left;
    const double height = rect.bottom - rect.top;
    const double px = rect.left + width * 0.5 + (x - view.centerX) * view.scale;
    const double py = rect.top + height * 0.5 - (y - view.centerY) * view.scale;

    gui::Point point;
    point.x = static_cast<gui::CoordType>(px);
    point.y = static_cast<gui::CoordType>(py);
    return point;
}

inline MpcPathCanvas::Viewport MpcPathCanvas::computeViewport(const gui::Rect& rect) const {
    Viewport view;
    std::vector<mpc::PlotPoint> allPoints;

    if (_frame) {
        allPoints.insert(allPoints.end(), _frame->historyPath.begin(), _frame->historyPath.end());
        allPoints.insert(allPoints.end(), _frame->predictedPath.begin(), _frame->predictedPath.end());
        allPoints.insert(allPoints.end(), _frame->referencePath.begin(), _frame->referencePath.end());
    }

    if (allPoints.empty() && _telem) {
        view.centerX = _telem->x;
        view.centerY = _telem->y;
        view.scale = 30.0;
        return view;
    }

    if (!allPoints.empty()) {
        double minX = allPoints.front().x;
        double maxX = allPoints.front().x;
        double minY = allPoints.front().y;
        double maxY = allPoints.front().y;

        for (const auto& pt : allPoints) {
            minX = std::min(minX, static_cast<double>(pt.x));
            maxX = std::max(maxX, static_cast<double>(pt.x));
            minY = std::min(minY, static_cast<double>(pt.y));
            maxY = std::max(maxY, static_cast<double>(pt.y));
        }

        const double spanX = std::max(1.0, maxX - minX);
        const double spanY = std::max(1.0, maxY - minY);
        const double width = rect.right - rect.left;
        const double height = rect.bottom - rect.top;
        const double scaleX = (width * 0.80) / spanX;
        const double scaleY = (height * 0.80) / spanY;
        view.scale = std::max(5.0, std::min(scaleX, scaleY));
        view.centerX = (minX + maxX) * 0.5;
        view.centerY = (minY + maxY) * 0.5;
    }

    if (_follow && _telem) {
        view.centerX = _telem->x;
        view.centerY = _telem->y;
    }

    return view;
}

inline void MpcPathCanvas::drawPolyline(const std::vector<mpc::PlotPoint>& pts,
                                        const gui::Rect& rect,
                                        const Viewport& view,
                                        td::ColorID color,
                                        float lineWidth) const {
    if (pts.size() < 2) {
        return;
    }

    for (std::size_t i = 1; i < pts.size(); ++i) {
        const gui::Point p1 = worldToScreen(rect, pts[i - 1].x, pts[i - 1].y, view);
        const gui::Point p2 = worldToScreen(rect, pts[i].x, pts[i].y, view);
        gui::Shape::drawLine(p1, p2, color, lineWidth);
    }
}

inline void MpcPathCanvas::drawVehicle(const gui::Rect& rect, const Viewport& view) const {
    if (!_telem) {
        return;
    }

    const gui::Point center = worldToScreen(rect, _telem->x, _telem->y, view);
    const double psi = _telem->psi;
    const double size = std::max(12.0, 0.28 * view.scale);
    const double halfLength = size * 0.75;
    const double halfWidth = size * 0.45;

    auto rotate = [&](double localX, double localY) {
        const double worldX = localX * std::cos(psi) - localY * std::sin(psi);
        const double worldY = localX * std::sin(psi) + localY * std::cos(psi);
        gui::Point pt;
        pt.x = center.x + static_cast<gui::CoordType>(worldX);
        pt.y = center.y - static_cast<gui::CoordType>(worldY);
        return pt;
    };

    const gui::Point frontLeft = rotate(halfLength, halfWidth);
    const gui::Point frontRight = rotate(halfLength, -halfWidth);
    const gui::Point rearRight = rotate(-halfLength, -halfWidth);
    const gui::Point rearLeft = rotate(-halfLength, halfWidth);
    const gui::Point nose = rotate(halfLength + 0.45 * size, 0.0);

    gui::Shape::drawLine(frontLeft, frontRight, td::ColorID::SysText, 2.0f);
    gui::Shape::drawLine(frontRight, rearRight, td::ColorID::SysText, 2.0f);
    gui::Shape::drawLine(rearRight, rearLeft, td::ColorID::SysText, 2.0f);
    gui::Shape::drawLine(rearLeft, frontLeft, td::ColorID::SysText, 2.0f);
    gui::Shape::drawLine(center, nose, td::ColorID::DarkBlue, 2.0f);
}

inline void MpcPathCanvas::drawPreviewLine(const gui::Rect& rect, const Viewport& view) const {
    if (!_telem) {
        return;
    }

    const double previewLength = std::max(20.0, 15.0 * std::fabs(_telem->v));
    const double dx = std::cos(_telem->psi) * previewLength;
    const double dy = std::sin(_telem->psi) * previewLength;

    const gui::Point start = worldToScreen(rect, _telem->x, _telem->y, view);
    const gui::Point end = worldToScreen(rect, _telem->x + dx, _telem->y + dy, view);
    gui::Shape::drawLine(start, end, td::ColorID::DarkBlue, 2.0f);
}

inline void MpcPathCanvas::onDraw(const gui::Rect& rect) {
    const Viewport view = computeViewport(rect);

    gui::Shape::drawRect(rect, td::ColorID::DarkGray, 1.0f);

    const gui::CoordType padLeft = 12;
    const gui::CoordType padTop = 28;
    const gui::Rect plotRect(
        gui::Point(rect.left + padLeft, rect.top + padTop),
        gui::Size(rect.right - rect.left - 2 * padLeft, rect.bottom - rect.top - padTop - 24));

    gui::Shape::drawRect(plotRect, td::ColorID::DimGray, 1.0f);
    gui::Shape::drawLine(gui::Point(plotRect.left, plotRect.top), gui::Point(plotRect.right, plotRect.top), td::ColorID::Gray, 1.0f);
    gui::Shape::drawLine(gui::Point(plotRect.left, plotRect.bottom), gui::Point(plotRect.right, plotRect.bottom), td::ColorID::Gray, 1.0f);
    gui::Shape::drawLine(gui::Point(plotRect.left, plotRect.top), gui::Point(plotRect.left, plotRect.bottom), td::ColorID::Gray, 1.0f);
    gui::Shape::drawLine(gui::Point(plotRect.right, plotRect.top), gui::Point(plotRect.right, plotRect.bottom), td::ColorID::Gray, 1.0f);

    gui::DrawableString title(tr("pathCanvasTitle"));
    gui::Point titlePoint;
    titlePoint.x = rect.left + 10;
    titlePoint.y = rect.top + 10;
    title.draw(titlePoint, gui::Font::ID::SystemNormal, td::ColorID::SysText);

    if (!_frame) {
        gui::DrawableString empty(tr("pathCanvasEmpty"));
        gui::Point msgPoint;
        msgPoint.x = rect.left + 10;
        msgPoint.y = rect.top + 30;
        empty.draw(msgPoint, gui::Font::ID::SystemNormal, td::ColorID::SysText);
        drawPreviewLine(plotRect, view);
        drawVehicle(plotRect, view);
        return;
    }

    gui::DrawableString axisX(tr("axisX"));
    gui::Point axisXPoint;
    axisXPoint.x = plotRect.right - 90;
    axisXPoint.y = plotRect.bottom + 4;
    axisX.draw(axisXPoint, gui::Font::ID::SystemNormal, td::ColorID::SysText);

    gui::DrawableString axisY(tr("axisY"));
    gui::Point axisYPoint;
    axisYPoint.x = plotRect.left + 4;
    axisYPoint.y = plotRect.top + 2;
    axisY.draw(axisYPoint, gui::Font::ID::SystemNormal, td::ColorID::SysText);

    const gui::CoordType legendX = plotRect.right - 220;
    const gui::CoordType legendY = plotRect.top + 8;
    gui::Shape::drawLine(gui::Point(legendX, legendY + 6), gui::Point(legendX + 20, legendY + 6), td::ColorID::Yellow, 2.0f);
    gui::Shape::drawLine(gui::Point(legendX, legendY + 22), gui::Point(legendX + 20, legendY + 22), td::ColorID::DarkRed, 2.0f);
    gui::Shape::drawLine(gui::Point(legendX, legendY + 38), gui::Point(legendX + 20, legendY + 38), td::ColorID::DarkBlue, 2.0f);

    gui::DrawableString legendHistory(tr("legendHistory"));
    gui::Point legendTextPoint;
    legendTextPoint.x = legendX + 26;
    legendTextPoint.y = legendY;
    legendHistory.draw(legendTextPoint, gui::Font::ID::SystemNormal, td::ColorID::SysText);

    gui::DrawableString legendReference(tr("legendReference"));
    legendTextPoint.y = legendY + 16;
    legendReference.draw(legendTextPoint, gui::Font::ID::SystemNormal, td::ColorID::SysText);

    gui::DrawableString legendPredicted(tr("legendPredicted"));
    legendTextPoint.y = legendY + 32;
    legendPredicted.draw(legendTextPoint, gui::Font::ID::SystemNormal, td::ColorID::SysText);

    drawPolyline(_frame->historyPath, plotRect, view, td::ColorID::Yellow, 2.0f);
    drawPolyline(_frame->referencePath, plotRect, view, td::ColorID::DarkRed, 1.5f);
    drawPolyline(_frame->predictedPath, plotRect, view, td::ColorID::DarkBlue, 2.0f);

    drawPreviewLine(plotRect, view);
    drawVehicle(plotRect, view);

    char buffer[128];
    std::snprintf(buffer, sizeof(buffer), "%s %.3f", tr("pathCanvasTrackingErr").c_str(), _trackingErr);
    gui::DrawableString errText(buffer);
    gui::Point errPoint;
    errPoint.x = rect.left + 10;
    errPoint.y = rect.bottom - 24;
    errText.draw(errPoint, gui::Font::ID::SystemNormal, td::ColorID::SysText);
}