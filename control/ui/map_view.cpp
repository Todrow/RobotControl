#include "map_view.h"

#include <QImage>
#include <QPainter>
#include <QPainterPath>
#include <QtMath>

#include <cmath>

namespace {

// Deliberately three flat tones, not a gradient: the operator is reading
// topology (where can I go, where have I not been), not probabilities.
const QColor kUnknown(24, 24, 28);
const QColor kFree(58, 74, 92);
const QColor kOccupied(206, 214, 226);
const QColor kRobot(255, 186, 60);

constexpr int kPadding = 10;

}  // namespace

MapView::MapView(QWidget* parent) : QWidget(parent) {
    setAttribute(Qt::WA_TransparentForMouseEvents);
}

void MapView::setMap(const MapFrame& frame) {
    map_ = frame;
    have_map_ = frame.size > 0 && !frame.cells.empty();
    update();
}

void MapView::clearMap() {
    have_map_ = false;
    update();
}

void MapView::setLinkStatus(bool connected) {
    connected_ = connected;
    update();
}

void MapView::setExploring(bool exploring) {
    exploring_ = exploring;
    update();
}

void MapView::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    painter.fillRect(rect(), QColor(12, 12, 16, 220));
    painter.setPen(QColor(90, 90, 100));
    painter.drawRect(rect().adjusted(0, 0, -1, -1));

    const QRect canvas = rect().adjusted(kPadding, kPadding + 18, -kPadding, -kPadding);
    painter.setPen(QColor(190, 190, 200));
    painter.drawText(QRect(kPadding, 4, width() - 2 * kPadding, 16), Qt::AlignLeft,
                     exploring_ ? QStringLiteral("MAP  -  EXPLORING")
                                : QStringLiteral("MAP"));

    if (!connected_ || !have_map_) {
        painter.setPen(QColor(150, 150, 160));
        painter.drawText(canvas, Qt::AlignCenter,
                         connected_ ? QStringLiteral("waiting for the first map")
                                    : QStringLiteral("map link down"));
        return;
    }

    // Build the whole grid as an image and let Qt scale it. Drawing 160 000
    // rectangles by hand would stall the GUI thread on every frame.
    QImage image(map_.size, map_.size, QImage::Format_RGB32);
    for (int cy = 0; cy < map_.size; ++cy) {
        // The grid's +y is up in world terms, but an image's +y runs down, so
        // rows are filled bottom-up. Without this the map is mirrored and every
        // left turn looks like a right one.
        uint32_t* row = reinterpret_cast<uint32_t*>(image.scanLine(map_.size - 1 - cy));
        for (int cx = 0; cx < map_.size; ++cx) {
            const uint8_t value = map_.cells[static_cast<size_t>(cy) * map_.size + cx];
            const QColor& colour = value == mapproto::CELL_OCCUPIED ? kOccupied
                                   : value == mapproto::CELL_FREE   ? kFree
                                                                    : kUnknown;
            row[cx] = colour.rgb();
        }
    }

    // Keep the robot centred and show a fixed span of metres, so the scale on
    // screen never changes under the operator as the map grows.
    const float pixels_per_metre =
        static_cast<float>(std::min(canvas.width(), canvas.height())) / kViewMetres;
    const float cell_pixels = pixels_per_metre * map_.resolution;
    const float centre = static_cast<float>(map_.size) / 2.0f;
    const float robot_px = (map_.pose_x / map_.resolution + centre) * cell_pixels;
    const float robot_py = (map_.pose_y / map_.resolution + centre) * cell_pixels;
    const float full = static_cast<float>(map_.size) * cell_pixels;

    painter.save();
    painter.setClipRect(canvas);
    const QPointF origin(canvas.center().x() - robot_px, canvas.center().y() - (full - robot_py));
    painter.drawImage(QRectF(origin, QSizeF(full, full)), image);

    // The robot: a triangle, because a dot would not show which way it faces,
    // and heading is what tells the operator whether it is about to turn away.
    const QPointF here = canvas.center();
    const float heading = -map_.pose_theta;  // screen y runs the other way
    QPainterPath marker;
    const float nose = 11.0f;
    const float tail = 7.0f;
    marker.moveTo(here.x() + nose * std::cos(heading), here.y() + nose * std::sin(heading));
    marker.lineTo(here.x() + tail * std::cos(heading + 2.4f),
                  here.y() + tail * std::sin(heading + 2.4f));
    marker.lineTo(here.x() + tail * std::cos(heading - 2.4f),
                  here.y() + tail * std::sin(heading - 2.4f));
    marker.closeSubpath();
    painter.fillPath(marker, kRobot);
    painter.restore();

    // A low score means SLAM has lost track: the map may be wrong from here on,
    // and the operator needs to know that before believing what they see.
    if (map_.match_score < 0.30f) {
        painter.setPen(QColor(255, 120, 120));
        painter.drawText(QRect(kPadding, height() - 20, width() - 2 * kPadding, 16),
                         Qt::AlignLeft,
                         QStringLiteral("SLAM lost (match %1)").arg(map_.match_score, 0, 'f', 2));
    }
}
