#include "yaw_indicator.h"

#include <QPainter>
#include <QPainterPath>
#include <QPolygonF>
#include <QtMath>
#include <algorithm>
#include <cmath>

namespace {
constexpr int kSize = 168;
constexpr int kFooter = 18;
constexpr int kHeader = 20;

// yaw travels -1..1 on the wire; the angle that maps to depends on the servo
// and the mount, so the grid is labelled with this nominal half-range.
constexpr double kYawRangeDeg = 90.0;

constexpr float kEpsilon = 1e-4f;

// 0 = forward (up), positive = clockwise, matching mouse-right = yaw up.
QPointF dirAt(double deg) {
    const double rad = qDegreesToRadians(deg);
    return QPointF(std::sin(rad), -std::cos(rad));
}

// Colour and opacity together, so an unrecognised byte cannot end up painted
// solid like a real verdict. That happens when the peer was built from a
// different protocol.h: the frames desynchronise and arbitrary bytes land in
// this field. A value we cannot interpret is not a reading, so it is drawn
// exactly like Unknown.
struct SectorPaint {
    QColor color;
    int alpha;
};

SectorPaint sectorPaint(proto::SectorStatus status) {
    switch (status) {
        case proto::SectorStatus::Green:  return {QColor(58, 176, 100), 165};
        case proto::SectorStatus::Yellow: return {QColor(224, 176, 56), 165};
        case proto::SectorStatus::Red:    return {QColor(226, 68, 68), 165};
        case proto::SectorStatus::Unknown: break;
    }
    return {QColor(78, 78, 86), 70};
}

// One annulus segment. Angles here use this widget's convention (0 = up,
// clockwise positive), while Qt's arcs start at 3 o'clock and run
// counter-clockwise, hence the flip.
QPainterPath sectorWedge(const QPointF& c, double inner, double outer, double centre_deg,
                         double half_width_deg) {
    const QRectF outer_box(c.x() - outer, c.y() - outer, 2 * outer, 2 * outer);
    const QRectF inner_box(c.x() - inner, c.y() - inner, 2 * inner, 2 * inner);
    const double start = 90.0 - (centre_deg - half_width_deg);
    const double span = -2.0 * half_width_deg;
    QPainterPath path;
    path.arcMoveTo(outer_box, start);
    path.arcTo(outer_box, start, span);
    path.arcTo(inner_box, start + span, -span);
    path.closeSubpath();
    return path;
}

QString formatDeg(double deg) {
    return QString::asprintf("%+.0f", deg) + QChar(0x00B0);
}
}  // namespace

YawIndicator::YawIndicator(QWidget* parent) : QWidget(parent) {
    sectors_.fill(proto::SectorStatus::Unknown);
    setFixedSize(kSize, kSize);
    setAttribute(Qt::WA_TransparentForMouseEvents);
    setFocusPolicy(Qt::NoFocus);
}

void YawIndicator::setDesiredYaw(float yaw) {
    if (std::fabs(yaw - desired_) < kEpsilon) return;
    desired_ = yaw;
    update();
}

void YawIndicator::setActualYaw(float yaw) {
    // The robot reports NaN whenever the servo setpoint is unknown (servos off,
    // PWM error, no command yet). Painting that would rotate the turret by NaN.
    if (!std::isfinite(yaw)) {
        clearActualYaw();
        return;
    }
    if (has_actual_ && std::fabs(yaw - actual_) < kEpsilon) return;
    has_actual_ = true;
    actual_ = yaw;
    update();
}

void YawIndicator::clearActualYaw() {
    if (!has_actual_) return;
    has_actual_ = false;
    update();
}

void YawIndicator::setSectors(const proto::SectorStatus (&sectors)[proto::SECTOR_COUNT]) {
    bool changed = false;
    for (int i = 0; i < proto::SECTOR_COUNT; ++i) {
        const size_t index = static_cast<size_t>(i);
        if (sectors_[index] == sectors[index]) continue;
        sectors_[index] = sectors[index];
        changed = true;
    }
    if (changed) update();
}

void YawIndicator::clearSectors() {
    proto::SectorStatus unknown[proto::SECTOR_COUNT];
    for (auto& sector : unknown) sector = proto::SectorStatus::Unknown;
    setSectors(unknown);
}

void YawIndicator::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(0, 0, 0));
    p.setBrush(QColor(24, 24, 26, 210));
    p.setPen(QColor(70, 70, 74));
    p.drawRoundedRect(QRectF(0.5, 0.5, width() - 1.0, height() - 1.0), 4, 4);

    p.setFont(QFont(QStringLiteral("Consolas"), 8));
    p.setPen(QColor(150, 150, 155));
    p.drawText(QRect(10, 4, 60, 14), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("YAW"));
    p.setPen(QColor(105, 105, 112));
    p.drawText(QRect(width() - 70, 4, 60, 14), Qt::AlignRight | Qt::AlignVCenter,
               QChar(0x00B1) + QString::asprintf("%.0f", kYawRangeDeg) + QChar(0x00B0));

    const QPointF c(width() / 2.0, (kHeader + (height() - kFooter)) / 2.0);
    const double r = std::min(width() / 2.0 - 12.0, (height() - kFooter - kHeader) / 2.0 - 4.0);
    const QRectF ring(c.x() - r, c.y() - r, 2 * r, 2 * r);

    // Obstacle sectors, clockwise from the nose: one 60 deg wedge each, in the
    // outer band. Drawn before the grid so the ticks stay legible on top, and
    // left with a gap between wedges so the six stay countable at this size.
    p.setPen(Qt::NoPen);
    for (int i = 0; i < proto::SECTOR_COUNT; ++i) {
        const SectorPaint paint = sectorPaint(sectors_[static_cast<size_t>(i)]);
        QColor fill = paint.color;
        fill.setAlpha(paint.alpha);
        p.setBrush(fill);
        p.drawPath(sectorWedge(c, r - 10.0, r, i * 60.0, 27.0));
    }

    // Degree grid. Angles the turret cannot reach are drawn dimmer, and the
    // reachable sector is outlined on the ring.
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(QColor(56, 56, 62), 1.0));
    p.drawEllipse(ring);
    p.setPen(QPen(QColor(92, 92, 100), 2.0));
    p.drawArc(ring, static_cast<int>((90.0 + kYawRangeDeg) * 16),
              static_cast<int>(-2.0 * kYawRangeDeg * 16));

    for (int deg = -180; deg < 180; deg += 15) {
        const bool major = deg % 45 == 0;
        const QPointF d = dirAt(deg);
        QColor tick = major ? QColor(120, 120, 128) : QColor(76, 76, 84);
        if (std::abs(deg) > kYawRangeDeg) tick = tick.darker(165);
        p.setPen(QPen(tick, major ? 1.4 : 1.0));
        p.drawLine(c + d * (r - (major ? 9.0 : 5.0)), c + d * r);
    }

    p.setFont(QFont(QStringLiteral("Consolas"), 7));
    for (int deg = -135; deg <= 180; deg += 45) {
        const QPointF at = c + dirAt(deg) * (r - 19.0);
        p.setPen(std::abs(deg) > kYawRangeDeg ? QColor(84, 84, 90) : QColor(142, 142, 150));
        p.drawText(QRectF(at.x() - 16, at.y() - 7, 32, 14), Qt::AlignCenter,
                   QString::number(deg));
    }

    // Chassis: fixed, always facing forward.
    p.setPen(QPen(QColor(104, 104, 112), 1.0));
    p.setBrush(QColor(44, 44, 50));
    p.drawRoundedRect(QRectF(c.x() - 13, c.y() - 19, 26, 38), 4, 4);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(72, 72, 80));
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            p.drawRoundedRect(
                QRectF(c.x() + sx * 13.0 - (sx > 0 ? 0.0 : 5.0), c.y() + sy * 10.0 - 5.5, 5, 11),
                2, 2);
        }
    }
    p.setBrush(QColor(122, 122, 132));
    QPolygonF nose;
    nose << QPointF(c.x(), c.y() - 25.5) << QPointF(c.x() - 4.5, c.y() - 20.0)
         << QPointF(c.x() + 4.5, c.y() - 20.0);
    p.drawPolygon(nose);

    const double desired_deg = desired_ * kYawRangeDeg;
    const double actual_deg = actual_ * kYawRangeDeg;

    // Turret follows the reported angle; with no telemetry it sits at zero, greyed.
    p.save();
    p.translate(c);
    p.rotate(has_actual_ ? actual_deg : 0.0);
    const QColor turret = has_actual_ ? QColor(96, 172, 122) : QColor(78, 78, 86);
    p.setPen(QPen(turret.darker(135), 1.0));
    p.setBrush(QColor(54, 54, 60));
    p.drawEllipse(QPointF(0, 0), 11, 11);
    p.setPen(Qt::NoPen);
    p.setBrush(turret);
    p.drawRoundedRect(QRectF(-3.5, -22, 7, 16), 2, 2);
    p.restore();

    const double tip = r - 4.0;
    p.setPen(QPen(QColor(224, 72, 72), 2.0, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(c + dirAt(desired_deg) * 13.0, c + dirAt(desired_deg) * tip);
    if (has_actual_) {
        p.setPen(QPen(QColor(80, 200, 120), 2.0, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(c + dirAt(actual_deg) * 13.0, c + dirAt(actual_deg) * tip);
    }

    p.setFont(QFont(QStringLiteral("Consolas"), 8));
    const int fy = height() - kFooter - 2;
    p.setPen(QColor(224, 72, 72));
    p.drawText(QRect(10, fy, width() / 2 - 12, kFooter), Qt::AlignLeft | Qt::AlignVCenter,
               QStringLiteral("CMD ") + formatDeg(desired_deg));
    p.setPen(has_actual_ ? QColor(80, 200, 120) : QColor(110, 110, 115));
    p.drawText(QRect(width() / 2, fy, width() / 2 - 10, kFooter), Qt::AlignRight | Qt::AlignVCenter,
               QStringLiteral("ACT ") +
                   (has_actual_ ? formatDeg(actual_deg) : QStringLiteral("--")));
}
