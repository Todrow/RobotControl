#include "hud_overlay.h"

#include <QPainter>
#include <cmath>

namespace {
constexpr int kWidth = 232;
constexpr int kHeight = 128;
constexpr int kRowHeight = 22;

QColor qualityColor(int percent) {
    if (percent < 0) return QColor(150, 150, 155);
    if (percent >= 85) return QColor(80, 200, 120);
    if (percent >= 50) return QColor(220, 180, 70);
    return QColor(200, 70, 70);
}
}  // namespace

HudOverlay::HudOverlay(QWidget* parent) : QWidget(parent) {
    setFixedSize(kWidth, kHeight);
    setAttribute(Qt::WA_TransparentForMouseEvents);
    setFocusPolicy(Qt::NoFocus);
}

void HudOverlay::setTelemetry(float cpu_temp, float battery_level) {
    has_telemetry_ = true;
    cpu_temp_ = cpu_temp;
    battery_level_ = battery_level;
    update();
}

void HudOverlay::clearTelemetry() {
    if (!has_telemetry_) return;
    has_telemetry_ = false;
    update();
}

void HudOverlay::setLinkStatus(bool command_up, bool telemetry_up) {
    command_up_ = command_up;
    telemetry_up_ = telemetry_up;
    update();
}

void HudOverlay::setLinkQuality(int percent) {
    if (percent == quality_) return;
    quality_ = percent;
    update();
}

void HudOverlay::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(0, 0, 0));
    p.setBrush(QColor(24, 24, 26, 210));
    p.setPen(QColor(70, 70, 74));
    p.drawRoundedRect(QRectF(0.5, 0.5, width() - 1.0, height() - 1.0), 4, 4);
    p.setFont(QFont(QStringLiteral("Consolas"), 10));

    const QString dash = QStringLiteral("--");
    const auto row = [&](int index, const QString& label, const QString& value, const QColor& color) {
        const int y = 8 + index * kRowHeight;
        p.setPen(QColor(150, 150, 155));
        p.drawText(QRect(12, y, 90, kRowHeight - 2), Qt::AlignLeft | Qt::AlignVCenter, label);
        p.setPen(color);
        p.drawText(QRect(102, y, kWidth - 114, kRowHeight - 2), Qt::AlignRight | Qt::AlignVCenter,
                   value);
    };

    const QColor white(228, 228, 232);
    row(0, QStringLiteral("CPU"),
        has_telemetry_ && std::isfinite(cpu_temp_)
            ? QString::asprintf("%.1f C", cpu_temp_) : dash, white);
    row(1, QStringLiteral("BATTERY"),
        has_telemetry_ && std::isfinite(battery_level_)
            ? QString::asprintf("%.0f %%", battery_level_) : dash, white);
    row(2, QStringLiteral("QUALITY"),
        quality_ >= 0 ? QString::asprintf("%d %%", quality_) : dash, qualityColor(quality_));

    // link row: one dot per channel
    const int y = 8 + 3 * kRowHeight + 4;
    p.setPen(QColor(150, 150, 155));
    p.drawText(QRect(12, y, 90, kRowHeight - 2), Qt::AlignLeft | Qt::AlignVCenter,
               QStringLiteral("LINK"));
    const auto dot = [&](int x, bool up, const QString& label) {
        p.setBrush(up ? QColor(80, 200, 120) : QColor(200, 70, 70));
        p.setPen(Qt::NoPen);
        p.drawEllipse(QRectF(x, y + 6, 8, 8));
        p.setPen(QColor(180, 180, 185));
        p.drawText(QRect(x + 12, y, 40, kRowHeight - 2), Qt::AlignLeft | Qt::AlignVCenter, label);
    };
    dot(96, command_up_, QStringLiteral("CMD"));
    dot(160, telemetry_up_, QStringLiteral("TLM"));

    p.setPen(QColor(110, 110, 115));
    p.setFont(QFont(QStringLiteral("Consolas"), 8));
    p.drawText(QRect(12, kHeight - 22, kWidth - 24, 18), Qt::AlignLeft | Qt::AlignVCenter,
               QStringLiteral("LMB capture  ESC release  Shift boost"));
}
