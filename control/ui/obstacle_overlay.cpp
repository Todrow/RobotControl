#include "obstacle_overlay.h"

#include <QPainter>
#include <QStringList>
#include <QTimer>

#include "../../common/protocol.h"

namespace {
constexpr int kBlinkPeriodMs = 450;  // on/off toggle interval

// Human-readable direction list from the mask, in Russian for the operator.
QString describe(uint8_t mask) {
    QStringList parts;
    if (mask & proto::OBSTACLE_FRONT) parts << QStringLiteral("СПЕРЕДИ");
    if (mask & proto::OBSTACLE_BACK)  parts << QStringLiteral("СЗАДИ");
    if (mask & proto::OBSTACLE_LEFT)  parts << QStringLiteral("СЛЕВА");
    if (mask & proto::OBSTACLE_RIGHT) parts << QStringLiteral("СПРАВА");
    if (parts.isEmpty()) return QString();
    return QStringLiteral("ПОМЕХА ") + parts.join(QStringLiteral(" • "));
}
}  // namespace

ObstacleOverlay::ObstacleOverlay(QWidget* parent) : QWidget(parent) {
    setAttribute(Qt::WA_TransparentForMouseEvents);
    setFocusPolicy(Qt::NoFocus);
    hide();  // nothing to show until an obstacle appears

    blink_timer_ = new QTimer(this);
    blink_timer_->setInterval(kBlinkPeriodMs);
    connect(blink_timer_, &QTimer::timeout, this, [this] {
        blink_on_ = !blink_on_;
        update();
    });
}

void ObstacleOverlay::setObstacles(uint8_t mask) {
    if (mask == mask_) return;
    mask_ = mask;
    if (mask_ == proto::OBSTACLE_NONE) {
        clearObstacles();
        return;
    }
    blink_on_ = true;
    if (!blink_timer_->isActive()) blink_timer_->start();
    show();
    raise();
    update();
}

void ObstacleOverlay::clearObstacles() {
    mask_ = 0;
    blink_timer_->stop();
    hide();
}

void ObstacleOverlay::paintEvent(QPaintEvent*) {
    if (mask_ == 0) return;

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    const QString text = describe(mask_);
    if (text.isEmpty()) return;

    // Banner box centred horizontally, in the upper third of the screen so it
    // does not hide what is straight ahead in the video.
    QFont font(QStringLiteral("Consolas"), 22, QFont::Bold);
    p.setFont(font);
    const QFontMetrics fm(font);
    const int text_w = fm.horizontalAdvance(text);
    const int box_w = text_w + 56;
    const int box_h = fm.height() + 28;
    const int x = (width() - box_w) / 2;
    const int y = height() / 6;

    // Blink: alternate between a bright and a dim red so it is impossible to miss.
    const int alpha = blink_on_ ? 235 : 90;
    const QColor fill(190, 30, 30, alpha);
    const QColor border(255, 90, 90, blink_on_ ? 255 : 140);
    const QColor textColor(255, 255, 255, blink_on_ ? 255 : 170);

    p.setBrush(fill);
    p.setPen(QPen(border, 3));
    p.drawRoundedRect(QRectF(x, y, box_w, box_h), 8, 8);

    p.setPen(textColor);
    p.drawText(QRect(x, y, box_w, box_h), Qt::AlignCenter, text);
}
