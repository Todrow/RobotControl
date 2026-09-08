#include "main_window.h"

#include <QCursor>
#include <QDial>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>

#include "hud_overlay.h"
#include "yaw_indicator.h"

namespace {
constexpr int kTelemetryStaleMs = 1000;
constexpr int kQualityWindowMs = 2000;
constexpr int kMargin = 12;

const char* kPanelStyle =
    "QWidget { background: rgba(20,20,22,235); color: #e4e4e8; }"
    "QLineEdit, QSpinBox { background: #2a2a2e; border: 1px solid #45454a;"
    " padding: 3px 5px; color: #e4e4e8; }"
    "QPushButton { background: #33333a; border: 1px solid #55555c; padding: 4px 14px; }"
    "QPushButton:hover { background: #3d3d45; }"
    "QLabel { color: #9a9aa0; }";

// Qt::Key follows the keyboard layout, so on a Cyrillic layout W arrives as a
// Cyrillic key and WASD would silently stop working. Virtual-key codes do not.
int virtualKey(const QKeyEvent* event) {
    if (event->nativeVirtualKey()) return static_cast<int>(event->nativeVirtualKey());
    switch (event->key()) {  // synthetic events carry no native code
        case Qt::Key_W: return vkey::W;
        case Qt::Key_A: return vkey::A;
        case Qt::Key_S: return vkey::S;
        case Qt::Key_D: return vkey::D;
        case Qt::Key_Up: return vkey::Up;
        case Qt::Key_Down: return vkey::Down;
        case Qt::Key_Left: return vkey::Left;
        case Qt::Key_Right: return vkey::Right;
        default: return 0;
    }
}
}  // namespace

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), input_(slot_), conn_(slot_, this), video_(this) {
    setWindowTitle(QStringLiteral("Robot Control"));
    uptime_.start();

    video_widget_ = new QWidget(this);
    video_widget_->setAttribute(Qt::WA_NativeWindow);
    video_widget_->setStyleSheet(QStringLiteral("background: #000;"));
    video_widget_->setFocusPolicy(Qt::StrongFocus);
    video_widget_->setMouseTracking(true);
    video_widget_->installEventFilter(this);
    setCentralWidget(video_widget_);

    // The overlays are siblings of the video widget, not its children: the video
    // sink puts its own child window inside that widget and would cover them.
    buildConnectBar();
    buildPowerPanel();
    hud_ = new HudOverlay(this);
    hud_->setAttribute(Qt::WA_NativeWindow);
    hud_->show();
    yaw_view_ = new YawIndicator(this);
    yaw_view_->setAttribute(Qt::WA_NativeWindow);
    yaw_view_->setDesiredYaw(slot_.get().camera.yaw);
    yaw_view_->show();

    connect(&conn_, &ConnectionManager::commandStatusChanged, this, &MainWindow::onCommandStatus,
            Qt::QueuedConnection);
    connect(&conn_, &ConnectionManager::telemetryStatusChanged, this,
            &MainWindow::onTelemetryStatus, Qt::QueuedConnection);
    connect(conn_.telemetry(), &TelemetryReceiver::telemetryReceived, this,
            &MainWindow::onTelemetry, Qt::QueuedConnection);

    tick_timer_ = new QTimer(this);
    connect(tick_timer_, &QTimer::timeout, this, &MainWindow::tick);
    tick_timer_->start(250);

    resize(1280, 720);
}

MainWindow::~MainWindow() {
    video_.stop();
    conn_.stop();
}

void MainWindow::buildConnectBar() {
    bar_ = new QWidget(this);
    bar_->setAttribute(Qt::WA_NativeWindow);
    bar_->setStyleSheet(QString::fromLatin1(kPanelStyle));

    host_edit_ = new QLineEdit(QStringLiteral("127.0.0.1"), bar_);
    host_edit_->setFixedWidth(110);

    const auto makePort = [this](int value) {
        auto* box = new QSpinBox(bar_);
        box->setRange(1, 65535);
        box->setValue(value);
        box->setFixedWidth(64);
        box->setButtonSymbols(QAbstractSpinBox::NoButtons);
        return box;
    };
    command_port_ = makePort(proto::COMMAND_PORT);
    telemetry_port_ = makePort(proto::TELEMETRY_PORT);
    video_port_ = makePort(proto::VIDEO_PORT);

    connect_button_ = new QPushButton(QStringLiteral("Connect"), bar_);
    connect(connect_button_, &QPushButton::clicked, this, &MainWindow::onConnectClicked);

    auto* layout = new QHBoxLayout(bar_);
    layout->setContentsMargins(10, 6, 10, 6);
    layout->setSpacing(6);
    layout->addWidget(new QLabel(QStringLiteral("IP"), bar_));
    layout->addWidget(host_edit_);
    layout->addWidget(new QLabel(QStringLiteral("cmd"), bar_));
    layout->addWidget(command_port_);
    layout->addWidget(new QLabel(QStringLiteral("tlm"), bar_));
    layout->addWidget(telemetry_port_);
    layout->addWidget(new QLabel(QStringLiteral("video"), bar_));
    layout->addWidget(video_port_);
    layout->addWidget(connect_button_);
    bar_->adjustSize();
    bar_->show();
}

void MainWindow::buildPowerPanel() {
    power_panel_ = new QWidget(this);
    power_panel_->setAttribute(Qt::WA_NativeWindow);
    power_panel_->setStyleSheet(QString::fromLatin1(kPanelStyle));

    auto* caption = new QLabel(QStringLiteral("POWER"), power_panel_);
    caption->setAlignment(Qt::AlignCenter);

    power_dial_ = new QDial(power_panel_);
    power_dial_->setRange(0, 100);  // percent on screen, 0.0..1.0 in the protocol
    power_dial_->setValue(static_cast<int>(input_.power() * 100.0f + 0.5f));
    power_dial_->setNotchesVisible(true);
    power_dial_->setFixedSize(84, 84);
    // Never take focus, or the arrow keys would turn the dial instead of driving.
    power_dial_->setFocusPolicy(Qt::NoFocus);
    connect(power_dial_, &QDial::valueChanged, this, &MainWindow::onPowerChanged);

    power_label_ = new QLabel(power_panel_);
    power_label_->setAlignment(Qt::AlignCenter);
    power_label_->setStyleSheet(QStringLiteral("QLabel { color: #e4e4e8; }"));

    auto* layout = new QVBoxLayout(power_panel_);
    layout->setContentsMargins(10, 8, 10, 8);
    layout->setSpacing(4);
    layout->addWidget(caption);
    layout->addWidget(power_dial_, 0, Qt::AlignHCenter);
    layout->addWidget(power_label_);
    power_panel_->adjustSize();
    power_panel_->show();

    onPowerChanged(power_dial_->value());
}

void MainWindow::onPowerChanged(int percent) {
    input_.setPower(percent / 100.0f);
    power_label_->setText(QString::asprintf("%d %%", percent));
}

void MainWindow::layoutOverlays() {
    if (!video_widget_) return;
    const QRect area = video_widget_->geometry();
    if (bar_) {
        bar_->move(area.left() + kMargin, area.top() + kMargin);
        bar_->raise();
    }
    if (power_panel_) {
        power_panel_->move(area.right() - power_panel_->width() - kMargin,
                           area.top() + (area.height() - power_panel_->height()) / 2);
        power_panel_->raise();
    }
    if (hud_) {
        hud_->move(area.left() + kMargin, area.bottom() - hud_->height() - kMargin);
        hud_->raise();
        hud_->update();
    }
    if (yaw_view_) {
        yaw_view_->move(area.right() - yaw_view_->width() - kMargin,
                        area.bottom() - yaw_view_->height() - kMargin);
        yaw_view_->raise();
        yaw_view_->update();
    }
}

// GstVideoOverlay's render rectangle is in native window pixels, but Qt reports
// widget sizes in logical ones. On a scaled display the two differ, and passing
// the logical size lays the picture out into a fraction of the window, leaving
// dead space along the right and bottom edges.
void MainWindow::updateVideoRect() {
    if (!video_widget_) return;
    const qreal dpr = video_widget_->devicePixelRatioF();
    video_.setRenderRect(qRound(video_widget_->width() * dpr),
                         qRound(video_widget_->height() * dpr));
}

void MainWindow::resizeEvent(QResizeEvent* event) {
    QMainWindow::resizeEvent(event);
    layoutOverlays();
    updateVideoRect();
}

void MainWindow::showEvent(QShowEvent* event) {
    QMainWindow::showEvent(event);
    layoutOverlays();  // without this the overlays sit unpositioned until the first resize
}

void MainWindow::onConnectClicked() {
    if (conn_.running()) {
        video_.stop();
        conn_.stop();
        setMouseCaptured(false);
        connected_at_ = -1;
        last_telemetry_ = -1;
        arrivals_.clear();
        hud_->clearTelemetry();
        hud_->setLinkQuality(-1);
        yaw_view_->clearActualYaw();
        connect_button_->setText(QStringLiteral("Connect"));
        return;
    }

    conn_.start(host_edit_->text().trimmed(), static_cast<uint16_t>(command_port_->value()),
                static_cast<uint16_t>(telemetry_port_->value()));
    video_.start(static_cast<quint16>(video_port_->value()), video_widget_->winId());
    updateVideoRect();
    connected_at_ = uptime_.elapsed();
    arrivals_.clear();
    connect_button_->setText(QStringLiteral("Disconnect"));
    video_widget_->setFocus();
    layoutOverlays();
    // The sink creates its window a moment later, on top of ours: claim the
    // z-order back once it exists.
    QTimer::singleShot(700, this, [this] { layoutOverlays(); });
}

void MainWindow::onTelemetry(proto::Telemetry telemetry) {
    last_telemetry_ = uptime_.elapsed();
    arrivals_.push_back(last_telemetry_);
    if (arrivals_.size() > 2 * static_cast<size_t>(kQualityWindowMs / proto::TELEMETRY_PERIOD_MS))
        arrivals_.pop_front();
    hud_->setTelemetry(telemetry.cpu_temp, telemetry.battery_level);
    yaw_view_->setActualYaw(telemetry.camera.yaw);
    yaw_view_->setSectors(telemetry.sectors);
    updateQuality();
}

void MainWindow::onCommandStatus(bool connected) {
    command_up_ = connected;
    hud_->setLinkStatus(command_up_, telemetry_up_);
    updateQuality();
}

void MainWindow::onTelemetryStatus(bool connected) {
    telemetry_up_ = connected;
    hud_->setLinkStatus(command_up_, telemetry_up_);
    if (!connected) {
        last_telemetry_ = -1;
        arrivals_.clear();
        hud_->clearTelemetry();
        yaw_view_->clearActualYaw();
        yaw_view_->clearSectors();
    }
    updateQuality();
}

// Quality = share of the telemetry that actually arrived in the last window.
// The protocol carries no sequence numbers or timestamps, so the fixed 100 ms
// send rate is the only reference available.
void MainWindow::updateQuality() {
    if (!conn_.running() || connected_at_ < 0 || !command_up_ || !telemetry_up_) {
        hud_->setLinkQuality(conn_.running() ? 0 : -1);
        return;
    }

    const qint64 now = uptime_.elapsed();
    while (!arrivals_.empty() && arrivals_.front() < now - kQualityWindowMs) arrivals_.pop_front();

    // Right after connecting the window is still filling up, so measure only the
    // time actually spent connected.
    const qint64 window = std::min<qint64>(kQualityWindowMs, now - connected_at_);
    if (window < proto::TELEMETRY_PERIOD_MS) return;  // too early to judge

    const int expected = static_cast<int>(window / proto::TELEMETRY_PERIOD_MS);
    const int received = static_cast<int>(arrivals_.size());
    hud_->setLinkQuality(std::clamp(received * 100 / std::max(1, expected), 0, 100));
}

void MainWindow::tick() {
    const bool fresh = last_telemetry_ >= 0 && uptime_.elapsed() - last_telemetry_ < kTelemetryStaleMs;
    if (!fresh) {
        hud_->clearTelemetry();
        yaw_view_->clearActualYaw();
    }
    yaw_view_->setDesiredYaw(slot_.get().camera.yaw);
    updateQuality();
}

void MainWindow::handleKey(QKeyEvent* event, bool pressed) {
    // On a modifier release Qt may still report it in modifiers(), so the Shift
    // key itself is judged by the event type.
    const bool shift = event->key() == Qt::Key_Shift
                           ? pressed
                           : event->modifiers().testFlag(Qt::ShiftModifier);
    input_.setShift(shift);

    if (event->isAutoRepeat() && !pressed) return;
    const int vk = virtualKey(event);
    const bool handled = pressed ? input_.keyPress(vk, event->isAutoRepeat())
                                 : input_.keyRelease(vk);
    if (handled) return;
    if (pressed)
        QMainWindow::keyPressEvent(event);
    else
        QMainWindow::keyReleaseEvent(event);
}

void MainWindow::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        setMouseCaptured(false);
        return;
    }
    if (event->key() == Qt::Key_F11) {
        if (isFullScreen())
            showNormal();
        else
            showFullScreen();
        return;
    }
    handleKey(event, true);
}

void MainWindow::keyReleaseEvent(QKeyEvent* event) { handleKey(event, false); }

bool MainWindow::eventFilter(QObject* watched, QEvent* event) {
    if (watched != video_widget_) return QMainWindow::eventFilter(watched, event);

    switch (event->type()) {
        // The central widget only gets its real geometry from the layout pass
        // that runs after showEvent. Everything except the connect bar is
        // placed from area.right()/bottom(), so laying out against the
        // pre-layout size pushes those overlays off the window entirely.
        case QEvent::Resize:
            layoutOverlays();
            break;
        case QEvent::MouseMove:
            handleMouseMove(static_cast<QMouseEvent*>(event)->pos());
            return true;
        case QEvent::MouseButtonPress:
            if (static_cast<QMouseEvent*>(event)->button() == Qt::LeftButton) {
                setMouseCaptured(true);
                return true;
            }
            break;
        case QEvent::KeyPress:
            keyPressEvent(static_cast<QKeyEvent*>(event));
            return true;
        case QEvent::KeyRelease:
            keyReleaseEvent(static_cast<QKeyEvent*>(event));
            return true;
        case QEvent::FocusOut:
            setMouseCaptured(false);
            break;
        default:
            break;
    }
    return QMainWindow::eventFilter(watched, event);
}

void MainWindow::handleMouseMove(const QPoint& pos) {
    // The camera only moves while the pointer is captured; with the cursor
    // visible, mouse movement is ignored.
    if (!mouse_captured_) return;

    // Warp back to the centre after every move, so the camera keeps turning
    // instead of stopping when the pointer hits the window edge.
    const QPoint center(video_widget_->width() / 2, video_widget_->height() / 2);
    const QPoint delta = pos - center;
    if (delta.isNull()) return;
    input_.mouseDelta(delta.x(), delta.y());
    yaw_view_->setDesiredYaw(slot_.get().camera.yaw);
    QCursor::setPos(video_widget_->mapToGlobal(center));
}

void MainWindow::setMouseCaptured(bool captured) {
    if (captured == mouse_captured_) return;
    mouse_captured_ = captured;

    if (captured) {
        video_widget_->setCursor(Qt::BlankCursor);
        video_widget_->setFocus();
        QCursor::setPos(video_widget_->mapToGlobal(
            QPoint(video_widget_->width() / 2, video_widget_->height() / 2)));
    } else {
        video_widget_->unsetCursor();
    }
}
