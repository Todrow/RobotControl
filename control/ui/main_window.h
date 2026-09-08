#pragma once
#include <QElapsedTimer>
#include <QMainWindow>
#include <QPoint>
#include <deque>

#include "../input/input_controller.h"
#include "../net/connection_manager.h"
#include "../state/desired_state_slot.h"
#include "../video/video_receiver.h"

class HudOverlay;
class YawIndicator;
class QDial;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QTimer;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void showEvent(QShowEvent* event) override;
    // The video widget owns a native window of its own, so its input events are
    // filtered here instead of relying on propagation to the main window.
    bool eventFilter(QObject* watched, QEvent* event) override;

private slots:
    void onConnectClicked();
    void onPowerChanged(int percent);
    void onTelemetry(proto::Telemetry telemetry);
    void onCommandStatus(bool connected);
    void onTelemetryStatus(bool connected);
    void tick();  // telemetry staleness + link quality

private:
    void buildConnectBar();
    void buildPowerPanel();
    void layoutOverlays();
    void updateVideoRect();
    void handleKey(QKeyEvent* event, bool pressed);
    void handleMouseMove(const QPoint& pos);
    void setMouseCaptured(bool captured);
    void updateQuality();

    DesiredStateSlot slot_;
    InputController input_;
    ConnectionManager conn_;
    VideoReceiver video_;

    QWidget* video_widget_ = nullptr;
    QWidget* bar_ = nullptr;
    QWidget* power_panel_ = nullptr;
    QDial* power_dial_ = nullptr;
    QLabel* power_label_ = nullptr;
    HudOverlay* hud_ = nullptr;
    YawIndicator* yaw_view_ = nullptr;
    QLineEdit* host_edit_ = nullptr;
    QSpinBox* command_port_ = nullptr;
    QSpinBox* telemetry_port_ = nullptr;
    QSpinBox* video_port_ = nullptr;
    QPushButton* connect_button_ = nullptr;
    QTimer* tick_timer_ = nullptr;

    QElapsedTimer uptime_;
    std::deque<qint64> arrivals_;  // telemetry arrival times, for link quality
    qint64 connected_at_ = -1;
    qint64 last_telemetry_ = -1;
    bool command_up_ = false;
    bool telemetry_up_ = false;
    bool mouse_captured_ = false;
};
