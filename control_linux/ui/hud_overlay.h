#pragma once
#include <QWidget>

// Dark HUD panel drawn on top of the video window.
class HudOverlay : public QWidget {
    Q_OBJECT

public:
    explicit HudOverlay(QWidget* parent = nullptr);

    void setTelemetry(float cpu_temp, float battery_level);
    void clearTelemetry();
    void setLinkStatus(bool command_up, bool telemetry_up);
    void setLinkQuality(int percent);  // -1 = unknown

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    bool has_telemetry_ = false;
    float cpu_temp_ = 0.0f;
    float battery_level_ = 0.0f;
    bool command_up_ = false;
    bool telemetry_up_ = false;
    int quality_ = -1;
};
