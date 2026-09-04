#pragma once
#include <QWidget>

// Top-down yaw indicator: the body always points forward, the camera turret
// turns. Red = the yaw we are commanding, green = the yaw telemetry reports.
class YawIndicator : public QWidget {
    Q_OBJECT

public:
    explicit YawIndicator(QWidget* parent = nullptr);

    void setDesiredYaw(float yaw);  // -1.0 .. 1.0, as sent in DesiredState
    void setActualYaw(float yaw);   // -1.0 .. 1.0, as reported in Telemetry
    void clearActualYaw();

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    float desired_ = 0.0f;
    float actual_ = 0.0f;
    bool has_actual_ = false;
};
