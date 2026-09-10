#pragma once
#include <QWidget>

#include <array>

#include "../../common/protocol.h"

// Top-down yaw indicator: the body always points forward, the camera turret
// turns. Red = the yaw we are commanding, green = the yaw telemetry reports.
// The outer ring shows the robot's own obstacle verdict for each of the six
// sectors; this widget only paints what it is given and never applies a
// threshold of its own.
class YawIndicator : public QWidget {
    Q_OBJECT

public:
    explicit YawIndicator(QWidget* parent = nullptr);

    void setDesiredYaw(float yaw);  // -1.0 .. 1.0, as sent in DesiredState
    void setActualYaw(float yaw);   // -1.0 .. 1.0, as reported in Telemetry
    void clearActualYaw();

    void setSectors(const proto::SectorStatus (&sectors)[proto::SECTOR_COUNT]);
    void clearSectors();  // back to Unknown, e.g. when telemetry drops

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    float desired_ = 0.0f;
    float actual_ = 0.0f;
    bool has_actual_ = false;
    std::array<proto::SectorStatus, proto::SECTOR_COUNT> sectors_{};
};
