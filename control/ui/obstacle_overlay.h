#pragma once
#include <QWidget>
#include <cstdint>

class QTimer;

// Full-window transparent overlay that flashes a red warning banner in the
// centre of the screen while the robot reports obstacles. Driven by the
// obstacle bitmask (proto::ObstacleFlags) from telemetry. When the mask is 0
// the widget hides itself and stops the blink timer.
class ObstacleOverlay : public QWidget {
    Q_OBJECT

public:
    explicit ObstacleOverlay(QWidget* parent = nullptr);

    // Update from the latest telemetry mask (proto::ObstacleFlags bits).
    void setObstacles(uint8_t mask);
    // Clear the warning (e.g. telemetry link lost or stale).
    void clearObstacles();

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    uint8_t mask_ = 0;
    bool blink_on_ = true;
    QTimer* blink_timer_ = nullptr;
};
