#pragma once
#include <QWidget>

#include "../net/map_client.h"

// The SLAM map, drawn over the video.
//
// Shows only what the robot actually knows: unknown space stays dark and is
// never filled in as floor. That distinction is the whole point of the display
// -- the operator has to be able to tell "there is nothing there" from "nobody
// has looked there yet", because the second one is where the robot is going
// next.
class MapView : public QWidget {
    Q_OBJECT

public:
    explicit MapView(QWidget* parent = nullptr);

    void setMap(const MapFrame& frame);
    void clearMap();
    void setLinkStatus(bool connected);
    void setExploring(bool exploring);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    // Metres of map shown across the widget. Wide enough to see where the robot
    // has been, tight enough that a doorway is still a visible gap.
    static constexpr float kViewMetres = 12.0f;

    MapFrame map_;
    bool have_map_ = false;
    bool connected_ = false;
    bool exploring_ = false;
};
