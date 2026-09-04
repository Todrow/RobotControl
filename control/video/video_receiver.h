#pragma once
#include <QWidget>
#include <gst/gst.h>

// RTP/H264 over UDP -> decoded straight into a native Qt widget via
// GstVideoOverlay, so no frame ever crosses into Qt's paint path.
class VideoReceiver : public QObject {
    Q_OBJECT

public:
    explicit VideoReceiver(QObject* parent = nullptr);
    ~VideoReceiver() override;

    bool start(quint16 port, WId window);
    void stop();
    bool running() const { return pipeline_ != nullptr; }
    void setRenderRect(int width, int height);

private:
    static GstBusSyncReply busSync(GstBus* bus, GstMessage* msg, gpointer user_data);

    GstElement* pipeline_ = nullptr;
    GstElement* sink_ = nullptr;
    guintptr window_ = 0;
};
