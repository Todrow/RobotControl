#include "video_receiver.h"

#include <QDebug>
#include <gst/video/videooverlay.h>

namespace {

// A named sink we can hand the window to up front is more reliable than
// autovideosink, whose real sink only appears once the pipeline is running.
const char* pickSink() {
    for (const char* name : {"d3d11videosink", "d3d12videosink", "glimagesink"}) {
        if (GstElementFactory* factory = gst_element_factory_find(name)) {
            gst_object_unref(factory);
            return name;
        }
    }
    return "autovideosink";
}

}  // namespace

VideoReceiver::VideoReceiver(QObject* parent) : QObject(parent) {}

VideoReceiver::~VideoReceiver() { stop(); }

GstBusSyncReply VideoReceiver::busSync(GstBus*, GstMessage* msg, gpointer user_data) {
    auto* self = static_cast<VideoReceiver*>(user_data);
    // Fallback for sinks that only ask for the handle once they are running
    // (autovideosink): the question comes from the streaming thread and must be
    // answered here, before the sink creates a window of its own.
    if (gst_is_video_overlay_prepare_window_handle_message(msg) && self->window_) {
        gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(GST_MESSAGE_SRC(msg)), self->window_);
        gst_message_unref(msg);
        return GST_BUS_DROP;
    }
    return GST_BUS_PASS;
}

bool VideoReceiver::start(quint16 port, WId window) {
    stop();
    window_ = (guintptr)window;  // WId is an integer on some Qt builds, HWND on others

    const QString desc =
        QStringLiteral(
            "udpsrc port=%1 caps=\"application/x-rtp,media=(string)video,"
            "clock-rate=(int)90000,encoding-name=(string)H264,payload=(int)96\" ! "
            "rtpjitterbuffer latency=100 ! rtph264depay ! h264parse ! decodebin ! "
            "videoconvert ! %2 name=vsink sync=false")
            .arg(port)
            .arg(QString::fromLatin1(pickSink()));

    GError* err = nullptr;
    pipeline_ = gst_parse_launch(desc.toUtf8().constData(), &err);
    if (!pipeline_) {
        qWarning() << "video: pipeline error:" << (err ? err->message : "unknown");
        if (err) g_error_free(err);
        return false;
    }
    if (err) g_error_free(err);

    sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "vsink");
    if (sink_ && GST_IS_VIDEO_OVERLAY(sink_))
        gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(sink_), window_);

    GstBus* bus = gst_element_get_bus(pipeline_);
    gst_bus_set_sync_handler(bus, &VideoReceiver::busSync, this, nullptr);
    gst_object_unref(bus);

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        qWarning() << "video: failed to start pipeline";
        stop();
        return false;
    }
    return true;
}

void VideoReceiver::setRenderRect(int width, int height) {
    if (!sink_ || !GST_IS_VIDEO_OVERLAY(sink_) || width <= 0 || height <= 0) return;
    GstVideoOverlay* overlay = GST_VIDEO_OVERLAY(sink_);
    gst_video_overlay_set_render_rectangle(overlay, 0, 0, width, height);
    gst_video_overlay_expose(overlay);
}

void VideoReceiver::stop() {
    if (!pipeline_) return;
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    if (sink_) {
        gst_object_unref(sink_);
        sink_ = nullptr;
    }
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
}
