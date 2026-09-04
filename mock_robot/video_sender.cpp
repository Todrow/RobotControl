#if !defined(_WIN32) && !defined(_GNU_SOURCE)
#define _GNU_SOURCE  // pipe2 and O_CLOEXEC on Raspberry Pi OS/glibc.
#endif

#include "robot_runtime.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <gst/gst.h>

#ifndef _WIN32
#include <cerrno>
#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;
#endif

namespace {

using Clock = std::chrono::steady_clock;
constexpr auto POLL_INTERVAL = std::chrono::milliseconds(100);
constexpr auto RETRY_INTERVAL = std::chrono::seconds(3);

bool hasElement(const char* name) {
    GstElementFactory* factory = gst_element_factory_find(name);
    if (!factory) return false;
    gst_object_unref(factory);
    return true;
}

std::string pickEncoder(const RobotOptions& options, std::string& raw_format) {
    const std::string kbps = std::to_string((static_cast<long long>(options.bitrate) + 999) / 1000);
    const std::string gop = std::to_string(options.framerate);
    raw_format = "I420";
    if (hasElement("x264enc"))
        return "x264enc tune=zerolatency speed-preset=veryfast bitrate=" + kbps +
               " key-int-max=" + gop + " bframes=0";
    if (hasElement("mfh264enc")) {
        // Media Foundation accepts NV12, unlike the software encoders below.
        raw_format = "NV12";
        return "mfh264enc bitrate=" + kbps + " low-latency=true gop-size=" + gop + " bframes=0";
    }
    if (hasElement("openh264enc"))
        return "openh264enc bitrate=" + std::to_string(options.bitrate) +
               " rate-control=bitrate gop-size=" + gop;
    return {};
}

#ifndef _WIN32
class CameraProcess {
public:
    CameraProcess() = default;
    CameraProcess(const CameraProcess&) = delete;
    CameraProcess& operator=(const CameraProcess&) = delete;
    ~CameraProcess() { stop(); }

    int outputFd() const { return output_fd_; }

    bool start(const RobotOptions& options) {
        // Pass an argument vector directly: camera settings are never
        // interpreted by a shell in this multithreaded process.
        std::vector<std::string> arguments{
            "rpicam-vid", "--camera", std::to_string(options.camera_index),
            "--nopreview", "--timeout", "0", "--codec", "h264", "--inline",
            "--width", std::to_string(options.width),
            "--height", std::to_string(options.height),
            "--framerate", std::to_string(options.framerate),
            "--bitrate", std::to_string(options.bitrate),
            "--intra", std::to_string(options.framerate),
            "--flush", "--output", "-",
        };
        std::vector<char*> argv;
        argv.reserve(arguments.size() + 1);
        for (auto& argument : arguments) argv.push_back(argument.data());
        argv.push_back(nullptr);

        int descriptors[2];
        // Raspberry Pi OS is Linux; atomically mark both pipe ends close-on-exec.
        if (pipe2(descriptors, O_CLOEXEC) != 0) {
            std::printf("[video] camera pipe: %s\n", std::strerror(errno));
            return false;
        }
        // Keep spawn actions valid even if the service started with stdin or
        // stdout closed and pipe2 reused one of their descriptor numbers.
        for (int& descriptor : descriptors) {
            if (descriptor > STDERR_FILENO) continue;
            const int duplicate = fcntl(descriptor, F_DUPFD_CLOEXEC, STDERR_FILENO + 1);
            if (duplicate < 0) {
                std::printf("[video] camera pipe descriptor: %s\n", std::strerror(errno));
                close(descriptors[0]);
                close(descriptors[1]);
                return false;
            }
            close(descriptor);
            descriptor = duplicate;
        }

        posix_spawn_file_actions_t actions;
        int error = posix_spawn_file_actions_init(&actions);
        if (error != 0) {
            close(descriptors[0]);
            close(descriptors[1]);
            std::printf("[video] camera spawn actions: %s\n", std::strerror(error));
            return false;
        }
        error = posix_spawn_file_actions_adddup2(&actions, descriptors[1], STDOUT_FILENO);
        if (error == 0) error = posix_spawn_file_actions_addclose(&actions, descriptors[0]);
        if (error == 0) error = posix_spawn_file_actions_addclose(&actions, descriptors[1]);
        if (error == 0)
            error = posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);

        posix_spawnattr_t attributes;
        const int attribute_error = posix_spawnattr_init(&attributes);
        if (error == 0) error = attribute_error;
        if (error == 0) {
            sigset_t defaults;
            sigemptyset(&defaults);
            sigaddset(&defaults, SIGINT);
            sigaddset(&defaults, SIGTERM);
            sigaddset(&defaults, SIGPIPE);
            sigaddset(&defaults, SIGHUP);
            sigset_t mask;
            sigemptyset(&mask);
            error = posix_spawnattr_setsigdefault(&attributes, &defaults);
            if (error == 0) error = posix_spawnattr_setsigmask(&attributes, &mask);
            if (error == 0)
                error = posix_spawnattr_setflags(&attributes,
                                               POSIX_SPAWN_SETSIGDEF | POSIX_SPAWN_SETSIGMASK);
            if (error == 0)
                error = posix_spawnp(&pid_, argv[0], &actions, &attributes, argv.data(), environ);
        }
        if (attribute_error == 0) posix_spawnattr_destroy(&attributes);
        posix_spawn_file_actions_destroy(&actions);
        close(descriptors[1]);
        if (error != 0) {
            close(descriptors[0]);
            pid_ = -1;
            std::printf("[video] cannot start rpicam-vid: %s (check rpicam-apps installation)\n",
                        std::strerror(error));
            return false;
        }
        output_fd_ = descriptors[0];
        return true;
    }

    bool running(bool report = true) {
        if (pid_ <= 0) return false;
        int status = 0;
        pid_t result;
        do {
            result = waitpid(pid_, &status, WNOHANG);
        } while (result < 0 && errno == EINTR);
        if (result == 0) return true;
        if (result == pid_) {
            if (report) {
                if (WIFEXITED(status))
                    std::printf("[video] rpicam-vid exited with status %d; see camera log above\n",
                                WEXITSTATUS(status));
                else if (WIFSIGNALED(status))
                    std::printf("[video] rpicam-vid stopped by signal %d\n", WTERMSIG(status));
            }
            pid_ = -1;
        } else if (result < 0 && errno == ECHILD) {
            pid_ = -1;
        } else if (result < 0 && report) {
            std::printf("[video] camera waitpid: %s\n", std::strerror(errno));
        }
        return false;
    }

    void stop() {
        if (pid_ > 0) {
            if (running(false)) kill(pid_, SIGTERM);
            const auto deadline = Clock::now() + std::chrono::seconds(1);
            while (running(false) && Clock::now() < deadline)
                std::this_thread::sleep_for(std::chrono::milliseconds(25));
            if (pid_ > 0) {
                kill(pid_, SIGKILL);
                int status = 0;
                while (waitpid(pid_, &status, 0) < 0 && errno == EINTR) {}
                pid_ = -1;
            }
        }
        if (output_fd_ >= 0) {
            close(output_fd_);
            output_fd_ = -1;
        }
    }

private:
    pid_t pid_ = -1;
    int output_fd_ = -1;
};
#endif

class VideoStream {
public:
    VideoStream() = default;
    VideoStream(const VideoStream&) = delete;
    VideoStream& operator=(const VideoStream&) = delete;
    ~VideoStream() {
        // fdsrc must stop reading before CameraProcess closes its descriptor.
        if (pipeline_) gst_element_set_state(pipeline_, GST_STATE_NULL);
        if (bus_) gst_object_unref(bus_);
        if (pipeline_) gst_object_unref(pipeline_);
    }

    bool start(const RobotOptions& options, const std::string& host) {
        const std::string dimensions = "width=" + std::to_string(options.width) +
                                       ",height=" + std::to_string(options.height) +
                                       ",framerate=" + std::to_string(options.framerate) + "/1";
        std::string source;
#ifndef _WIN32
        if (options.video_source == VideoSource::Camera) {
            if (!camera_.start(options)) return false;
            uses_camera_process_ = true;
            // A pipe has arbitrary byte boundaries, not one access unit per
            // read. h264parse restores frame boundaries and frame durations.
            source = "fdsrc do-timestamp=true timeout=10000000 fd=" +
                     std::to_string(camera_.outputFd()) +
                     " ! video/x-h264,stream-format=byte-stream," + dimensions;
        } else
#endif
        {
            std::string raw_format;
            const std::string encoder = pickEncoder(options, raw_format);
            if (encoder.empty()) {
                std::printf("[video] no H.264 encoder found (x264enc, mfh264enc or openh264enc)\n");
                return false;
            }
            if (options.video_source == VideoSource::Test) {
                source = "videotestsrc is-live=true pattern=ball";
            } else {
#ifdef _WIN32
                source = "ksvideosrc device-index=" + std::to_string(options.camera_index);
#else
                return false;
#endif
            }
            source += " ! videoconvert ! videoscale ! videorate ! video/x-raw,format=" +
                      raw_format + "," + dimensions + " ! " + encoder;
        }

        const std::string description = source +
            " ! h264parse ! video/x-h264,stream-format=byte-stream,alignment=au"
            " ! rtph264pay config-interval=-1 pt=96 mtu=1200"
            " ! udpsink name=network sync=false async=false";
        GError* error = nullptr;
        pipeline_ = gst_parse_launch(description.c_str(), &error);
        // gst_parse_launch can return a partial pipeline together with an error.
        if (error || !pipeline_) {
            std::printf("[video] pipeline: %s\n", error ? error->message : "parse failed");
            if (error) g_error_free(error);
            return false;
        }
        GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline_), "network");
        if (!sink) {
            std::printf("[video] pipeline has no network sink\n");
            return false;
        }
        // Host is a GObject string property, never part of pipeline syntax.
        g_object_set(sink, "host", host.c_str(), "port", static_cast<gint>(options.video_port), nullptr);
        gst_object_unref(sink);
        bus_ = gst_element_get_bus(pipeline_);
        if (!bus_) {
            std::printf("[video] pipeline has no bus\n");
            return false;
        }
        if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            std::printf("[video] cannot start pipeline\n");
            poll();
            return false;
        }
        std::printf("[video] starting %s %dx%d@%d -> %s:%u (RTP/H.264, PT=96)\n",
                    options.video_source == VideoSource::Test ? "test source" : "camera",
                    options.width, options.height, options.framerate,
                    host.c_str(), static_cast<unsigned>(options.video_port));
        return true;
    }

    bool poll() {
        GstMessage* message = gst_bus_timed_pop_filtered(
            bus_, 100 * GST_MSECOND,
            static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_ELEMENT));
        if (message) {
            bool healthy = true;
            if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
                GError* error = nullptr;
                gchar* debug = nullptr;
                gst_message_parse_error(message, &error, &debug);
                std::printf("[video] %s: %s\n", GST_OBJECT_NAME(GST_MESSAGE_SRC(message)),
                            error ? error->message : "unknown stream error");
                if (debug && *debug) std::printf("[video] details: %s\n", debug);
                if (error) g_error_free(error);
                g_free(debug);
                healthy = false;
            } else if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_EOS) {
                std::printf("[video] stream ended\n");
                healthy = false;
            } else {
                const GstStructure* structure = gst_message_get_structure(message);
                if (structure && gst_structure_has_name(structure, "GstFdSrcTimeout")) {
                    std::printf("[video] camera produced no video for 10 seconds\n");
                    healthy = false;
                }
            }
            gst_message_unref(message);
            if (!healthy) return false;
        }
#ifndef _WIN32
        if (uses_camera_process_ && !camera_.running()) return false;
#endif
        return true;
    }

private:
    GstElement* pipeline_ = nullptr;
    GstBus* bus_ = nullptr;
#ifndef _WIN32
    CameraProcess camera_;
    bool uses_camera_process_ = false;
#endif
};

}  // namespace

void runVideoSender(const RobotOptions& options, RobotRuntime& runtime, const VideoTarget& target) {
    if (options.video_source == VideoSource::Disabled) {
        std::printf("[video] disabled\n");
        return;
    }
    bool waiting_reported = false;
    while (runtime.running.load()) {
        const std::string host = target.host();
        if (host.empty()) {
            if (!waiting_reported) std::printf("[video] waiting for command client\n");
            waiting_reported = true;
            std::this_thread::sleep_for(POLL_INTERVAL);
            continue;
        }
        waiting_reported = false;
        bool stream_failed = false;
        {
            VideoStream stream;
            stream_failed = !stream.start(options, host);
            while (!stream_failed && runtime.running.load() && target.host() == host)
                stream_failed = !stream.poll();
        }
        if (!runtime.running.load()) break;
        if (target.host() != host) continue;
        if (stream_failed) {
            std::printf("[video] retrying in 3 seconds; command and telemetry services remain active\n");
            const auto retry_at = Clock::now() + RETRY_INTERVAL;
            while (runtime.running.load() && target.host() == host && Clock::now() < retry_at)
                std::this_thread::sleep_for(POLL_INTERVAL);
        }
    }
}
