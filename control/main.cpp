#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>

#include <QApplication>
#include <QCoreApplication>
#include <QFile>
#include <QPalette>
#include <QTextStream>
#include <gst/gst.h>

#include "ui/main_window.h"

namespace {

// The app is built without a console, so warnings go next to the executable.
void logToFile(QtMsgType, const QMessageLogContext&, const QString& message) {
    static QFile file(QCoreApplication::applicationDirPath() + QStringLiteral("/control.log"));
    static const bool open = file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);
    if (!open) return;
    QTextStream(&file) << message << '\n';
    file.flush();
}

}  // namespace

int main(int argc, char* argv[]) {
    WSADATA wsa{};
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return 1;

    gst_init(&argc, &argv);
    QApplication app(argc, argv);
    qInstallMessageHandler(logToFile);

    QPalette dark;
    dark.setColor(QPalette::Window, QColor(18, 18, 20));
    dark.setColor(QPalette::WindowText, QColor(228, 228, 232));
    dark.setColor(QPalette::Base, QColor(30, 30, 34));
    dark.setColor(QPalette::Text, QColor(228, 228, 232));
    dark.setColor(QPalette::Button, QColor(40, 40, 46));
    dark.setColor(QPalette::ButtonText, QColor(228, 228, 232));
    app.setPalette(dark);

    int rc = 0;
    {
        MainWindow window;
        window.showMaximized();
        rc = app.exec();
    }  // window destroyed here: threads joined and sockets closed before cleanup

    gst_deinit();
    WSACleanup();
    return rc;
}
