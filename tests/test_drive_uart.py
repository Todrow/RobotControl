#!/usr/bin/env python3
"""Linux integration tests: TCP DesiredState -> actual termios UART -> PTY.

Run from any directory with Python 3 and a C++17 compiler:
    python3 tests/test_drive_uart.py

No GStreamer, Raspberry Pi, ESP32, external network or third-party Python
packages are needed. All compiled artifacts and server logs stay in temporary
directories. A PTY checks the software serial path, not physical GPIO wiring.
"""

import os
from pathlib import Path
import select
import shlex
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest

if sys.platform.startswith("linux"):
    import pty
    import termios


ROOT = Path(__file__).resolve().parents[1]
STOP = b"R:0%|L:0%\n"


def command(direction, speed, pitch=0.0, yaw=0.0):
    # Three ABI padding bytes precede DriveCommand.speed in protocol.h.
    return struct.pack("<B3xfff", direction, speed, pitch, yaw)


def unused_port():
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        return reservation.getsockname()[1]


@unittest.skipUnless(sys.platform.startswith("linux"), "requires Linux termios and PTYs")
class DriveUartTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build = tempfile.TemporaryDirectory(prefix="robot-drive-uart-")
        cls.addClassCleanup(cls.build.cleanup)
        cls.executable = Path(cls.build.name) / "drive_uart_harness"
        compiler = shlex.split(os.environ.get("CXX", "c++"))
        subprocess.run(
            compiler + [
                "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wpedantic", "-pthread",
                "-I" + str(ROOT / "common"), "-I" + str(ROOT / "mock_robot"),
                str(ROOT / "tests" / "drive_uart_harness.cpp"),
                str(ROOT / "mock_robot" / "command_listener.cpp"),
                str(ROOT / "mock_robot" / "servo_controller.cpp"),
                str(ROOT / "mock_robot" / "drive_controller.cpp"),
                "-o", str(cls.executable),
            ],
            check=True,
        )

    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.addCleanup(os.close, self.slave)
        self.addCleanup(os.close, self.master)
        self.serial_buffer = b""
        self.port = unused_port()
        self.log = tempfile.TemporaryFile(mode="w+b")
        self.addCleanup(self.log.close)
        self.process = subprocess.Popen(
            [str(self.executable), str(self.port), os.ttyname(self.slave)],
            stdout=self.log, stderr=subprocess.STDOUT,
        )
        self.addCleanup(self.stop_server)
        self.expect_line(STOP)  # Initialization must put both wheels at rest.

    def logs(self):
        self.log.seek(0)
        return self.log.read().decode("utf-8", errors="replace")

    def stop_server(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
                self.fail("listener did not stop after SIGTERM\n" + self.logs())
        self.assertEqual(self.process.returncode, 0, self.logs())

    def connect(self):
        deadline = time.monotonic() + 3
        while True:
            try:
                client = socket.create_connection(("127.0.0.1", self.port), timeout=1)
                self.addCleanup(client.close)
                return client
            except ConnectionRefusedError:
                if self.process.poll() is not None or time.monotonic() >= deadline:
                    self.fail("listener did not accept connections\n" + self.logs())
                time.sleep(0.01)

    def read_line(self, timeout=2):
        deadline = time.monotonic() + timeout
        while b"\n" not in self.serial_buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.master], [], [], remaining)[0]:
                return None
            self.serial_buffer += os.read(self.master, 4096)
        line, self.serial_buffer = self.serial_buffer.split(b"\n", 1)
        return line + b"\n"

    def expect_line(self, expected, timeout=2):
        self.assertEqual(self.read_line(timeout), expected, self.logs())

    def assert_closed(self, client):
        client.settimeout(2)
        try:
            self.assertEqual(client.recv(1), b"", self.logs())
        except ConnectionResetError:
            pass  # Closing a connection with unread input may send TCP RST.

    def test_uart_is_raw_115200_8n1(self):
        settings = termios.tcgetattr(self.slave)
        self.assertEqual(settings[4:6], [termios.B115200, termios.B115200])
        self.assertEqual(settings[2] & termios.CSIZE, termios.CS8)
        self.assertFalse(settings[2] & (termios.PARENB | termios.CSTOPB | termios.CRTSCTS))
        self.assertTrue(settings[2] & termios.CLOCAL)
        self.assertTrue(settings[2] & termios.CREAD)
        self.assertFalse(settings[0] & (termios.IXON | termios.IXOFF | termios.ICRNL))
        self.assertFalse(settings[1] & termios.OPOST)
        self.assertFalse(settings[3] & (termios.ICANON | termios.ECHO | termios.ISIG))

    def test_directions_speed_rounding_and_repeated_commands(self):
        client = self.connect()
        cases = [
            (1, 1.0, b"R:100%|L:100%\n"),
            (1, 0.5, b"R:50%|L:50%\n"),
            (2, 0.5, b"R:-50%|L:-50%\n"),
            (3, 0.75, b"R:75%|L:-75%\n"),
            (4, 0.25, b"R:-25%|L:25%\n"),
            (0, 1.0, STOP),
            (1, 0.0, STOP),
            (2, 0.0, STOP),
            (3, 0.0, STOP),
            (4, 0.0, STOP),
            (1, 0.005, b"R:1%|L:1%\n"),
            (1, 0.374, b"R:37%|L:37%\n"),
            (1, 0.375, b"R:38%|L:38%\n"),
            (2, 0.375, b"R:-38%|L:-38%\n"),
            (1, 0.995, b"R:100%|L:100%\n"),
            (1, 0.996, b"R:100%|L:100%\n"),
            (1, 0.996, b"R:100%|L:100%\n"),
        ]
        for direction, speed, expected in cases:
            with self.subTest(direction=direction, speed=speed):
                client.sendall(command(direction, speed))
                self.expect_line(expected)

    def test_fragmented_and_coalesced_tcp_frames(self):
        client = self.connect()
        frame = command(3, 0.6)
        client.sendall(frame[:3])
        self.assertIsNone(self.read_line(timeout=0.04), "partial frame moved wheels")
        client.sendall(frame[3:11])
        self.assertIsNone(self.read_line(timeout=0.04), "partial frame moved wheels")
        client.sendall(frame[11:])
        self.expect_line(b"R:60%|L:-60%\n")
        client.sendall(command(1, 0.2) + command(4, 0.9))
        self.expect_line(b"R:20%|L:20%\n")
        self.expect_line(b"R:-90%|L:90%\n")

    def test_camera_changes_preserve_drive_and_explicit_stop(self):
        client = self.connect()
        client.sendall(command(1, 0.7, -1.0, 1.0))
        self.expect_line(b"R:70%|L:70%\n")
        client.sendall(command(1, 0.7, 0.5, -0.5))
        self.expect_line(b"R:70%|L:70%\n")
        client.sendall(command(0, 0.7, 1.0, -1.0))
        self.expect_line(STOP)

    def test_disconnect_stops_and_reconnect_resumes(self):
        for _ in range(2):
            client = self.connect()
            client.sendall(command(1, 0.5))
            self.expect_line(b"R:50%|L:50%\n")
            client.close()
            self.expect_line(STOP)

    def test_silent_client_timeout_stops(self):
        client = self.connect()
        client.sendall(command(1, 0.8))
        self.expect_line(b"R:80%|L:80%\n")
        # The listener has a 500 ms deadline; leave broad scheduling headroom.
        self.expect_line(STOP, timeout=2)
        self.assert_closed(client)

    def test_partial_frame_does_not_extend_timeout(self):
        client = self.connect()
        client.sendall(command(2, 0.8))
        self.expect_line(b"R:-80%|L:-80%\n")
        partial = command(1, 0.3)
        # Keep input arriving for longer than one deadline, but never complete
        # the 16-byte state. Activity must not keep old wheel power running.
        for byte in partial[:7]:
            try:
                client.sendall(bytes([byte]))
            except (BrokenPipeError, ConnectionResetError):
                break
            time.sleep(0.1)
        self.expect_line(STOP, timeout=0.15)
        self.assert_closed(client)

    def test_invalid_state_stops_and_rejects_connection(self):
        invalid = [
            command(255, 0.5), command(1, -0.01), command(1, 1.01),
            command(1, float("nan")), command(1, float("inf")),
            command(1, float("-inf")), command(1, 0.5, 1.01, 0),
            command(1, 0.5, 0, -1.01), command(1, 0.5, float("nan"), 0),
            command(1, 0.5, 0, float("inf")),
        ]
        for frame in invalid:
            with self.subTest(frame=frame.hex()):
                client = self.connect()
                client.sendall(command(1, 0.4))
                self.expect_line(b"R:40%|L:40%\n")
                client.sendall(frame)
                self.expect_line(STOP)
                self.assert_closed(client)
                client.close()

    def test_graceful_shutdown_stops_active_wheels(self):
        client = self.connect()
        client.sendall(command(4, 1.0))
        self.expect_line(b"R:-100%|L:100%\n")
        self.process.send_signal(signal.SIGTERM)
        self.expect_line(STOP)
        self.process.wait(timeout=3)
        self.assertEqual(self.process.returncode, 0, self.logs())

    def test_missing_uart_fails_startup(self):
        missing = Path(self.build.name) / "missing-uart"
        result = subprocess.run(
            [str(self.executable), str(unused_port()), str(missing)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=3,
        )
        self.assertNotEqual(result.returncode, 0, result.stdout.decode())
        self.assertFalse(missing.exists(), "UART startup must not create a regular file")


if __name__ == "__main__":
    unittest.main(verbosity=2)
