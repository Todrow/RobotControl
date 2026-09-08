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
REVERSE = b"R:-20%|L:-20%\n"


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
                str(ROOT / "mock_robot" / "net" / "command_listener.cpp"),
                str(ROOT / "mock_robot" / "actuators" / "servo_controller.cpp"),
                str(ROOT / "mock_robot" / "actuators" / "drive_controller.cpp"),
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

    def disconnect_while_moving(self):
        client = self.connect()
        client.sendall(command(1, 0.5))
        self.expect_line(b"R:50%|L:50%\n")
        client.close()
        self.expect_line(STOP)
        return time.monotonic()

    def expect_reverse_after_pause(self, stopped_at):
        self.expect_line(REVERSE, timeout=2)
        reversing_at = time.monotonic()
        self.assertGreaterEqual(reversing_at - stopped_at, 0.90, self.logs())
        self.assertLess(reversing_at - stopped_at, 2.0, self.logs())
        return reversing_at

    def expect_recovery_sequence(self, stopped_at):
        reversing_at = self.expect_reverse_after_pause(stopped_at)
        # Repeated reverse frames keep the ESP32 watchdog supplied. The final
        # neutral command must arrive after a short, bounded reverse pulse.
        reverse_count = 1
        deadline = reversing_at + 0.8
        while True:
            remaining = deadline - time.monotonic()
            self.assertGreater(remaining, 0, "reverse did not stop\n" + self.logs())
            line = self.read_line(timeout=remaining)
            if line == STOP:
                break
            self.assertEqual(line, REVERSE, self.logs())
            reverse_count += 1
        self.assertGreaterEqual(time.monotonic() - reversing_at, 0.20, self.logs())
        self.assertGreaterEqual(reverse_count, 2, self.logs())
        self.assertIsNone(self.read_line(timeout=1.6), "recovery repeated while disconnected")

    def keep_command_active(self, client, frame, expected, duration=1.5):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            client.sendall(frame)
            self.expect_line(expected, timeout=0.5)
            time.sleep(0.05)

    def expect_stop_during_reverse(self):
        # A heartbeat already queued before accept/shutdown may precede STOP.
        # Require interruption before the normal 300 ms pulse would finish.
        deadline = time.monotonic() + 0.2
        while True:
            remaining = deadline - time.monotonic()
            self.assertGreater(remaining, 0, "reverse was not interrupted\n" + self.logs())
            line = self.read_line(timeout=remaining)
            if line == STOP:
                return
            self.assertEqual(line, REVERSE, self.logs())

    def assert_only_stops_after_shutdown(self):
        self.process.wait(timeout=3)
        self.assertEqual(self.process.returncode, 0, self.logs())
        while True:
            line = self.read_line(timeout=0.1)
            if line is None:
                return
            self.assertEqual(line, STOP, self.logs())

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

    def test_disconnect_stops_reverses_once_and_stays_stopped(self):
        self.expect_recovery_sequence(self.disconnect_while_moving())
        # A fresh established session may later start its own single recovery.
        self.expect_recovery_sequence(self.disconnect_while_moving())

    def test_silent_client_timeout_stops_and_recovers_once(self):
        client = self.connect()
        client.sendall(command(1, 0.8))
        self.expect_line(b"R:80%|L:80%\n")
        # The listener has a 500 ms deadline; leave broad scheduling headroom.
        self.expect_line(STOP, timeout=2)
        stopped_at = time.monotonic()
        self.assert_closed(client)
        self.expect_recovery_sequence(stopped_at)

    def test_reconnect_during_pause_cancels_reverse(self):
        self.disconnect_while_moving()
        client = self.connect()
        # Keep receiving valid states beyond the old recovery deadline. A stale
        # automatic command must never overwrite the reconnected controller.
        self.keep_command_active(client, command(4, 0.4), b"R:-40%|L:40%\n")

    def test_reconnect_during_reverse_stops_before_new_motion(self):
        self.expect_reverse_after_pause(self.disconnect_while_moving())
        client = self.connect()
        self.expect_stop_during_reverse()
        self.keep_command_active(client, command(1, 0.4), b"R:40%|L:40%\n")

    def test_silent_reconnect_cancels_pending_reverse(self):
        self.disconnect_while_moving()
        client = self.connect()
        # Accept alone cancels recovery. Without a first valid command this
        # socket times out stopped and must not arm another reverse movement.
        self.expect_line(STOP, timeout=1)
        self.assert_closed(client)
        self.assertIsNone(self.read_line(timeout=1.6), "silent reconnect armed recovery")

    def test_startup_and_unproven_connection_never_reverse(self):
        self.assertIsNone(self.read_line(timeout=1.6), "startup triggered motion")
        client = self.connect()
        self.expect_line(STOP, timeout=1)
        self.assert_closed(client)
        self.assertIsNone(self.read_line(timeout=1.6), "unproven client armed recovery")

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
        self.assertIsNone(self.read_line(timeout=1.6), "invalid command armed recovery")

    def test_graceful_shutdown_stops_active_wheels(self):
        client = self.connect()
        client.sendall(command(4, 1.0))
        self.expect_line(b"R:-100%|L:100%\n")
        self.process.send_signal(signal.SIGTERM)
        self.expect_line(STOP)
        self.process.wait(timeout=3)
        self.assertEqual(self.process.returncode, 0, self.logs())

    def test_shutdown_during_pause_cancels_reverse(self):
        self.disconnect_while_moving()
        self.process.send_signal(signal.SIGTERM)
        self.assert_only_stops_after_shutdown()

    def test_shutdown_during_reverse_stops_and_does_not_restart(self):
        self.expect_reverse_after_pause(self.disconnect_while_moving())
        self.process.send_signal(signal.SIGTERM)
        self.expect_stop_during_reverse()
        self.assert_only_stops_after_shutdown()

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
