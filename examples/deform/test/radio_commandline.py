"""Command-line keyboard teleop for DEFORM_RADIO using terminal keypresses.

Key bindings:
  W/S: +Z / -Z (channel 2)
  left/right arrows: +Y / -Y (channel 1)
  up/down arrows: +X / -X (channel 0)
  T: Toggle teleoperation mode
  ESC: Quit

This script runs entirely in the terminal (no GUI). It uses raw terminal
mode and non-blocking reads, and publishes LCM messages at RATE Hz.
"""

import lcm
import numpy as np
import os.path as op
import sys
import select
import termios
import tty
import time

DAIRLIB_DIR = op.abspath(
    op.dirname(op.dirname(op.dirname(op.dirname(__file__))))
)
sys.path.append(op.join(DAIRLIB_DIR, "bazel-bin", "lcmtypes"))
import dairlib


RATE = 60  # target rate (Hz) for radio publishing
POLL_PERIOD = 1.0 / RATE


def read_key_nonblocking(timeout=0.0):
    """Read a single key (or escape sequence) non-blocking from stdin.

    Returns None if no key was pressed during the timeout window, otherwise
    returns a string like 'up', 'down', 'left', 'right', 'w', 's', 't', 'esc'.
    """
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return None
    ch1 = sys.stdin.read(1)
    if not ch1:
        return None
    # Handle escape sequences (arrow keys start with '\x1b')
    if ch1 == "\x1b":
        # Try to read the next two bytes (common for arrow keys).
        # Use very small timeouts so we don't block.
        seq = ch1
        if select.select([sys.stdin], [], [], 0.001)[0]:
            seq += sys.stdin.read(1)
        if select.select([sys.stdin], [], [], 0.001)[0]:
            seq += sys.stdin.read(1)
        if seq.startswith("\x1b[A"):
            return "up"
        if seq.startswith("\x1b[B"):
            return "down"
        if seq.startswith("\x1b[C"):
            return "right"
        if seq.startswith("\x1b[D"):
            return "left"
        # Unknown escape sequence
        return None
    key = ch1.lower()
    if key == "\x03":  # Ctrl-C
        raise KeyboardInterrupt
    if key == "\r" or key == "\n":
        return None
    if key == "\x1b":
        return "esc"
    if key in ("w", "s", "t"):
        return key
    return None


def print_instructions():
    lines = [
        "Keyboard Teleoperation (terminal):",
        "  up/down arrows: ±X (channel 0)",
        "  left/right arrows: ±Y (channel 1)",
        "  W/S: ±Z (channel 2)",
        "  T: toggle teleop (channel 14)",
        "  ESC: quit",
        "",
    ]
    print("\n".join(lines))


def main():
    publisher = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")

    axis_inputs = np.zeros(3, dtype=float)
    latching = {"a": True}

    print_instructions()

    # Save terminal settings and enter raw mode.
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        last_print = 0.0
        while True:
            start = time.monotonic()

            # Read all keys available within this period. We'll treat each key
            # event as a momentary command for this publish step. Holding a key
            # typically generates repeated key events from the OS.
            key = read_key_nonblocking(timeout=0.0)
            # reset axis inputs to zero each step; key events in this frame
            # will set them for this publish packet (repeat behavior from OS
            # produces continuous control when holding keys).
            axis_inputs[:] = 0.0
            while key is not None:
                if key == "esc":
                    print("Exiting.")
                    return
                if key == "t":
                    latching["a"] = not latching["a"]
                    print(f"Teleop toggled: {latching['a']}")
                elif key == "up":
                    axis_inputs[0] = 1.0
                elif key == "down":
                    axis_inputs[0] = -1.0
                elif key == "right":
                    axis_inputs[1] = 1.0
                elif key == "left":
                    axis_inputs[1] = -1.0
                elif key == "w":
                    axis_inputs[2] = 1.0
                elif key == "s":
                    axis_inputs[2] = -1.0

                # Consume any additional immediate key events for this loop
                key = read_key_nonblocking(timeout=0.0)

            # Build and publish the LCM message.
            msg = dairlib.lcmt_radio_out()
            # Channels are floats in the Python bindings; convert as needed.
            msg.channel[0] = float(axis_inputs[0])
            msg.channel[1] = float(axis_inputs[1])
            msg.channel[2] = float(axis_inputs[2])
            # Use channel 14 for teleop toggle to match original GUI.
            msg.channel[14] = int(bool(latching["a"]))
            publisher.publish("DEFORM_RADIO", msg.encode())

            # Periodically print status (every 0.25s)
            now = time.monotonic()
            if now - last_print > 0.25:
                print(
                    f"Axes: X={axis_inputs[0]:+.1f} Y={axis_inputs[1]:+.1f} "
                    f"Z={axis_inputs[2]:+.1f}  Teleop={latching['a']}"
                )
                last_print = now

            # Sleep until the next period, accounting for time spent.
            elapsed = time.monotonic() - start
            to_sleep = POLL_PERIOD - elapsed
            if to_sleep > 0:
                time.sleep(to_sleep)

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
    main()
