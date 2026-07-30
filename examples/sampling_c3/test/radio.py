"""Simple keyboard teleop GUI with Matplotlib for using keypresses instead of
xbox controller as inputs.

Key bindings:
  W/S: +Z / -Z (channel 2)
  left/right arrows: +Y / -Y (channel 1)
  up/down arrows: +X / -X (channel 0)
  T: Toggle teleoperation mode
  ESC: Quit
"""

import argparse
import lcm
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import os.path as op
import sys
import yaml

DAIRLIB_DIR = op.abspath(
    op.dirname(op.dirname(op.dirname(op.dirname(__file__))))
)
sys.path.append(op.join(DAIRLIB_DIR, "bazel-bin", "lcmtypes"))
import dairlib


RATE = 60  # target rate (Hz) for radio publishing
MS_INTERVAL = 1000.0 / RATE  # in milliseconds

CONTROLLER_PARAMS_PATH = op.join(
    DAIRLIB_DIR, "examples/sampling_c3/three_d_printer/parameters/sampling_c3_controller_params.yaml"
)


def str2bool(v: str) -> bool:
    if v.lower() in ("true", "false"):
        return v.lower() == "true"
    raise argparse.ArgumentTypeError(f"Expected true/false, got: {v}")


def get_radio_channel(is_simulation: bool) -> str:
    with open(CONTROLLER_PARAMS_PATH, "r") as f:
        controller_params = yaml.safe_load(f)

    channels_key = (
        "lcm_channels_simulation_file"
        if is_simulation
        else "lcm_channels_hardware_file"
    )
    channels_path = op.join(DAIRLIB_DIR, controller_params[channels_key])
    with open(channels_path, "r") as f:
        channels = yaml.safe_load(f)
    return channels["radio_channel"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--is_simulation",
        type=str2bool,
        default=True,
        help="Whether to use the simulation or hardware radio channel name "
        "(true/false).",
    )
    args = parser.parse_args()
    radio_channel = get_radio_channel(args.is_simulation)

    # publisher = lcm.LCM()
    publisher = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")

    # Disable all default matplotlib key bindings.
    for key in mpl.rcParams:
        if key.startswith("keymap."):
            mpl.rcParams[key] = []

    # Variables to update each loop.
    axis_inputs = np.zeros(3)
    latching = {  # Can add more toggle states to this dictionary.
        "a": True,  # Teleop mode
    }
    pressed = set()

    # Create figure.
    fig, ax = plt.subplots(figsize=(6, 3))
    fig.canvas.manager.set_window_title("Keyboard Teleoperation")
    text = ax.text(
        0.05, 0.95, "", fontsize=12, va="top", ha="left", family="monospace"
    )
    ax.axis("off")

    # Key press callbacks.
    def on_key_press(event):
        key = event.key.lower()

        if key == "escape":
            plt.close(fig)
            exit()

        # continuous axes
        if key in ["up", "down", "left", "right", "w", "s"]:
            pressed.add(key)

        # toggle keys
        elif key == "t":
            latching["a"] = not latching["a"]
            print("Teleop:", latching["a"])

    def on_key_release(event):
        key = event.key.lower()
        pressed.discard(key)

    fig.canvas.mpl_connect("key_press_event", on_key_press)
    fig.canvas.mpl_connect("key_release_event", on_key_release)

    # Update function on every loop:  update figure and publish over LCM.
    def update(_=None):
        # Compute continuous axes from held keys.
        axis_inputs[0] = float("up" in pressed) - float("down" in pressed)
        axis_inputs[1] = float("right" in pressed) - float("left" in pressed)
        axis_inputs[2] = float("w" in pressed) - float("s" in pressed)

        # Build and publish the LCM message.
        msg = dairlib.lcmt_radio_out()
        msg.channel[0] = axis_inputs[0]
        msg.channel[1] = axis_inputs[1]
        msg.channel[2] = axis_inputs[2]
        msg.channel[14] = latching["a"]
        publisher.publish(radio_channel, msg.encode())

        # Update the text in the figure.
        lines = [
            f"Teleop (T): {latching['a']}",
            f"Axes: X={axis_inputs[0]:.1f}, Y={axis_inputs[1]:.1f}, "
            + f"Z={axis_inputs[2]:.1f}",
            "",
            "Controls:",
            "  up/down arrows: ±X",
            "  left/right arrows: ±Y",
            "  W/S: ±Z",
            "  T toggle features",
            "  ESC to quit",
        ]
        text.set_text("\n".join(lines))
        fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=MS_INTERVAL)
    timer.add_callback(update)
    timer.start()
    plt.show()


if __name__ == "__main__":
    main()
