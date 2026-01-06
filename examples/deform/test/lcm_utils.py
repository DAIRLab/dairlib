"""LCM publishing and subscribing."""

import lcm
import os.path as op
import sys

DAIRLIB_DIR = op.abspath(
    op.dirname(op.dirname(op.dirname(op.dirname(__file__))))
)
sys.path.append(op.join(DAIRLIB_DIR, "bazel-bin", "lcmtypes"))
import dairlib


class RadioSubscriber:
    """Subscribes to DEFORM_RADIO (lcmt_radio_out) channel and stores the latest
    message."""

    def __init__(self, channel: str = "DEFORM_RADIO"):
        self._lc = lcm.LCM()
        self._channel = channel
        self._subscription = self._lc.subscribe(channel, self._callback)
        self._msg = dairlib.lcmt_radio_out()  # default message

    def _callback(self, channel, data):
        self._msg = dairlib.lcmt_radio_out.decode(data)

    def get_latest_msg(self):
        """Returns the latest message received, or None if no message has been
        received yet."""
        self._lc.handle_timeout(0)  # Non-blocking check for new messages.
        return self._msg
