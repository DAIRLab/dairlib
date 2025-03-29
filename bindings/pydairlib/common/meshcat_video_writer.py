import os
import tempfile
import subprocess
from time import sleep

from pydrake.systems.all import Diagram, Simulator
from pydairlib.lcm.log_playback import LcmLogPlayback
from pydairlib.common.meshcat_chrome_capture import MeshcatChromeCapture


def write_meshcat_video_from_log(diagram, lcm_log, meshcat, channel_to_type_map,
                                 channel_to_port_map, video_out_path, start=0, duration=-1,
                                 window_size=(1440, 1080)):
    capture = MeshcatChromeCapture(meshcat, window_size=window_size)

    # wait for plant to load
    sleep(5)
    playback = LcmLogPlayback(
        lcm_log, diagram, channel_to_type_map, channel_to_port_map, start_time=start)

    dt = 1.0/30.0
    t = dt
    frame = 0
    with tempfile.TemporaryDirectory() as temp_dir:
        while not playback.finished() and (duration < 0 or frame * dt <= duration):
            playback.advance(t)
            # save screenshot
            frame_filename = os.path.join(temp_dir, f"frame_{frame:06d}.png")
            capture.grab(frame_filename)
            frame += 1
            t += dt

        subprocess.run([
            'ffmpeg',
            '-y',
            '-framerate',
            '30',
            '-i',
            os.path.join(temp_dir, f"frame_%06d.png"),
            '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
            video_out_path
        ], check=True)
