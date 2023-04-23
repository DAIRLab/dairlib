import dairlib
import pyrealsense2 as rs
import lcm
import numpy as np


class TrackingCamera():
    def __init__(self, camera_pos_in_pelvis,
                 position_offset=np.zeros((3,)), lcm_url=None):
        self.p_C = camera_pos_in_pelvis
        self.p0 = position_offset
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.pose)
        self.channel = "CASSIE_GPS_POSITION"
        profile = self.cfg.resolve(self.pipe)
        dev = profile.get_device().first_pose_sensor()
        dev.set_option(rs.option.enable_pose_jumping, 0)

        if lcm_url:
            self.lcm = lcm.LCM(lcm_url)
        else:
            self.lcm = lcm.LCM()

    def p_w(self, p_c):
        return self.p0 + np.array([-p_c.z, -p_c.x, p_c.y])

    def publish_frame(self):
        msg = dairlib.lcmt_gps_signal()
        msg.parent_body_name = "pelvis"
        msg.receiver_pos_in_parent_body = self.p_C.tolist()

        frames = self.pipe.wait_for_frames()
        pose = frames.get_pose_frame().get_pose_data()
        msg.receiver_pos_in_world = self.p_w(pose.translation)
        confidence = pose.tracker_confidence
        if confidence > 0:
            msg.cov = 10.0**(1 - confidence)
        else:
            msg.cov = -1

        self.lcm.publish(self.channel, msg.encode())

    def start(self):
        self.pipe.start(self.cfg)

def test():
    cam = TrackingCamera(np.zeros(3,))
    cam.start()
    while(True):
        cam.publish_frame()


if __name__ == '__main__': test()