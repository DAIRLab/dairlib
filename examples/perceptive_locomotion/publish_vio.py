import dairlib
import pyrealsense2 as rs
import lcm
import numpy as np


from pydrake.all import RigidTransform, RotationMatrix, Quaternion


# Decent guess for the camera location in the pelvis frame - need to calibrate
t265_translation = np.array([0.20658760101219291, 0.0, 0.14530425217005591])

class TrackingCamera():
    def __init__(
            self,
            camera_pos_in_pelvis,
            position_offset=np.zeros((3,)),
            lcm_url=None
    ):
        # constants
        self.camera_pos_in_pelvis = camera_pos_in_pelvis
        self.realsense_origin_in_world = position_offset
        self.pub_channel = "CASSIE_GPS_POSITION"
        self.sub_channel = "NETWORK_CASSIE_STATE_DISPATCHER"
        self.start_time = -1
        self.reset_position_threshold = 0.1
        self.vio_position = np.zeros((3,))
        self.dispatcher_position = np.zeros((3,))

        # setup realense pipeline
        self.rs_pipeline = rs.pipeline()
        self.rs_config = rs.config()
        self.rs_config.enable_stream(rs.stream.pose)
        profile = self.rs_config.resolve(self.rs_pipeline)
        dev = profile.get_device().first_pose_sensor()
        dev.set_option(rs.option.enable_pose_jumping, 0)

        # setup lcm
        if lcm_url:
            self.lcm = lcm.LCM(lcm_url)
        else:
            self.lcm = lcm.LCM()

    def set_dispatcher_position(self, robot_output):
        rotation = Quaternion(np.array(robot_output.position[:4])).rotation()
        translation = np.array(robot_output.position[4:7])
        self.dispatcher_position = translation + rotation @ self.camera_pos_in_pelvis

    def maybe_reset_vio_position_from_dispatcher(self):
        error = self.dispatcher_position - self.vio_position
        if np.linalg.norm(error) > self.reset_position_threshold:
            self.realsense_origin_in_world += error

    def p_w(self, p_c):
        return self.realsense_origin_in_world + np.array([-p_c.z, -p_c.x, p_c.y])

    def get_camera_pose_raw(self):
        frames = self.rs_pipeline.wait_for_frames()
        pose = frames.get_pose_frame().get_pose_data()
        return frames.timestamp, pose

    def publish_frame(self, timestamp, pose):
        msg = dairlib.lcmt_gps_signal()
        msg.parent_body_name = "pelvis"
        msg.receiver_pos_in_parent_body = self.camera_pos_in_pelvis
        msg.receiver_pos_in_world = self.p_w(pose.translation)
        msg.mtime = int(timestamp - self.start_time)

        if pose.tracker_confidence > 0:
            msg.cov = 5.0**(1 - pose.tracker_confidence)
        else:
            msg.cov = -1

        self.lcm.publish(self.pub_channel, msg.encode())

    def step(self):
        t, pose = self.get_camera_pose_raw()
        self.vio_position = self.p_w(pose.translation)
        self.publish_frame(t, pose)
        self.lcm.handle()
        self.maybe_reset_vio_position_from_dispatcher()

    def start(self, state_callback):
        self.lcm.subscribe(self.sub_channel, state_callback)
        self.lcm.handle()
        self.rs_pipeline.start(self.rs_config)
        self.start_time, start_pose = self.get_camera_pose_raw()
        self.vio_position = np.array([
            -start_pose.translation.z,
            -start_pose.translation.x,
             start_pose.translation.y
        ])
        self.maybe_reset_vio_position_from_dispatcher()

    def stop(self):
        self.rs_pipeline.stop()

def vio_main():
    cam = TrackingCamera(t265_translation)
    def handle_state_callback(channel, data):
        cam.set_dispatcher_position(dairlib.lcmt_robot_output.decode(data))
    cam.start(handle_state_callback)
    while True:
        cam.step()



def test():
    cam = TrackingCamera(np.zeros(3,))
    cam.start()
    while(True):
        cam.publish_frame()


if __name__ == '__main__': vio_main()
