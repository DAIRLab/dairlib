import rospy
import numpy as np

from franka_msgs.msg import FrankaStateCustom
from geometry_msgs.msg import Point


class SafetyLayer:
  def __init__(self, robot_state_topic, safety_params):
    """
    robot_state_topic: name of ROS topic that contains robot state
    virtual_walls: dictionary that maps name of wall to wall parameters
    """

    # TODO:
    # 1. check if we can use FrankaStateCustom, if not, use own topic
    # 2. check how to send error, might need to publish to a topic as well
    # 3. add other safeties such as velocity, torques, external forces,
    #    bad camera measurements, etc

    self.robot_state_topic = robot_state_topic
    self.virtual_walls = safety_params["virtual_walls"]
    self.subsriber = rospy.Subscriber(robot_state_topic, FrankaStateCustom, self.safety_callback)

  def safety_callback(self, msg):
    EEx = msg.O_T_EE[12] # x
    EEy = msg.O_T_EE[13] # y
    EEz = msg.O_T_EE[14] # z

    for name, wall in self.virtual_walls.items():
      a, b, c, d = wall['a'], wall['b'], wall['c'], wall['d']
      if ((a * EEx + b * EEy + c * EEz) <= d):
          rospy.logerr('VIOLATED VIRTUAL WALL: ' + name)
          # TODO: may need to publish error to some topic here to stop robot

    # TODO: implement other safety checks
  
if __name__ = "__main__":
  rospy.init_node("safety_script")
  param_path = "parameters_safety.yaml"
  franka_state_topic = "/panda/franka_state_controller/franka_states"

  with open(param_path, 'r') as stream:
      safety_params = yaml.safe_load(stream)
  safety = SafetyLayer(franka_state_topic, safety_params)

  rospy.spin()