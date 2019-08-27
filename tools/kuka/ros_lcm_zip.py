import lcm
import rosbag
import subprocess, yaml
import numpy as np
import pdb

from PythonLCM import lcmt_iiwa_status
# from scipy.spatial.transform import Rotation as R

from scipy import interpolate

def quatmult(quaternion1, quaternion0):
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    return np.array([x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
                     -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0], dtype=np.float64)


inbag = '2019-08-22-16-18-51.odom.bag'
cube_topic = '/tagslam/odom/body_cube'
CUBE_CONFIG = 7
CUBE_VELOCITY = 6

info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', inbag], stdout=subprocess.PIPE).communicate()[0])

topics = [topic for topic in info_dict['topics'] if topic['topic'] == cube_topic]
num_msg = topics[0]['messages']
bag = rosbag.Bag(inbag)
t_ros = np.zeros(num_msg)
cube_ros = np.zeros((13,num_msg))
i = 0
for topic, msg, t in bag.read_messages(topics=['/tagslam/odom/body_cube']):
    tstamp = msg.header.stamp
    t_ros[i] = tstamp.secs + tstamp.nsecs*1e-9
    cube_pose = msg.pose.pose
    cube_pos = np.asarray([cube_pose.position.x, cube_pose.position.y, cube_pose.position.z])
    cube_quat = np.asarray([cube_pose.orientation.x, cube_pose.orientation.y, cube_pose.orientation.z, cube_pose.orientation.w])
    cube_ros[:4,i] = cube_quat
    cube_ros[4:7,i] = cube_pos
    i = i + 1
bag.close()
#pdb.set_trace()
cube_ros[10:,:-1] = np.divide(cube_ros[4:7,1:]-cube_ros[4:7,:-1],t_ros[1:]-t_ros[:-1])
cube_ros[10:,-1] = cube_ros[10:,-2]

qdot = np.divide(cube_ros[:4,1:]-cube_ros[:4,:-1],t_ros[1:]-t_ros[:-1])

for i in range(num_msg-1):
    #pdb.set_trace()
    qbar = cube_ros[:4,i]
    qbar[:3] = -qbar[:3]
    cube_ros[7:10,i] = 2*quatmult(qdot[:,i],qbar)[:3]
    print(cube_ros[7:10,i])
cube_ros[7:10,-1] = cube_ros[7:10,-2]
#pdb.set_trace()
# print(cube_ros[:7,:8])
log = lcm.EventLog("test66.log", "r")
#print(lcm.EventLog.size(log))
num_lcm = 0
for event in log:
    if event.channel == "IIWA_STATUS":
        num_lcm = num_lcm + 1
t_lcm = np.zeros(num_lcm)
kuka_lcm = np.zeros((14,num_lcm))
i = 0
for event in log:
    if event.channel == "IIWA_STATUS":
        t_lcm[i] = event.timestamp*1e-6
        msg = lcmt_iiwa_status.decode(event.data)
        kuka_lcm[:7,i] = np.asarray(msg.joint_position_measured)
        kuka_lcm[7:,i] = np.asarray(msg.joint_velocity_estimated)
        i = i + 1
print(kuka_lcm[:,:3])
#pdb.set_trace()

# trim ROS times to LCM times
com_times = np.logical_and(t_ros > np.min(t_lcm), t_ros < np.max(t_lcm))
t_ros = t_ros[com_times]
#pdb.set_trace()
cube_ros = cube_ros[:,com_times]

f = interpolate.interp1d(t_lcm, kuka_lcm)
kuka_ros = f(t_ros)
data = np.concatenate((np.expand_dims(t_ros, axis=0),kuka_ros,cube_ros),axis=0)
data = data.T
np.savetxt('test66.csv', data, delimiter=',')
