
'''
Here is a basic example to plot a signal from an lcm message.
In this example, the channel is POSE_BODY.
The X coordinate is the message timestamp in microseconds,
and the Y value is pos[0], or the first value of the pos array.
Note, msg is a pre-defined variable that you must use in order
for this to work.  When you define a signal, the msg variable
is used to record the attribute lookups that are required to
extract the signal data from the lcm message in the future.
'''

print(dir())
print(globals())
print(locals())

pos_measured_indices = [0, 1, 2, 3, 4, 5, 6]
position_measured = [
    "shoulder_yaw",
    "shoulder_pitch",
    "shoulder_roll",
    "elbow",
    "wrist_roll",
    "wrist_pitch",
    "wrist_yaw"]

vel_estimated_indices = [0, 1, 2, 3, 4, 5, 6]
velocity_estimated = [
    "shoulder_yawdot",
    "shoulder_pitchdot",
    "shoulder_rolldot",
    "elbowdot",
    "wrist_rolldot",
    "wrist_pitchdot",
    "wrist_yawdot"]

torque_measured_indices = [0, 1, 2, 3, 4, 5, 6]
torque_measured = [
    "shoulder_yaw_torque",
    "shoulder_pitch_torque",
    "shoulder_roll_torque",
    "elbow_torque",
    "wrist_roll_torque",
    "wrist_pitch_torque",
    "wrist_yaw_torque"]

pos_names = msg.position_names
vel_names = msg.velocity_names
eff_names = msg.effort_names
addPlot()
addSignals('IIWA_STATUS', msg.utime, msg.joint_position_measured, pos_measured_indices)


# you can assign the plot to a variable and reference it later
p=addPlot()
addSignals('IIWA_STATUS', msg.utime, msg.joint_velocity_estimated, vel_estimated_indices, plot=p)


p3 = addPlot()
addSignals('IIWA_STATUS', msg.utime, msg.joint_torque_measured, torque_measured_indices, plot=p3)
