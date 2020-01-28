
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

position_names = [
    "planar_x",
    "planar_z",
    "planar_roty",
    "left_hip_pin",
    "right_hip_pin",
    "left_knee_pin",
    "right_knee_pin"]

velocity_names = [
    "planar_xdot",
    "planar_zdot",
    "planar_rotydot",
    "left_hip_pindot",
    "right_hip_pindot",
    "left_knee_pindot",
    "right_knee_pindot"]

effort_names = [
    "left_hip_torque",
    "right_hip_torque",
    "left_knee_torque",
    "right_knee_torque"]

pos_names = msg.position_names
vel_names = msg.velocity_names
eff_names = msg.effort_names

addPlot()
addSignals('RABBIT_STATE_SIMULATION', msg.utime, msg.position, position_names, keyLookup=pos_names)

#you can assign the plot to a variable and reference it later
p=addPlot()
addSignals('RABBIT_STATE_SIMULATION', msg.utime, msg.velocity, velocity_names, keyLookup=vel_names, plot=p)


p3 = addPlot()
addSignals('RABBIT_INPUT', msg.utime, msg.efforts, effort_names,
           keyLookup=eff_names, plot=p3)
