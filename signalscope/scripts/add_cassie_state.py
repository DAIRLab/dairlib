
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
    "hip_roll_left",
    "hip_roll_right",
    "hip_yaw_left",
    "hip_yaw_right",
    "hip_pitch_left",
    "hip_pitch_right",
    "knee_left",
    "knee_right",
    "toe_left",
    "toe_right"]

velocity_names = [
    "hip_roll_leftdot",
    "hip_roll_rightdot",
    "hip_yaw_leftdot",
    "hip_yaw_rightdot",
    "hip_pitch_leftdot",
    "hip_pitch_rightdot",
    "knee_leftdot",
    "knee_rightdot",
    "toe_leftdot",
    "toe_rightdot"]

effort_names = [
    "hip_roll_left_motor",
    "hip_roll_right_motor",
    "hip_yaw_left_motor",
    "hip_yaw_right_motor",
    "hip_pitch_left_motor",
    "hip_pitch_right_motor",
    "knee_left_motor",
    "knee_right_motor",
    "toe_left_motor",
    "toe_right_motor"]

pos_names = msg.position_names
vel_names = msg.velocity_names
eff_names = msg.effort_names
addPlot()
addSignals('CASSIE_STATE', msg.utime, msg.position, position_names, keyLookup=pos_names)


# you can assign the plot to a variable and reference it later
p=addPlot()
addSignals('CASSIE_STATE', msg.utime, msg.velocity, velocity_names, keyLookup=vel_names, plot=p)


p3 = addPlot()
addSignals('CASSIE_INPUT', msg.utime, msg.efforts, effort_names, keyLookup=eff_names, plot=p3)



