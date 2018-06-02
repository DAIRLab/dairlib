
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

joints = ['L_HIP_ROLL', 'L_HIP_YAW','L_HIP_PITCH', 'L_KNEE', 'L_FOOT','R_HIP_ROLL', 'R_HIP_YAW','R_HIP_PITCH', 'R_KNEE', 'R_FOOT']
names = msg.joint_names


addPlot()
addSignals('CASSIE_STATE', msg.timestamp, msg.position, joints, keyLookup=names)


# you can assign the plot to a variable and reference it later
p=addPlot()
addSignals('CASSIE_STATE', msg.timestamp, msg.velocity, joints, keyLookup=names, plot=p)


p3 = addPlot()
#addSignals('CASSIE_INPUT', msg.timestamp, msg.inputs, joints, keyLookup=input_names, plot=p3)
addSignals('CASSIE_INPUT', msg.timestamp, msg.inputs, joints, keyLookup=names, plot=p)



