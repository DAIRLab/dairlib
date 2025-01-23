from densetact_optical_flow import OpticalFlow
from densetact_visualizer import graphical
from densetact_lcm import lcmt_densetact_measurement_data

import yaml
import os
import numpy as np
import lcm
import cv2 as cv

# Import YAML config
with open(os.getcwd() + '/examples/trifinger/parameters/trifinger_densetact_configuration.yml', 'r') as file:
    config = yaml.safe_load(file)

vis_bool = config['data']['visual_bool']

# Create Densetact obejcts
densetact_obj1 = OpticalFlow(config['sensor']['DenseTact1'], config['data'])
densetact_obj2 = OpticalFlow(config['sensor']['DenseTact2'], config['data'])

# Initialize Real-time Visual
if vis_bool:
    real_time_visual1 = graphical("DenseTact1", config['sensor']['DenseTact1'])
    real_time_visual2 = graphical("DenseTact2", config['sensor']['DenseTact2'])


lcm_data = lcmt_densetact_measurement_data()

while True:
    # Calculates flow vectors and outputs the next frame
    frame_color1 = densetact_obj1.calc_flow()
    frame_color2 = densetact_obj2.calc_flow()

    # Calculates several parameters including force
    flow_data1 = densetact_obj1.get_force(frame_color1)
    flow_data2 = densetact_obj2.get_force(frame_color2)

    ### LCM
    lcm_data.sensorData[0].utime = np.int64(flow_data1['time']*1E6)
    lcm_data.sensorData[0].inContact = flow_data1['contact_bool']
    lcm_data.sensorData[0].contactPose = flow_data1['contactPose']
    lcm_data.sensorData[0].scaledNormal = flow_data1['Sn']
    lcm_data.sensorData[0].scaledFriction = flow_data1['St']

    lcm_data.sensorData[1].utime = np.int64(flow_data2['time']*1E6)
    lcm_data.sensorData[1].inContact = flow_data2['contact_bool']
    lcm_data.sensorData[1].contactPose = flow_data2['contactPose']
    lcm_data.sensorData[1].scaledNormal = flow_data2['Sn']
    lcm_data.sensorData[1].scaledFriction = flow_data2['St']

    lc = lcm.LCM()

    lc.publish("CONTACT_FORCE_CHANNEL", lcm_data.encode())

    # Display real-time visual
    if vis_bool:
        real_time_visual1.images(frame_color1, flow_data1) 
        real_time_visual2.images(frame_color2, flow_data2)

        # Required for opencv images
        k = cv.waitKey(1) & 0xff
        if k == ord('q'):
            break


