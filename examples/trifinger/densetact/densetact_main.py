from densetact_optical_flow import OpticalFlow
from densetact_visualizer import graphical, visual
from densetact_lcm import lcmt_densetact_measurement_data

import yaml
import os
import numpy as np
import lcm
import cv2 as cv

from PyQt5.QtWidgets import QApplication, QMainWindow, QSlider, QLabel, QVBoxLayout, QWidget

import tkinter as tk
from tkinter import Label
import cv2
from PIL import Image, ImageTk

import plotly.graph_objects as go


# # Import YAML config
# with open(os.getcwd() + '/examples/trifinger/parameters/trifinger_densetact_configuration.yml', 'r') as file:
#     config = yaml.safe_load(file)

# vis_bool = config['data']['visual_bool']

# # Create Densetact obejcts
# densetact_obj1 = OpticalFlow(config['sensor']['DenseTact1'], config['data'])
# densetact_obj2 = OpticalFlow(config['sensor']['DenseTact2'], config['data'])

# # Initialize Real-time Visual
# if vis_bool:
#     real_time_visual1 = graphical("DenseTact1", config['sensor']['DenseTact1'])
#     real_time_visual2 = graphical("DenseTact2", config['sensor']['DenseTact2'])




# lcm_data = lcmt_densetact_measurement_data()



# height_thresh_dt1 = config['sensor']['DenseTact1']['height_thresh']
# image_diff_thresh_dt1 =  config['sensor']['DenseTact1']['image_diff_thresh']

# height_thresh_dt2 = config['sensor']['DenseTact2']['height_thresh']
# image_diff_thresh_dt2 =  config['sensor']['DenseTact2']['image_diff_thresh']

#frames = []os.path.exists(filename)
#timestamp = []




# while True:
#     # Calculates flow vectors and outputs the next frame
#     frame_color1 = densetact_obj1.calc_flow()
#     frame_color2 = densetact_obj2.calc_flow()

#     # Calculates several parameters including force
#     flow_data1 = densetact_obj1.get_force(frame_color1, flow_norm_thres=height_thresh_dt1, image_diff_thres=image_diff_thresh_dt1)
#     flow_data2 = densetact_obj2.get_force(frame_color2, flow_norm_thres=height_thresh_dt2, image_diff_thres=image_diff_thresh_dt2)

#     data_row = np.array([flow_data1['time'], flow_data1['contact_bool'], flow_data1['contactPose'], flow_data1['Sn'], flow_data1['St'], flow_data2['contact_bool'], flow_data2['contactPose'], flow_data2['Sn'], flow_data2['St']])

    

    ### LCM
    # lcm_data.sensorData[0].utime = np.int64(flow_data1['time']*1E6)
    # lcm_data.sensorData[0].inContact = flow_data1['contact_bool']
    # lcm_data.sensorData[0].contactPose = flow_data1['contactPose']
    # lcm_data.sensorData[0].scaledNormal = flow_data1['Sn']
    # lcm_data.sensorData[0].scaledFriction = flow_data1['St']

    # lcm_data.sensorData[1].utime = np.int64(flow_data2['time']*1E6)
    # lcm_data.sensorData[1].inContact = flow_data2['contact_bool']
    # lcm_data.sensorData[1].contactPose = flow_data2['contactPose']
    # lcm_data.sensorData[1].scaledNormal = flow_data2['Sn']
    # lcm_data.sensorData[1].scaledFriction = flow_data2['St']

    # lc = lcm.LCM()

    # lc.publish("CONTACT_FORCE_CHANNEL", lcm_data.encode())

    # if vis_bool:
        
        #window.update_image(frame_color1, frame_color2, flow_data1, flow_data2)
    # Display real-time visual
    # if vis_bool:
#         f1 = real_time_visual1.images(frame_color1, flow_data1) 
#         f2 = real_time_visual2.images(frame_color2, flow_data2)


#         frames.append(go.Frame(data=[go.Surface(z=-flow_data1['A']+flow_data1['height'], x=flow_data1['x'], y=flow_data1['y'],)], name=str(flow_data1['time'])))
#         timestamp.append(flow_data1['time'])
        
#         k = cv.waitKey(1) & 0xff
#         if k == ord('q'):
#             fig = go.Figure(
#             data=[go.Surface(z=-flow_daos.path.exists(filename)ta1['A'], x=flow_data1['x'], y=flow_data1['y'], colorscale="viridis")],
#             frames=frames)

#             fig.update_layout(
#             title="Interactive 3D Surface with Time Slider",
#             updatemenus=[{
#                 "buttons": [
#                     {
#                         "args": [None, {"frame": {"duration": 100, "redraw": True}, "mode": "immediate"}],
#                         "label": "Play",
#                         "method": "animate"
#                     },
#                     {
#                         "args": [[None], {"frame": {"duration": 0, "redraw": True}, "mode": "immediate"}],
#                         "label": "Pause",
#                         "method": "animate"
#                     }
#                 ],
#                 "direction": "left",
#                 "pad": {"r": 10, "t": 87},
#                 "showactive": False,
#                 "type": "buttons",
#                 "x": 0.1,
#                 "xanchor": "right",
#                 "y": 0,
#                 "yanchor": "top"os.path.exists(filename)
#             }],
#             sliders=[{
#                 "steps": [
#                     {
#                         "args": [[str(t)], {"frame": {"duration": 0, "redraw": True}, "mode": "immediate"}],
#                         "label": f"{t:.1f}",
#                         "method": "animate"
#                     } for t in timestamp
#                 ],
#                 "currentvalue": {"prefix": "Time: ", "font": {"size": 18}},
#                 "pad": {"b": 10, "t": 50},
#                 "len": 0.9,
#                 "x": 0.1,
#                 "xanchor": "left",
#                 "y": 0,
#                 "yanchor": "top"
#             }]
# )
            
#             fig.update_layout(
#             scene = {
#             "aspectratio": {"x": 1, "y": 1, "z": 1}
#             })

#             fig.update_layout(scene=dict(
#             zaxis=dict(range=[-50, 50]),  # Set x-axis range from 0 to 4
#             ))
#             fig.show()
#             break

from datetime import datetime
import pdb
import csv

import yaml
import os
import numpy as np
import lcm
import cv2 as cv


class csv_generate():
    def __init__(self) -> None:

        filename = "raw_densetact_data_" + datetime.now().strftime('%m-%d_%H-%M-%S') + ".csv"

        dir_path = os.getcwd() + '/examples/trifinger/densetact/densetact_training_data/' + filename

        assert not os.path.exists(dir_path), filename + " already exists!"

        self.file = open(dir_path, mode='w', newline='')
        self.writer = csv.writer(self.file)

        header = ['time [s]', 'contact_bool', 'Snx', 'Sny', 'Stx', 'Sty', 'Staux', 'Stauy']
        self.writer.writerow(header)
    
    def add_data(self, data) -> None:
        # data array: [time, contact_bool, Snx, Sny, Stx, Sty, Staux, Stauy]
        assert len(data) == 8, "Data array must have 8 elements"

        if isinstance(data, np.ndarray):
            data = data.tolist()

        self.writer.writerow(data)
        self.file.flush()

    def close(self) -> None:
        self.file.close()


    def densetact_loop():
        pass

if __name__ == "__main__":

    # # Import YAML config
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

    height_thresh_dt1 = config['sensor']['DenseTact1']['height_thresh']
    image_diff_thresh_dt1 =  config['sensor']['DenseTact1']['image_diff_thresh']

    height_thresh_dt2 = config['sensor']['DenseTact2']['height_thresh']
    image_diff_thresh_dt2 =  config['sensor']['DenseTact2']['image_diff_thresh']


    while True:
        # Calculates flow vectors and outputs the next frame
        frame_color1 = densetact_obj1.calc_flow()
        frame_color2 = densetact_obj2.calc_flow()

        # Calculates several parameters including force
        flow_data1 = densetact_obj1.get_force(frame_color1, flow_norm_thres=height_thresh_dt1, image_diff_thres=image_diff_thresh_dt1)
        flow_data2 = densetact_obj2.get_force(frame_color2, flow_norm_thres=height_thresh_dt2, image_diff_thres=image_diff_thresh_dt2) 

    data_csv = csv_generate()

    data_csv.close()

