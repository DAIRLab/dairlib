#import densetact_.projection_model as pm
import numpy as np

import optical_flow as of

import yaml

from helper import graphical, real_time_visual

"""
Testing Framework
"""
from io import BytesIO
import struct

import dairlib

class lcmt_densetact_measurement_data(object):
    """
      Measured DenseTact Array
    
    """

    __slots__ = ["numSensors", "sensorData"]

    __typenames__ = ["int8_t", "dairlib.lcmt_densetact_measurement"]

    __dimensions__ = [None, ["numSensors"]]

    def __init__(self):
        self.numSensors = 2
        """ LCM Type: int8_t """
        self.sensorData = []
        """ LCM Type: lcmt_densetact_measurement[numSensors] """

    def encode(self):
        buf = BytesIO()
        buf.write(lcmt_densetact_measurement_data._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">b", self.numSensors))
        for i0 in range(self.numSensors):
            assert self.sensorData[i0]._get_packed_fingerprint() == dairlib.lcmt_densetact_measurement._get_packed_fingerprint()
            self.sensorData[i0]._encode_one(buf)

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != lcmt_densetact_measurement_data._get_packed_fingerprint():
            raise ValueError("Decode error")
        return lcmt_densetact_measurement_data._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = lcmt_densetact_measurement_data()
        self.numSensors = struct.unpack(">b", buf.read(1))[0]
        self.sensorData = []
        for i0 in range(self.numSensors):
            self.sensorData.append(dairlib.lcmt_densetact_measurement._decode_one(buf))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if lcmt_densetact_measurement_data in parents: return 0
        newparents = parents + [lcmt_densetact_measurement_data]
        tmphash = (0x5eb95ea8bc9c69bf+ dairlib.lcmt_densetact_measurement._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if lcmt_densetact_measurement_data._packed_fingerprint is None:
            lcmt_densetact_measurement_data._packed_fingerprint = struct.pack(">Q", lcmt_densetact_measurement_data._get_hash_recursive([]))
        return lcmt_densetact_measurement_data._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", lcmt_densetact_measurement_data._get_packed_fingerprint())[0]



if __name__ == "__main__":
    import time
    import helper as helper
    import cv2 as cv
    import lcm

    #import meshcat
    #import meshcat.geometry as g
    import numpy as np
    import scipy as sp
    #import trimesh as tri


    import matplotlib.pyplot as plt
    import matplotlib

    import os

    
    #from dairlib import lcmt_densetact_measurement_data

    with open(os.getcwd() + '/examples/trifinger/parameters/trifinger_densetact_configuration.yml', 'r') as file:
        config = yaml.safe_load(file)
    vis_bool = config['data']['mesh_cat']
    stream_bool = config['data']['stream']
    # if vis_bool:
    #     import meshcat_vis as mv

    densetact_obj1 = of.OpticalFlow(config['sensor']['DenseTact1'], config['data'])
    densetact_obj2 = of.OpticalFlow(config['sensor']['DenseTact2'], config['data'])

    #vis = mv.meshcat_vis() if vis_bool else 0


    real_time_visual1 = helper.graphical("DenseTact1", config['sensor']['DenseTact1'])
    real_time_visual2 = helper.graphical("DenseTact2", config['sensor']['DenseTact2'])


    # Main loop that runs the densetact processing
    frame = 0
    while True:
        points, flow_vec, frame_color = densetact_obj1.calc_flow()
        points2, flow_vec2, frame_color2 = densetact_obj2.calc_flow()

        flow_data1 = densetact_obj1.get_force(points, flow_vec, frame_color)
        flow_data2 = densetact_obj2.get_force(points2, flow_vec2, frame_color2)

        #vis.vector_ray(flow_data1) if vis_bool else 0
        real_time_visual1.images(frame_color, flow_data1) if stream_bool else 0
        real_time_visual2.images(frame_color2, flow_data2) if stream_bool else 0

        msg = lcmt_densetact_measurement_data()
        data = lcmt_densetact_measurement_data().sensorData
        print(data)


        msg.numSensors = 2
        # msg.sensorData.utime = flow_data1['time']
        # msg.sensorData.inContact = flow_data1['contact_bool']
        # msg.sensorData.contactPose = flow_data1['T']
        # msg.sensorData.scaledNormal = flow_data1['Sn']
        # msg.sensorData.scaledFriction = flow_data1['St']

        sensorData_dt1 = [flow_data1['time'], flow_data1['contact_bool'], flow_data1['T'], flow_data1['Sn'], flow_data1['St']]
        sensorData_dt2 = [flow_data2['time'], flow_data2['contact_bool'], flow_data2['T'], flow_data2['Sn'], flow_data2['St']]
        msg.sensorData = [sensorData_dt1, sensorData_dt2]

        # msg.sensorData[1].utime = flow_data2['time']
        # msg.sensorData[1].inContact = flow_data2['contact_bool']
        # msg.sensorData[1].contactPose = flow_data2['T']
        # msg.sensorData[1].scaledNormal = flow_data2['Sn']
        # msg.sensorData[1].scaledFriction = flow_data2['St']

        lc = lcm.LCM()

        lc.publish("DENSETACT", msg.encode())




        # surface_points = (0.5*np.vstack([r*np.sin(theta_ang)*np.cos(phi_ang),
        #           r*np.sin(theta_ang)*np.sin(phi_ang),
        #           a*r*np.cos(theta_ang) + 0.01 * z.flatten()])).T
        #
        # colors = frame_color[..., ::-1].reshape(1024 * 768, 3) / 256
        # surface_points += np.array([0, 0, 0.21])
        # lam = 0.75/np.sqrt(a**2 + projrays[:,0]**2 + projrays[:,1]**2)
        # #print(lam)
        # normed_rays = lam[:, np.newaxis] * projrays + np.ones_like(projrays)*np.array([0, 0, 0.29])
        #
        # surface_points = vis.get_intersection(projrays)
        #print(surface_points)
        # point_cloud = g.PointCloud(position = surface_points.T, color = colors.T, size=0.01)
        # vis.pc(point_cloud)

        # if frame == 100:
        #     fig, axs = plt.subplots(1, 2, figsize=(10, 6))
        #     z = flow_data['div_grid']
        #
        #     contour = axs[0].contourf(flow_data['x'], flow_data['y'], flow_data['heightmap'])
        #     contour2 = axs[1].contourf(flow_data['x'], flow_data['y'], flow_data['div_grid'])
        #     plt.colorbar(contour, label='Height')
        #     plt.colorbar(contour2, label='Height')
        #
        #     plt.draw()
        #     plt.pause(100000)


        k = cv.waitKey(1) & 0xff
        if k == ord('q'):
            break

        frame += 1






