import numpy as np
import cv2 as cv
import scipy as sp
import time
from numpy.random import normal

import helper as h
#import projection_model as pm
import subprocess

import os


class OpticalFlow:
    def __init__(self, densetact, data):

        if data['real_time']:
            for i in range(15):
                try:
                    video_path = f"/dev/video{i}"
                    cmd = f"udevadm info --query=all --name={video_path} | grep 'ID_SERIAL_SHORT='"
                    output = subprocess.check_output(cmd, shell=True).decode()
                    serial_number = output.strip().split('=')[-1]

                    found = True if serial_number == densetact['serial_number'] else False
                except:
                    continue
                if found:
                    self.cap = cv.VideoCapture(i)
                    break
        else:
            self.cap = cv.VideoCapture(data['path'])
            print("Loading video path: " + data['path'])

        # Set the resolution to 1024x768
        self.resolution = (1024, 768)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.resolution[1])

        # Set camera settings
        self.cap.set(cv.CAP_PROP_SETTINGS, 1)

        # Set auto exposure off
        self.cap.set(cv.CAP_PROP_AUTO_EXPOSURE, 1)  # 0.25 is manual exposure mode in OpenCV
        time.sleep(1)
        # Sensor 1: Exposure 80 ms
        # Sensor 2: Exposure 11 ms
        self.cap.set(cv.CAP_PROP_EXPOSURE, densetact['expo_time'])
        time.sleep(2)
        ret, old_frame = self.cap.read()  # Get first frame
        self.start = time.perf_counter()
        print("Opened up camera: ", densetact['serial_number']) if ret else 0

        self.old_gray, self.old_frame = h.pre_processing(old_frame)

        #Parameters for Feature Detection
        self.feature_params = dict(maxCorners=2000,
                              qualityLevel=0.05,
                              minDistance=5,
                              blockSize=5)

        # Parameters for Lucas_Kanade
        self.lk_params = dict(winSize=(30, 30),
                         maxLevel=5,
                         criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 20, 0.03))

        # ShiTomasi corner detection
        self.p0 = cv.goodFeaturesToTrack(self.old_gray, mask=None, **self.feature_params)
        self.p0 = np.squeeze(self.p0, axis=1)

        self.init_p0 = self.p0.copy()

        # Setting up the ROI, centered in the middle of the image
        self.rad = densetact['rad_OI']#325
        grid_step = data['grid_step']
        self.x = np.linspace((self.resolution[0] / 2) - self.rad,
                             (self.resolution[0] / 2) + self.rad,
                             grid_step)
        self.y = np.linspace((self.resolution[1] / 2) - self.rad,
                             (self.resolution[1] / 2) + self.rad,
                             grid_step)

        self.XX, self.YY = np.meshgrid(self.x, self.y)

        #Set up the boundaries
        rad = densetact['rad_OI']
        bc_pos_x = np.linspace(((self.resolution[0] / 2) - rad),
                               ((self.resolution[0] / 2) + rad),
                               grid_step)

        sqrt_term = np.sqrt(rad ** 2 - (bc_pos_x - self.resolution[0] / 2) ** 2)
        bc_pos_y_1 = sqrt_term + (self.resolution[1] / 2)
        bc_pos_y_2 = -sqrt_term + (self.resolution[1] / 2)

        bc_pos = np.array([np.concatenate([bc_pos_x, bc_pos_x]),
                           np.concatenate([bc_pos_y_1, bc_pos_y_2])]).T

        flow_vec_bc = np.zeros_like(bc_pos)

        self.mask = ~np.isnan(sp.interpolate.griddata(bc_pos, flow_vec_bc, (self.XX, self.YY), method='linear')[...,0])

        self.bc_pos = bc_pos
        self.flow_vec_bc = flow_vec_bc

        self.f = (0.76 * 10**-3) / (6*10**-6)
        self.cx = self.resolution[0]/2
        self.cy = self.resolution[1]/2

        self.decomp = h.poisson(self.XX, self.YY, self.mask)

    def pyrLK(self, img0, img1, p0, back_threshold=0.5, checkTrace=True):
        """
        :param img0: first image
        :param img1: subsequent image
        :param p0: features
        :param back_threshold:
        :param checkTrace: Set True if you want to check if the LK estimation is correct, the LK algorithm will be
        applied in both directions
        :return:
        """

        if checkTrace == True:
            p1, _st, _err = cv.calcOpticalFlowPyrLK(img0, img1, p0, None, **self.lk_params)
            p0r, _st, _err = cv.calcOpticalFlowPyrLK(img1, img0, p1, None, **self.lk_params)
            d = abs(p0 - p0r).reshape(-1, 2).max(-1)
            status = d < back_threshold

            return p1, status[:, np.newaxis]

        else:
            p1, _st, _err = cv.calcOpticalFlowPyrLK(img0, img1, p0, None, **self.lk_params)

            return p1, _st

    def add_boundaries(self, pos, flow_vec, steps=50):
        """
        :param pos: nx2 array, the position of the features on image space
        :param flow_vec: nx2 array, the flow vector for each feature
        :param steps: int, the number steps along the axis for the boundaries
        :return: pos, flow_vec
        Adds boundaries to the pos and flow_vec, on the basis that flow is zero there    """
        flow_vec_bc = self.flow_vec_bc
        bc_pos = self.bc_pos

        pos = np.vstack([pos, bc_pos])
        flow_vec = np.vstack([flow_vec, flow_vec_bc])

        return pos, flow_vec

    def get_force(self, pos, flow_vec, frame_color):
        """
        Each feature has its respective position (i.e. pos) and flow vector (i.e. flow_vec).
        This extracts location of contact (max divergence) and force magnitudes:
        - divergence ~ normal force, norm(curl) ~ torsion, harmonic ~ friction
        :param pos: nx2 array, n is the number of features
        :param flow_vec: nx2 array
        :return: XX, YY, interp_grid, field_comps, max_div
        """
        x = self.x
        y = self.y
        XX = self.XX
        YY = self.YY

        ### GRID INTERPOLATION
        interp_grid = sp.interpolate.griddata(pos, flow_vec, (XX, YY), method='cubic', fill_value=np.nan)

        u_hat = (1/self.f) * (XX - self.cx)
        v_hat = (1/self.f) * (YY - self.cy)


        # Calculate divergence over the entire field
        div_grid = (np.gradient(interp_grid[..., 0], x, axis=1, edge_order=2)
                    + np.gradient(interp_grid[..., 1], y, axis=0, edge_order=2))

        # Get the CoP on the field
        armgax_2d = np.unravel_index(np.nanargmax(div_grid), interp_grid[..., 0].shape)
        div_max = div_grid[armgax_2d]
        CoP = np.array([int(x[armgax_2d[1]]), int(y[armgax_2d[0]])])

        # figure out the region of contact
        div_thres = 0.12
        contact_mask = div_grid > div_thres

        # HELMHOLTZ HODGE DECOMP
        nan_field = np.full(interp_grid.shape, np.nan)
        curl_free, div_free, harm = self.decomp.solve(interp_grid) \
            if np.any(contact_mask) \
            else [nan_field, nan_field, nan_field]

        contact_points = np.stack([XX[contact_mask], YY[contact_mask]], axis=1)


        

        proj_ray = np.hstack([(1/self.f) * (CoP[0] - self.cx), (1/self.f) * (CoP[1] - self.cy), 1])

        optical_center = np.array([0, 0, 23.5E-3])
        focal_point = optical_center - np.array([0, 0, 0.78E-3])

        sphere_mem_center = np.array([0, 0, 23.85E-3])

        CoP_ray = proj_ray

        CoP_ray_norm =  CoP_ray/np.linalg.norm(CoP_ray)
        #sensor diameter: 32mm
        CoP_on_sph = (0.032/2) * CoP_ray_norm

        theta_ang = np.arctan(np.sqrt(u_hat ** 2 + v_hat ** 2))
        phi_ang = np.arctan2(v_hat, u_hat)


        # the [theta, phi] at the CoP
        theta = np.arctan(np.sqrt(CoP_ray[0] ** 2 + CoP_ray[1] ** 2))
        phi = np.arctan2(CoP_ray[1], CoP_ray[0])

        # [theta, phi, rho] = M @ [i, j, k]
        M = np.array([[np.cos(theta) * np.cos(phi), np.cos(theta) * np.sin(phi), -np.sin(theta)],
                      [-np.sin(phi), np.cos(phi), 0],
                      [np.sin(theta) * np.cos(phi), np.sin(theta) * np.sin(phi), np.cos(theta)]])

        Sn = np.nansum(np.linalg.norm(curl_free, axis = 2))/50_000

        #St = np.nansum((div_free + harm)[contact_mask], axis = 0)/500 if np.any(contact_mask) else np.array([0, 0])
        St = np.nansum((div_free + harm), axis=(0,1)) / 5000 if np.any(contact_mask) else np.array([0, 0])


        # recalibration
        contact_thres = 6_000_000
        con = np.sum(cv.absdiff(frame_color, self.old_frame).astype(np.float32))
        self.recalibration(frame_color) if (con < contact_thres) and (div_max > div_thres) else 0

        return {
            'time': self.end - self.start,
            'x': XX, 'y' : YY,
            'theta': theta_ang, 'phi': phi_ang,
            'feat_pos': pos, 'feat_vec': flow_vec,
            'grid': interp_grid, 'div': div_max,
            'CoP': CoP, 'CoP_ray_norm': CoP_ray_norm,
            'curl_free': curl_free, 'div_free': div_free, 'harm': harm,
            'contact_bool': np.any(contact_mask), 'contact_points': contact_points,
            'Sn': Sn, 'St': St,
            'M': M, 'T': np.block([[M, CoP_on_sph.reshape(3, 1)], [np.zeros((1, 3)), np.array([[1]])]])
        }

    def recalibration(self, frame_color):
        frame_gray = cv.cvtColor(frame_color, cv.COLOR_BGR2GRAY)

        p0 = cv.goodFeaturesToTrack(frame_gray, mask=None, **self.feature_params)
        p0 = np.squeeze(p0, axis=1)

        self.p0 = p0
        self.init_p0 = p0

    def calc_flow(self):
        """
        Does the following:
        -Captures images from the densetact and extract features; features are defined as corners pick up by the Shi Tomasi algorithm
        -Calculates flow from subsequent frames using pyrimadal Lucas Kanade
        :return: good_new, flow_vec, frame_color
        """
        ### CAPTURE IMAGE
        ret, frame = self.cap.read()
        self.end = time.perf_counter()
        if not ret:
            return 0

        frame_gray, frame_color = h.pre_processing(frame)

        # Run Pyramidal Lucas-Kanade
        p1, st = self.pyrLK(self.old_gray, frame_gray, self.p0, checkTrace = True)

        # Discard bad points
        good_new = p1[(st == 1).flatten()]
        self.init_p0 =  self.init_p0[(st == 1).flatten()]

        # Calculate flow vectors with respect to the initial frame
        flow_vec = good_new - self.init_p0

        # Now update the previous frame and previous points
        self.old_gray = frame_gray
        self.p0 = good_new

        return good_new, flow_vec, frame_color

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    #import meshcat_vis as mv
    import matplotlib
    import time
    matplotlib.use('TkAgg')

    sensor1 = h.graphical()


    # fig = plt.figure(figsize=(16, 9))
    # ax = fig.subplots(1,2)
    # ax[0].invert_yaxis()
    # ax[1].invert_yaxis()
    #plt.colorbar(location='left')

    rad = 350
    resolution = (1024, 768)

    # [ax[i, j].set_aspect('equal', 'box') for i in range(1) for j in range(0)]
    #
    # [ax[i, j].set_xlim((resolution[0] / 2) - rad, (resolution[0] / 2) + rad) for i in range(1) for j in range(0)]
    # [ax[i, j].set_ylim((resolution[1] / 2) - rad, (resolution[1] / 2) + rad) for i in range(1) for j in range(0)]

    # plt.xlabel('X-axis')
    # plt.ylabel('Y-axis')
    # plt.grid(True)

    flow = OpticalFlow("/home/bbaraki/PycharmProjects/OpticalFlow/BasicTouches/normal_right_corner.mkv", real_time = 1)
    #dt_model = mv.meshcat_vis()
    proj = pm.sensor()

    # Parameters for the video
    output_filename = "DenseTact_smudge.mp4"
    frame_width, frame_height = resolution  # Set to your frame dimensions
    fps = 30  # Frames per second

    # Define the codec and create VideoWriter object
    fourcc = cv.VideoWriter_fourcc(*'MP4V')
    out = cv.VideoWriter(output_filename, fourcc, fps, (frame_width*2, frame_height))


    for frame in range(1000):
        points, flow_vec, frame_color = flow.calc_flow()

        start = time.perf_counter()
        flow_data = flow.get_force(points, flow_vec, frame_color)

        end = time.perf_counter()
        elapsed = end - start

        contact_binary = np.any(flow_data['contact_mask'])


        if contact_binary and 0:
            CoP = flow_data['CoP']
            _ = dt_model.point_cloud(CoP)

            # norm = flow_data['normal_force']
            # dt_model.vector_ray(CoP, norm, 'normal')
            # dt_model.vector_ray(CoP, flow_data['friction_force'], 'friction')

            #get boundaries
            #bc_3D = dt_model.update(bc)
            # h_bc = np.squeeze(bc_3D)[:,-1]
            #
            # h_bc_i = bc_mask.copy()
            # h_bc_i[bc_mask] = h_bc
            # print("test")
            # harm = decomp.solve(interp_grid)
            # print(harm)

            #length = friction_force

            # # ps = proj.getProj(contact_points)
            # # ps = ps[:-1,:].T
            # # print(ps)
            # # print(ps)
            # hull = sp.spatial.ConvexHull(contact_points)
            # vert = contact_points[hull.vertices]
            #
            # vert_in_3d = proj.getProj(vert)
            # BC = vert_in_3d
            #
            # #vert = vert.reshape((-1, 1, 2)).astype(np.int32)
            # #cv.fillConvexPoly(frame_color,vert,0,(255,0,0),lineType=cv.LINE_8)
            #
            # hull = cv.convexHull(contact_points.astype(np.int32))
            # hull = hull.reshape((-1, 1, 2)).astype(np.int32)
            # cv.fillConvexPoly(frame_color, hull, (255, 0, 0))
            #cv.polylines(frame_color, [pts], isClosed=True, color=(0, 0, 0), thickness=2)


        advanced_real_time_disp = True
        if advanced_real_time_disp:
            if frame == 1000:

                fig, axs = plt.subplots(2, 2, figsize=(10, 6))
                [axs[i, j].set_xlim((resolution[0] / 2) - rad, (resolution[0] / 2) + rad) for i in range(2) for j in range(2)]
                [axs[i, j].set_ylim((resolution[1] / 2) - rad, (resolution[1] / 2) + rad) for i in range(2) for j in range(2)]

                s = 1
                o = 0

                # OG
                YY = flow_data['y']
                XX = flow_data['x']
                interp_grid = flow_data['grid']
                curl_free = flow_data['curl_free']
                div_free = flow_data['div_free']
                harm = flow_data['harm']

                mag = np.linalg.norm(interp_grid, axis = 2)
                speed = (mag ) * s + o
                lw = 0.75
                d = 4
                speed = 'black'

                axs[0, 0].imshow(frame_color[..., ::-1])
                #axs[0,0].scatter(CoP[0], CoP[1], c='r')
                axis0 = axs[0, 0].streamplot(XX, YY, interp_grid[..., 0], interp_grid[..., 1],
                                        color=speed, linewidth=lw, cmap='viridis', density = d)
                axs[0, 0].title.set_text('Orginal Interpolated Field')
                cbar = fig.colorbar(axis0.lines)

                # Curl Free
                mag = np.linalg.norm(curl_free, axis = 2)
                speed = (mag ) * s + o
                axs[1, 0].imshow(frame_color[..., ::-1])
                axis1 = axs[1, 0].streamplot(XX, YY, curl_free[..., 0], curl_free[..., 1],
                                        color=speed, linewidth=lw, cmap='viridis', density = d)
                axs[1, 0].title.set_text('Curl Free Field')
                cbar = fig.colorbar(axis1.lines)

                # Divergent Free
                mag = np.linalg.norm(div_free, axis = 2)
                speed = (mag ) * s + o
                axs[0, 1].imshow(frame_color[..., ::-1])
                axis2 = axs[0, 1].streamplot(XX, YY, div_free[..., 0], div_free[..., 1],
                                     color=speed, linewidth=lw, cmap='viridis', density = d)
                axs[0, 1].title.set_text('Divergent Free Field')
                cbar = fig.colorbar(axis2.lines)
                # Haromonic
                mag = np.linalg.norm(harm, axis = 2)
                speed = (mag ) * s + o
                axs[1, 1].imshow(frame_color[..., ::-1])
                axis3 = axs[1, 1].streamplot(XX, YY, harm[..., 0], harm[..., 1],
                                    color=speed, linewidth=lw, cmap='viridis', density = d)
                axs[1, 1].title.set_text('Harmonic Field')
                cbar = fig.colorbar(axis3.lines)


                plt.draw()
                plt.pause(100000)

        # for point in flow_data['feat_pos']:
        #     cv.circle(frame_color, (int(point[0]), int(point[1])), 2, (0, 0, 255), -1)

        # if contact_binary:
        #     for con_p in flow_data['contact_points']:
        #         #print((int(con_p[0]), int(con_p[1])))
        #         cv.circle(frame_color, (int(con_p[0]), int(con_p[1])), 2, (255, 0, 0), -1)

        sensor1.images(frame_color, flow_data)


        #out.write(tot_image)
        # n = 4
        # x_qui = flow_data['x'][0, :][::n]#[:, ::n]
        # y_qui = flow_data['y'][:, 0][::n]#[::n, :]
        # grid_qui = flow_data['grid'][::n, ::n]
        # grid_qui[np.isnan(grid_qui)] = 0
        # print(grid_qui.shape)
        # # grid_qui =  sp.interpolate.griddata(np.stack([flow_data['x'], flow_data['y']], axis=2), flow_data['grid'], (x_qui, y_qui), method='cubic', fill_value=0)
        #
        #
        # s = 5
        # #print(grid_qui)
        # for i, y in enumerate(y_qui):
        #     for j, x in enumerate(x_qui):
        #         #print(grid_qui[i, j, 0])
        #         cv.arrowedLine(frame_color, (int(x), int(y)), (int(x + s*grid_qui[i, j, 0]), int(y + s*grid_qui[i, j, 1])), (0, 255, 255), 1)




        k = cv.waitKey(1) & 0xff
        if k == ord('q'):
            out.release()
            print("We got you on video ( ㄏ-᷅_-᷄)ㄏ")



    data = False
    if data:
        #data = np.stack((XX, YY, interp_grid[..., 0], interp_grid[..., 1]), axis=-1)
        XX.tofile('XX.csv', sep=',', format='%10.5f')
        YY.tofile('YY.csv', sep=',', format='%10.5f')
        interp_grid[..., 0].tofile('Nx.csv', sep=',', format='%10.5f')
        interp_grid[..., 1].tofile('Ny.csv', sep=',', format='%10.5f')

        ## PLOT 0 - Flow Norm
        axis0 = ax[0, 0]
        shw0 = axis0.contourf(XX, YY, np.linalg.norm(harm_comp, axis=2), levels=20)
        axis0.invert_yaxis()
        plt.colorbar(shw0, location='left')
        axis0.set_title("Flow Norm ||<u,v>|| in Image Space")

        axis0.grid(visible=True, which='both')

        axis1 = ax[0, 1]
        axis1.imshow(frame_color[..., ::-1])
        axis1.scatter(center[0], center[1], c='r')
        axis1.invert_yaxis()

        # PLOT 2 - Flow Angle
        axis3 = ax[1, 0]
        shw3 = axis3.contourf(XX, YY,
                              np.linalg.norm(interp_grid, axis=2),
                              levels=20,)
        axis3.invert_yaxis()
        plt.colorbar(shw3, location='left')
        axis3.set_title("Flow Angle of Divergent Component")
        axis3.grid(visible=True, which='both')

        # PLOT 2 - Flow Angle
        axis4 = ax[1, 1]
        shw3 = axis4.contourf(XX, YY,
                              np.rad2deg(np.arctan2(harm_comp[..., 1], harm_comp[..., 0])),
                              levels=20,
                              cmap='twilight')
        axis4.invert_yaxis()
        plt.colorbar(shw3, location='left')
        axis4.set_title("Norm of Harmonic Component")
        axis4.grid(visible=True, which='both')

        plt.draw()
        plt.pause(1000)

    # plt.show()
