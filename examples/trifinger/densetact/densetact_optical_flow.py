import time
import subprocess
import os
import numpy as np
import scipy as sp
import cv2 as cv

from densetact_poisson_solver import poisson
from densetact_visualizer import graphical

#import csv
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

        time.sleep(1)
        # Disable Manual Exposure (Only works in terminal)
        subprocess.check_call(f"v4l2-ctl -d /dev/video{i} -c auto_exposure=1", shell=True)
        time.sleep(1)
        # Set the Exposure Time [ms.] from the .yaml (Only works in terminal)
        subprocess.check_call(f"v4l2-ctl -d /dev/video{i} -c exposure_time_absolute={densetact['expo_time']}", shell=True)

        time.sleep(1)

        # Get first frame
        ret, old_frame = self.cap.read()
        # Get the time first frame was taken
        self.start = time.perf_counter()

        assert ret, "Camera could not open"
        print("Opened up camera: ", densetact['serial_number'])

        # Setup bounding box for densetact
        self.bx = (densetact['center'][0] - densetact['rad_OI'], densetact['center'][0] + densetact['rad_OI'])
        self.by = (densetact['center'][1] - densetact['rad_OI'], densetact['center'][1] + densetact['rad_OI'])

        # Run image through pre-processing
        self.old_gray, self.old_frame = self.pre_processing(old_frame)

        # Set up dense inverse search optical flow
        self.flow = cv.DISOpticalFlow_create()

        # grid for for region of interest of step 1
        self.x = np.arange(0, 2 * densetact['rad_OI'])
        self.y = np.arange(0, 2 * densetact['rad_OI'])

        # Sub-sampling (TODO: use interpolation instead of sub-sampling)
        self.x_ss = self.x[::data['grid_step']]
        self.y_ss = self.y[::data['grid_step']]
        self.n = data['grid_step']

        self.XX_ss, self.YY_ss = np.meshgrid(self.x_ss, self.y_ss)

        self.cx = densetact['rad_OI']
        self.cy = densetact['rad_OI']

        # Set up a binary mask for what is inside of the reflective surface on the image
        self.mask = np.sqrt((self.XX_ss - densetact['rad_OI'])**2 + (self.YY_ss - densetact['rad_OI'])**2) < 260

        # Set up the laplacian matrix
        self.decomp = poisson(self.XX_ss, self.YY_ss)

        self.f = (0.76 * 10**-3) / (6*10**-6)

        self.flow_vec = np.zeros((2 * densetact['rad_OI'], 2 * densetact['rad_OI'], 2))


    def pre_processing(self, image):
        """
        Helper function designed for any image pre-processing 
        - blurring, sharpening, or changing contrast)
        and conversion to gray-scale

        :param image, (746, 1024, 3) BGR image
        :return: proc_img, (746, 1024) gray-scale image
        """

        bx = self.bx
        by = self.by

        blurred = cv.GaussianBlur(image, (5, 5), 0)

        proc_img = blurred
        proc_img_gray = cv.cvtColor(blurred, cv.COLOR_BGR2GRAY)

        
        bx = self.bx
        by = self.by

        return proc_img_gray[by[0]: by[1], bx[0]: bx[1]], proc_img[by[0]: by[1], bx[0]: bx[1]]

    def get_force(self, frame_color):
        """
        Each feature has its respective position (i.e. pos) and flow vector (i.e. flow_vec).
        This extracts location of contact (max divergence) and force magnitudes:
        - divergence ~ normal force, norm(curl) ~ torsion, harmonic ~ friction
        :param pos: nx2 array, n is the number of features
        :param flow_vec: nx2 array
        :return: XX, YY, interp_grid, field_comps, max_div
        """
        x_ss = self.x_ss
        y_ss = self.y_ss
        XX_ss = self.XX_ss
        YY_ss = self.YY_ss
        interp_grid = self.flow_vec[::self.n, ::self.n]

        u_hat = (1/self.f) * (XX_ss - self.cx)
        v_hat = (1/self.f) * (YY_ss - self.cy)

        # Remove flow outside of ROI
        interp_grid[~self.mask] = np.nan

        # Calculate divergence over the entire field
        div_grid = (np.gradient(interp_grid[..., 0], x_ss, axis=1, edge_order=2)
                    + np.gradient(interp_grid[..., 1], y_ss, axis=0, edge_order=2))
        
        div_grid[~self.mask] = np.nan
        
        # Get the CoP on the field
        armgax_2d = np.unravel_index(np.nanargmax(div_grid), interp_grid[..., 0].shape)
        div_max = div_grid[armgax_2d]
        CoP = np.array([int(x_ss[armgax_2d[1]]), int(y_ss[armgax_2d[0]])])

        # figure out the region of contact
        div_thres = 0.03
        contact_mask = div_grid > div_thres

        # HELMHOLTZ HODGE DECOMP
        nan_field = np.full(interp_grid.shape, np.nan)
        curl_free, div_free, harm = self.decomp.solve(interp_grid) \
            if np.any(contact_mask) \
            else [nan_field, nan_field, nan_field]

        contact_points = np.stack([XX_ss[contact_mask], YY_ss[contact_mask]], axis=1)

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

        M = np.array([
            [np.sin(theta) * np.cos(phi), np.sin(theta) * np.sin(phi), np.cos(theta)],
            [np.cos(theta) * np.cos(phi), np.cos(theta) * np.sin(phi), -np.sin(theta)],
            [-np.sin(phi), np.cos(phi), 0]
        ])
        
        

        Sn = np.nansum(np.linalg.norm(curl_free, axis = 2))/5000
        
        # recalibration
        contact_thres = 6_000_000
        con = np.sum(cv.absdiff(frame_color, self.old_frame).astype(np.float32))
        self.recalibration(frame_color) if (con < contact_thres) and (div_max > div_thres) else 0

        St = np.nanmean(div_free[contact_mask], axis = 0) if np.any(contact_mask) else [0, 0]

        self.old_gray = self.old_gray#[b_x[0]: b_x[1], b_y[0]: b_y[1]]
        self.old_frame = self.old_frame#[b_x[0]: b_x[1], b_y[0]: b_y[1]]

        return {
            'time': self.end - self.start,
            'x': XX_ss, 'y' : YY_ss,
            'theta': theta_ang, 'phi': phi_ang,
            'feat_pos': np.stack([XX_ss, YY_ss], axis = -1),
            'grid': interp_grid, 'div': div_max,
            'CoP': CoP, 'CoP_ray_norm': CoP_ray_norm,
            'curl_free': curl_free, 'div_free': div_free, 'harm': harm,
            'contact_bool': np.any(contact_mask), 'contact_points': contact_points,
            'Sn': Sn, 'St': St,
            'M': M, 'contactPose': np.block([[M, CoP_on_sph.reshape(3, 1)], [np.zeros((1, 3)), np.array([[1]])]])
        }

    def recalibration(self, frame_color):
        self.flow_vec = np.zeros_like(self.flow_vec)

    def calc_flow(self):
        """
        Does the following:
        -Captures images from the densetact and extract features; features are defined as corners pick up by the Shi Tomasi algorithm
        -Calculates flow from subsequent frames using pyrimadal Lucas Kanade
        :return: good_new, flow_vec, frame_color
        """
        ### CAPTURE IMAGE
        ret, frame = self.cap.read()
        self.end = time.perf_counter() if not ret else 0

        frame_gray, frame_color = self.pre_processing(frame)

        instan_flow_vec = self.flow.calc(np.ascontiguousarray(self.old_gray), np.ascontiguousarray(frame_gray), None)
  
        self.old_gray = frame_gray
        self.flow_vec = self.flow_vec + instan_flow_vec

        return frame_color

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

    def recalibration(self, frame_color):
        frame_gray = cv.cvtColor(frame_color, cv.COLOR_BGR2GRAY)

        p0 = cv.goodFeaturesToTrack(frame_gray, mask=None, **self.feature_params)
        p0 = np.squeeze(p0, axis=1)
