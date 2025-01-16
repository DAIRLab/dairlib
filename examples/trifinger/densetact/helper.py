import numpy as np
import scipy as sp
import cv2 as cv
from sklearn.feature_extraction.image import grid_to_graph, img_to_graph

class graphical:
    def __init__(self, name, densetact):
        self.dis_RIO = densetact['rad_OI']
        self.resolution = (1024, 768)
        self.name = name

    def legend(self, symbol):
        pass

    def __crop(self, frame):
        res = self.resolution
        d = self.dis_RIO

        # cropping
        b_x = (int(res[1] / 2 - d), int(res[1] / 2 + d))
        b_y = (int(res[0] / 2 - d), int(res[0] / 2 + d))

        return frame[b_x[0]: b_x[1], b_y[0]: b_y[1]]
        #cv.imshow("Optical Flow", frame_color[b_x[0]:b_x[1], b_y[0]:b_y[1]])

    def images(self, og_frame, flow_data):
        frame_color = self.__crop(og_frame.copy())
        arrow_frame = og_frame.copy()
        comp_frame = og_frame.copy()

        arrow_frame = self.graphical_flow(arrow_frame, flow_data)
        images = self.graphical_decomp(og_frame, flow_data)

        top = np.hstack([arrow_frame, images[0]])
        bottom = np.hstack([images[1], images[2]])
        tot_image = np.vstack([top, bottom])

        resized_image = cv.resize(tot_image, (int(tot_image.shape[1] // 1.5), int(tot_image.shape[0] // 1.5)))
        cv.imshow("Optimal Flow: " + self.name, resized_image)

    def graphical_flow(self, arrow_frame, flow_data, n = 4, scale = 2):
        x_qui = flow_data['x'][0, :][::n]
        y_qui = flow_data['y'][:, 0][::n]
        grid_qui = flow_data['grid'][::n, ::n]

        for i, y in enumerate(y_qui):
            for j, x in enumerate(x_qui):
                if  ~np.isnan(grid_qui[i, j, 0]):
                    cv.arrowedLine(arrow_frame, (int(x), int(y)),
                                   (int(x + scale * grid_qui[i, j, 0]),
                                    int(y + scale * grid_qui[i, j, 1])),
                                   (255, 255, 255), 2)

        for point in flow_data['feat_pos']:
            cv.circle(arrow_frame,
                      (int(point[0]), int(point[1])),
                      1,
                      (0,0,0),
                       -1)

        right_align = 200
        cv.arrowedLine(arrow_frame, (right_align, 75),
                       (right_align, 60),
                       (255, 255, 255), 3)
        cv.putText(arrow_frame, "Optical Flow", (right_align+20, 70), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv.LINE_AA)

        cv.circle(arrow_frame, (flow_data['CoP'][0], flow_data['CoP'][1]), 5, (100, 255, 100), -1) if flow_data['contact_bool'] else 0

        return self.__crop(arrow_frame)

    def graphical_decomp(self, og_frame, flow_data, n = 4, scale = 3, thick = 3):
        x_qui = flow_data['x'][0, :][::n]
        y_qui = flow_data['y'][:, 0][::n]
        cf_qui = flow_data['curl_free'][::n, ::n]
        df_qui = flow_data['div_free'][::n, ::n]
        h_qui = flow_data['harm'][::n, ::n]

        cf_image = og_frame.copy()
        df_image = og_frame.copy()
        harm_image = og_frame.copy()

        for i, y in enumerate(y_qui):
            for j, x in enumerate(x_qui):
                if ~np.isnan(cf_qui[i, j, 0]):
                    # decomp
                    # red
                    cv.arrowedLine(cf_image, (int(x), int(y)),
                                   (int(x + scale * cf_qui[i, j, 0]),
                                    int(y + scale * cf_qui[i, j, 1])),
                                   (0, 0, 256), thick)
                    # green
                    cv.arrowedLine(df_image, (int(x), int(y)),
                                   (int(x + scale * df_qui[i, j, 0]),
                                    int(y + scale * df_qui[i, j, 1])),
                                   (150, 255, 150), thick)
                    # blue
                    cv.arrowedLine(harm_image, (int(x), int(y)),
                                   (int(x + scale * h_qui[i, j, 0]),
                                    int(y + scale * h_qui[i, j, 1])),
                                   (255, 0, 0), thick)

        right_align = 200

        cv.putText(cf_image, "Curl-free",(right_align + 20, 60), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv.LINE_AA)
        cv.putText(df_image, "Div-free", (right_align + 20, 60), cv.FONT_HERSHEY_SIMPLEX, 1, (200, 255, 200), 1, cv.LINE_AA)
        cv.putText(harm_image, "Harmonic", (right_align + 20, 60), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 100, 100), 1, cv.LINE_AA)
        return (self.__crop(cf_image), self.__crop(df_image), self.__crop(harm_image))

def real_time_visual(frame_color, flow_data, n = 4, flow_bool = True, feat_bool = True):
    arrow_frame = frame_color.copy()
    comp_frame = frame_color.copy()

    # subsampling

    if flow_bool:
        x_qui = flow_data['x'][0, :][::n]
        y_qui = flow_data['y'][:, 0][::n]
        grid_qui = flow_data['grid_sph'][::n, ::n]
        cf_qui = flow_data['curl_free'][::n, ::n]
        df_qui = flow_data['div_free'][::n, ::n]
        h_qui = flow_data['harm'][::n, ::n]

        s = 2
        for i, y in enumerate(y_qui):
            for j, x in enumerate(x_qui):
                if ~np.isnan(cf_qui[i, j, 0]) and ~np.isnan(grid_qui[i, j, 0]):
                    cv.arrowedLine(arrow_frame, (int(x), int(y)),
                                   (int(x + s * grid_qui[i, j, 0]),
                                    int(y + s * grid_qui[i, j, 1])),
                                   (255, 255, 255), 2)

                    #decomp
                    cv.arrowedLine(comp_frame, (int(x), int(y)),
                                   (int(x + s * cf_qui[i, j, 0]),
                                    int(y + s * cf_qui[i, j, 1])),
                                   (0, 0, 255), 2)
                    cv.arrowedLine(comp_frame, (int(x), int(y)),
                                   (int(x + s * df_qui[i, j, 0]),
                                    int(y + s * df_qui[i, j, 1])),
                                   (0, 255, 0), 2)
                    cv.arrowedLine(comp_frame, (int(x), int(y)),
                                   (int(x + s * h_qui[i, j, 0]),
                                    int(y + s * h_qui[i, j, 1])),
                                   (255, 0, 0), 2)

    if feat_bool:
        for point in flow_data['feat_pos']:
            cv.circle(arrow_frame, (int(point[0]), int(point[1])), 2, (0, 0, 0), -1)

    if np.any(flow_data['contact_mask']):
        cv.circle(frame_color, (int(flow_data['CoP'][0]), int(flow_data['CoP'][1])), 5, (0, 255, 0), -1)

    #LEGEND
    #FIRST IMAGE
    cv.rectangle(arrow_frame, (50,40), (65,55), (255, 255, 255), -1)
    cv.circle(arrow_frame, (58, 48), 5, (0, 0, 0), -1)
    cv.putText(arrow_frame, "Corner Feature", (70, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv.LINE_AA)

    cv.arrowedLine(arrow_frame, (60, 75),
                   (60,65),
                   (255, 255, 255), 2)
    cv.putText(arrow_frame, "Optical Flow", (70, 70), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv.LINE_AA)

    #SECOND IMAGE
    cv.circle(frame_color, (50, 50), 5, (0, 255, 0), -1)
    cv.putText(frame_color, "Center of Pressure", (60, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv.LINE_AA)

    rad = 400
    # b_x = (int(resolution[0] / 2 - rad), int(resolution[0] / 2 + rad))
    # b_y = (int(resolution[1] / 2 - rad), int(resolution[1] / 2 + rad))

    top =  np.hstack([arrow_frame, frame_color])
    bottom = np.hstack([comp_frame, frame_color])
    tot_image = np.vstack([top, bottom])

    resized_image = cv.resize(tot_image, (int(tot_image.shape[1] // 1.5), int(tot_image.shape[0] // 1.5)))
    cv.imshow("Optimal Flow", resized_image)
    return tot_image

def pre_processing(image):
    """
    Helper function designed for any image pre-processing (ie blurring, sharpening, changing contrast)
    and conversion to gray-scale

    :param image, (746, 1024, 3) BGR image
    :return: proc_img, (746, 1024) gray-scale image
    """

    blurred = cv.GaussianBlur(image, (5, 5), 0)
    #proc_img = cv.addWeighted(image, 2, blurred, -1, 0)

    proc_img = blurred
    proc_img_gray = cv.cvtColor(proc_img, cv.COLOR_BGR2GRAY)

    return proc_img_gray, proc_img

class poisson:
    def __init__(self, XX, YY, mask):
        x_n = XX.shape[0]
        y_n = YY.shape[0]

        laplacian = sp.sparse.linalg.LaplacianNd((x_n, y_n), boundary_conditions = 'neumann').tosparse()

        self.x = XX[0, :]
        self.y = YY[:, 0]

        self.dx = np.diff(self.x)[0]
        self.dy = np.diff(self.y)[0]

        self.laplacian = laplacian
        self.mask = mask
        self.XX = XX
        self.YY = YY
        self.x_n = x_n
        self.y_n = y_n

    def remove_discon(self, flow, mask):

        flow_zero = flow.copy()
        flow_zero[~mask] = 0
        blurred_flow = sp.ndimage.gaussian_filter(flow_zero, 4)

        flow[~mask] = blurred_flow[~mask]

    def solve(self, flow2):
        flow = flow2.copy()

        mask = ~np.isnan(flow[..., 0]) #nan mask

        self.remove_discon(flow, mask)

        x = self.x
        y = self.y

        x_n = self.x_n
        y_n = self.y_n

        dx = self.dx
        dy = self.dy

        dudy, dudx = np.gradient(flow[..., 0], y, x, edge_order = 1)
        dvdy, dvdx = np.gradient(flow[..., 1], y, x, edge_order = 1)

        div_F = dudx + dvdy
        curl_F = dudy - dvdx

        b1 = div_F * dx * dy
        b2 = curl_F * dx * dy

        phi = sp.sparse.linalg.spsolve(self.laplacian, b1.flatten()).reshape(x_n, y_n)
        A = sp.sparse.linalg.spsolve(self.laplacian, b2.flatten()).reshape(x_n, y_n)

        phi_y, phi_x = np.gradient(phi, y, x, edge_order=2)
        A_y, A_x = np.gradient(A, y, x, edge_order=2)

        phi_x[~self.mask] = np.nan
        phi_y[~self.mask] = np.nan
        A_x[~self.mask] = np.nan
        A_y[~self.mask] = np.nan
        phi[~self.mask] = np.nan

        curl_free = np.stack([phi_x, phi_y], axis = -1)
        div_free = np.stack([A_y, -A_x], axis = -1)

        harm = flow - curl_free - div_free

        return curl_free, div_free + harm, harm




if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # Define the grid
    l = 201
    x = np.linspace(-2*np.pi, 2*np.pi, l)
    y = np.linspace(-2*np.pi, 2*np.pi, l)
    X, Y = np.meshgrid(x, y)

    a = 1

    import time

    x = np.linspace(-8 * np.pi, 8 * np.pi, l)
    y = np.linspace(-8 * np.pi, 8 * np.pi, l)
    X, Y = np.meshgrid(x, y)

    Fx = np.sin(X) + np.cos(Y) + 1
    Fy = np.cos(Y) - np.sin(X) - 1

    g = (np.abs(X)<3*np.pi) & (np.abs(Y)<3*np.pi)

    g = g.astype(int)
    Fx = Fx * g
    Fy = Fy * g


    mask = np.ones_like(X) == 1





    start = time.time()
    eq = poisson(X, Y, mask)
    d,c,h,_ = eq.solve(np.stack((Fx, Fy), axis = -1))

    print("damn bro you time is ", time.time()-start, "[sec.]")
    fig, axs = plt.subplots(2,2)


    s = None
    axs[0, 0].streamplot(X, Y, Fx, Fy, color='black',  density = 3)
    axs[0, 0].title.set_text('Original Field')

    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.grid(True)

    axs[1, 0].streamplot(X, Y, d[...,0], d[...,1], color='black', density = 3)
    #axs[1, 0].streamplot(X, Y, np.sin(X), np.cos(Y), color='r', cmap ='autumn')
    axs[1, 0].title.set_text('Curl Free')
    #print("should",Fx_div, "est", np.round(d[...,0],2))

    axs[0, 1].streamplot(X, Y, c[...,0], c[...,1], color='black', density = 3)
    #axs[0, 1].streamplot(X, Y, -Y, X, color='r')
    axs[0, 1].title.set_text('Div Free')

    axs[1, 1].streamplot(X, Y, h[...,0], h[...,1], color='black', density = 3)
    #axs[1, 1].quiver(X, Y, Fx_harm, Fy_harm, color='r',scale=s)
    axs[1, 1].title.set_text('Harmonic')
    plt.show()

