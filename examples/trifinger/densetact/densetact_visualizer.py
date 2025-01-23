import numpy as np
import cv2 as cv

class graphical:
    def __init__(self, name, densetact):
        self.dis_RIO = densetact['rad_OI']
        self.center = densetact['center']
        self.name = name

    def legend(self, symbol):
        pass

    def __crop(self, frame):
        [c_y, c_x] = self.center
        d = self.dis_RIO

        # cropping
        b_x = (int(c_x - d), int(c_x + d))
        b_y = (int(c_y - d), int(c_y + d))

        return frame

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

        # for point in flow_data['feat_pos']:
        #     cv.circle(arrow_frame,
        #               (int(point[0]), int(point[1])),
        #               1,
        #               (0,0,0),
        #                -1)

        og_arrow = arrow_frame.copy()
            
        for contant_point in flow_data['contact_points']:
            cv.circle(arrow_frame,
                      (int(contant_point[0]), int(contant_point[1])),
                      1,
                      (255,0,0),
                       5)
        arrow_frame = cv.addWeighted(arrow_frame, 0.5, og_arrow, 0.5, 0)

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