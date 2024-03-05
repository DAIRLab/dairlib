import scipy.signal


def make_notch(w0: float, Q: float, fs: float):
    n, d = scipy.signal.iirnotch(w0, Q, fs)
    z, p, k = scipy.signal.tf2zpk(n, d)
    sos = scipy.signal.zpk2sos(z, p, k)
    print(sos)


# def plot_butter():
#     import scipy.signal
#
#     center = 0.10
#     wn = (center, center + 0.02)
#     sos = scipy.signal.butter(
#         8, wn, btype='bandstop', output='sos'
#     )
#     ufilt = scipy.signal.sosfilt(sos, robot_output['u'])
#     ps = PlotStyler()
#     ps.plot(
#         robot_output['t_x'],
#         robot_output['u'][:, act_map['hip_roll_left_motor']]
#     )
#     ps.plot(
#         robot_output['t_x'],
#         ufilt[:, act_map['hip_roll_left_motor']]
#     )


if __name__ == '__main__':
    make_notch(53.0, 20, 1200)
