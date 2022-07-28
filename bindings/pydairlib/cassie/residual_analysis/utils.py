import numpy as np
import scipy.fft

def get_freq_domain(signal, Ts=0.0005):
    n = signal.shape[0]

    signal_f = np.abs(scipy.fft.fft(signal))[0:n//2] / n
    signal_f[1:] = signal_f[1:]*2

    freq = scipy.fft.fftfreq(n, Ts)[:n//2]

    return signal_f, freq

def first_order_filter(orginal_signals, cutting_f=100, Ts=0.0005):
        a = np.exp(-2*np.pi*cutting_f*Ts)
        filtered_signals = [orginal_signals[0]]
        for i in range(1, orginal_signals.shape[0]):
            filtered_signals.append(a * filtered_signals[-1] + (1-a) * orginal_signals[i])
        filtered_signals = np.array(filtered_signals)
        return filtered_signals 

def interpolation(t,t1,t2,v1,v2):
    ratio = (t - t1)/(t2-t1)
    v = v1 + ratio * (v2 -v1)
    return v

def process_raw_data(raw_data, start_time, end_time):
    
    print("Begin process the data.")

    if start_time is None or end_time is None:
        start_time = max(raw_data['robot_output']["t_x"][0][0][0][100], raw_data['contact_output']['t_contact'][0][0][0][100], raw_data['osc_output']['t_osc'][0][0][0][100])
        end_time = min(raw_data['robot_output']["t_x"][0][0][0][-100], raw_data['contact_output']['t_contact'][0][0][0][-100], raw_data['osc_output']['t_osc'][0][0][0][-100])

    print("start time:{}, end time:{}".format(start_time, end_time))

    # processing robot output
    robot_output = raw_data['robot_output']
    t_robot_output = robot_output["t_x"][0][0][0]

    start_index = np.argwhere(t_robot_output > start_time)[0][0]
    end_index = np.argwhere(t_robot_output < end_time)[-1][0]
    t_robot_output = t_robot_output[start_index:end_index]
    q = robot_output['q'][0][0][start_index:end_index,:]
    v = robot_output['v'][0][0][start_index:end_index,:]
    u = robot_output['u'][0][0][start_index:end_index,:]
    # obtained v_dot by finite diff
    smooth_window = 10
    v_dot = (robot_output['v'][0][0][start_index+smooth_window:end_index+smooth_window,:] - robot_output['v'][0][0][start_index-smooth_window:end_index-smooth_window,:])\
        /(robot_output["t_x"][0][0][0][start_index+smooth_window:end_index+smooth_window] - robot_output["t_x"][0][0][0][start_index-smooth_window:end_index-smooth_window])[:,None]
    
    v_dot = first_order_filter(v_dot)

    # processing contact force
    contact_output = raw_data['contact_output']
    t_contact = contact_output['t_contact'][0][0][0]
    start_index = np.argwhere(t_contact > start_time - 0.01)[0][0] # make sure contact force period range cover the output states
    end_index = np.argwhere(t_contact < end_time + 0.01)[-1][0]
    t_contact = t_contact[start_index: end_index]
    is_contact = contact_output['is_contact'][0][0][start_index:end_index,:]
    is_contact_processed = []
    pointer = 0
    for t in t_robot_output:
        while not (t_contact[pointer] <= t and t_contact[pointer+1] >=t):
            pointer+=1
        if abs(t_contact[pointer] - t) < abs(t_contact[pointer+1] - t):
            is_contact_processed.append(is_contact[pointer,:])
        else:
            is_contact_processed.append(is_contact[pointer+1,:])
    is_contact_processed = np.array(is_contact_processed)

    # Get the osc_output

    osc_output = raw_data["osc_output"]
    t_osc = osc_output["t_osc"][0][0][0]
    start_index = np.argwhere(t_osc > start_time - 0.01)[0][0]
    end_index = np.argwhere(t_osc < end_time + 0.01)[-1][0]
    t_osc = t_osc[start_index: end_index]
    u_osc = osc_output["u_sol"][0][0][start_index:end_index,:]
    u_osc_processed = []
    v_dot_osc = osc_output["dv_sol"][0][0][start_index:end_index,:]
    v_dot_osc_proccessed = []
    pointer = 0
    for t in t_robot_output:
        while not (t_osc[pointer] <= t and t_osc[pointer+1] >=t):
            pointer+=1
        u_osc_interpolated = interpolation(t, t_osc[pointer], t_osc[pointer+1],
                                                u_osc[pointer,:], u_osc[pointer+1, :])
        v_dot_osc_interpolated = interpolation(t, t_osc[pointer], t_osc[pointer+1],
                                                v_dot_osc[pointer,:], v_dot_osc[pointer+1, :])
        u_osc_processed.append(u_osc_interpolated)
        v_dot_osc_proccessed.append(v_dot_osc_interpolated)
    u_osc_processed = np.array(u_osc_processed)
    v_dot_osc_proccessed = np.array(v_dot_osc_proccessed)

    # Get damping ratio
    damping_ratio = raw_data['damping_ratio']

    # Get spring stiffness
    spring_stiffness = raw_data['spring_stiffness']

    processed_data = {
        't':t_robot_output,
        'q':q,
        'v':v,
        'v_dot':v_dot,
        'u':u,
        'is_contact':is_contact_processed,
        'u_osc': u_osc_processed,
        'v_dot_osc': v_dot_osc_proccessed,
        'damping_ratio':damping_ratio,
        'spring_stiffness':spring_stiffness
    }

    print("Finish process data.")

    return processed_data

def get_residual(vdot_gt, vdot_est, cutting_f = 100):
    residuals = vdot_gt - vdot_est
    residuals_smoothed = first_order_filter(residuals, cutting_f=cutting_f)
    return residuals_smoothed