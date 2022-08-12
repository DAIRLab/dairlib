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

def process_raw_data(raw_data):
    
    print("Begin process the raw data.")
    
    # cut down some data in the begin and at the end to make sure processings like finite difference, interpolation won't have issue
    start_time = max(raw_data["states_info"]["t_x"][20], raw_data["contact_info"]["t_contact"][20])
    end_time = min(raw_data["states_info"]["t_x"][-20], raw_data["contact_info"]["t_contact"][-20])

    # processing robot output
    states_info = raw_data['states_info']
    t_states_info = states_info["t_x"]

    start_index = np.argwhere(t_states_info > start_time)[0][0]
    end_index = np.argwhere(t_states_info < end_time)[-1][0]
    t_states_info = t_states_info[start_index:end_index]
    q = states_info['q'][start_index:end_index,:]
    v = states_info['v'][start_index:end_index,:]
    u = states_info['u'][start_index:end_index,:]
    # obtained v_dot by finite diff
    smooth_window = 10
    v_dot = (states_info['v'][start_index+smooth_window:end_index+smooth_window,:] - states_info['v'][start_index-smooth_window:end_index-smooth_window,:])\
        /(states_info["t_x"][start_index+smooth_window:end_index+smooth_window] - states_info["t_x"][start_index-smooth_window:end_index-smooth_window])[:,None]
    
    v_dot = first_order_filter(v_dot)

    # processing contact info
    contact_output = raw_data['contact_info']
    t_contact = contact_output['t_contact']
    start_index = np.argwhere(t_contact > start_time - 0.01)[0][0] # make sure contact force period range cover the output states
    end_index = np.argwhere(t_contact < end_time + 0.01)[-1][0]
    t_contact = t_contact[start_index: end_index]
    is_contact = contact_output['is_contact'][start_index:end_index,:]
    is_contact_processed = []
    pointer = 0
    for t in t_states_info:
        while not (t_contact[pointer] <= t and t_contact[pointer+1] >=t):
            pointer+=1
        if abs(t_contact[pointer] - t) < abs(t_contact[pointer+1] - t):
            is_contact_processed.append(is_contact[pointer,:])
        else:
            is_contact_processed.append(is_contact[pointer+1,:])
    is_contact_processed = np.array(is_contact_processed)

    processed_data = {
        't':t_states_info,
        'q':q,
        'v':v,
        'v_dot':v_dot,
        'u':u,
        'is_contact':is_contact_processed,
    }

    print("Finish process data.")

    return processed_data

def get_residual(vdot_gt, vdot_est, cutting_f = 100):
    residuals = vdot_gt - vdot_est
    residuals_smoothed = first_order_filter(residuals, cutting_f=cutting_f)
    return residuals_smoothed