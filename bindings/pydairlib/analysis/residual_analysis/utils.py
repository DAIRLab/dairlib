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

def get_residual(vdot_gt, vdot_est, cutting_f = 100):
    residuals = vdot_gt - vdot_est
    residuals_smoothed = first_order_filter(residuals, cutting_f=cutting_f)
    return residuals_smoothed