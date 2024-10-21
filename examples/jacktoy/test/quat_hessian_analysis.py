'''Analysis of the symbolically computed Hessian of the squared angle between a
current and a desired quaternion.  Compares the raw output from the symbolic
result to an edited version with manual simplifications to reuse compute.  This
file is quick to run.'''

import numpy as np
import sympy as sym
import time


# The following have minimal edits from the symbolic output of
# quat_hessian_symbolic.py.
def H_ww_func(q_np, r_np):
    assert q_np.shape == r_np.shape == (4,)
    [q_w, q_x, q_y, q_z] = q_np
    [r_w, r_x, r_y, r_z] = r_np
    return 8*(-(2*q_w*(q_w*q_x*r_x + q_w*q_y*r_y + q_w*q_z*r_z - q_x**2*r_w - q_y**2*r_w - q_z**2*r_w)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) - (q_x*r_x + q_y*r_y + q_z*r_z)*(q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) + (q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w*r_x**2 + q_w*r_y**2 + q_w*r_z**2 - q_x*r_w*r_x - q_y*r_w*r_y - q_z*r_w*r_z)*(q_w*q_x*r_x + q_w*q_y*r_y + q_w*q_z*r_z - q_x**2*r_w - q_y**2*r_w - q_z**2*r_w))*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)*sym.atan2(sym.sqrt((q_w*r_x - q_x*r_w + q_y*r_z - q_z*r_y)**2 + (q_w*r_y - q_x*r_z - q_y*r_w + q_z*r_x)**2 + (q_w*r_z + q_x*r_y - q_y*r_x - q_z*r_w)**2), q_w*r_w + q_x*r_x + q_y*r_y + q_z*r_z) + (q_w*q_x*r_x + q_w*q_y*r_y + q_w*q_z*r_z - q_x**2*r_w - q_y**2*r_w - q_z**2*r_w)**2*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(3/2))/((q_w**2 + q_x**2 + q_y**2 + q_z**2)**2*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(5/2))

def H_xx_func(q_np, r_np):
    assert q_np.shape == r_np.shape == (4,)
    [q_w, q_x, q_y, q_z] = q_np
    [r_w, r_x, r_y, r_z] = r_np
    return 8*((2*q_x*(q_w**2*r_x - q_w*q_x*r_w - q_x*q_y*r_y - q_x*q_z*r_z + q_y**2*r_x + q_z**2*r_x)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) + (q_w*r_w + q_y*r_y + q_z*r_z)*(q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) - (q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w**2*r_x - q_w*q_x*r_w - q_x*q_y*r_y - q_x*q_z*r_z + q_y**2*r_x + q_z**2*r_x)*(q_w*r_w*r_x - q_x*r_w**2 - q_x*r_y**2 - q_x*r_z**2 + q_y*r_x*r_y + q_z*r_x*r_z))*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)*sym.atan2(sym.sqrt((q_w*r_x - q_x*r_w + q_y*r_z - q_z*r_y)**2 + (q_w*r_y - q_x*r_z - q_y*r_w + q_z*r_x)**2 + (q_w*r_z + q_x*r_y - q_y*r_x - q_z*r_w)**2), q_w*r_w + q_x*r_x + q_y*r_y + q_z*r_z) + (q_w**2*r_x - q_w*q_x*r_w - q_x*q_y*r_y - q_x*q_z*r_z + q_y**2*r_x + q_z**2*r_x)**2*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(3/2))/((q_w**2 + q_x**2 + q_y**2 + q_z**2)**2*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(5/2))

def H_yy_func(q_np, r_np):
    assert q_np.shape == r_np.shape == (4,)
    [q_w, q_x, q_y, q_z] = q_np
    [r_w, r_x, r_y, r_z] = r_np
    return 8*((2*q_y*(q_w**2*r_y - q_w*q_y*r_w + q_x**2*r_y - q_x*q_y*r_x - q_y*q_z*r_z + q_z**2*r_y)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) + (q_w*r_w + q_x*r_x + q_z*r_z)*(q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) - (q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w**2*r_y - q_w*q_y*r_w + q_x**2*r_y - q_x*q_y*r_x - q_y*q_z*r_z + q_z**2*r_y)*(q_w*r_w*r_y + q_x*r_x*r_y - q_y*r_w**2 - q_y*r_x**2 - q_y*r_z**2 + q_z*r_y*r_z))*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)*sym.atan2(sym.sqrt((q_w*r_x - q_x*r_w + q_y*r_z - q_z*r_y)**2 + (q_w*r_y - q_x*r_z - q_y*r_w + q_z*r_x)**2 + (q_w*r_z + q_x*r_y - q_y*r_x - q_z*r_w)**2), q_w*r_w + q_x*r_x + q_y*r_y + q_z*r_z) + (q_w**2*r_y - q_w*q_y*r_w + q_x**2*r_y - q_x*q_y*r_x - q_y*q_z*r_z + q_z**2*r_y)**2*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(3/2))/((q_w**2 + q_x**2 + q_y**2 + q_z**2)**2*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(5/2))

def H_zz_func(q_np, r_np):
    assert q_np.shape == r_np.shape == (4,)
    [q_w, q_x, q_y, q_z] = q_np
    [r_w, r_x, r_y, r_z] = r_np
    return 8*((2*q_z*(q_w**2*r_z - q_w*q_z*r_w + q_x**2*r_z - q_x*q_z*r_x + q_y**2*r_z - q_y*q_z*r_y)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) + (q_w*r_w + q_x*r_x + q_y*r_y)*(q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) - (q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w**2*r_z - q_w*q_z*r_w + q_x**2*r_z - q_x*q_z*r_x + q_y**2*r_z - q_y*q_z*r_y)*(q_w*r_w*r_z + q_x*r_x*r_z + q_y*r_y*r_z - q_z*r_w**2 - q_z*r_x**2 - q_z*r_y**2))*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)*sym.atan2(sym.sqrt((q_w*r_x - q_x*r_w + q_y*r_z - q_z*r_y)**2 + (q_w*r_y - q_x*r_z - q_y*r_w + q_z*r_x)**2 + (q_w*r_z + q_x*r_y - q_y*r_x - q_z*r_w)**2), q_w*r_w + q_x*r_x + q_y*r_y + q_z*r_z) + (q_w**2*r_z - q_w*q_z*r_w + q_x**2*r_z - q_x*q_z*r_x + q_y**2*r_z - q_y*q_z*r_y)**2*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(3/2))/((q_w**2 + q_x**2 + q_y**2 + q_z**2)**2*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(5/2))

def H_wx_func(q_np, r_np):
    assert q_np.shape == r_np.shape == (4,)
    [q_w, q_x, q_y, q_z] = q_np
    [r_w, r_x, r_y, r_z] = r_np
    return 8*((-2*q_x*(q_w*q_x*r_x + q_w*q_y*r_y + q_w*q_z*r_z - q_x**2*r_w - q_y**2*r_w - q_z**2*r_w)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) + (q_w*r_x - 2*q_x*r_w)*(q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) + (q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w*q_x*r_x + q_w*q_y*r_y + q_w*q_z*r_z - q_x**2*r_w - q_y**2*r_w - q_z**2*r_w)*(q_w*r_w*r_x - q_x*r_w**2 - q_x*r_y**2 - q_x*r_z**2 + q_y*r_x*r_y + q_z*r_x*r_z))*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)*sym.atan2(sym.sqrt((q_w*r_x - q_x*r_w + q_y*r_z - q_z*r_y)**2 + (q_w*r_y - q_x*r_z - q_y*r_w + q_z*r_x)**2 + (q_w*r_z + q_x*r_y - q_y*r_x - q_z*r_w)**2), q_w*r_w + q_x*r_x + q_y*r_y + q_z*r_z) - (q_w**2*r_x - q_w*q_x*r_w - q_x*q_y*r_y - q_x*q_z*r_z + q_y**2*r_x + q_z**2*r_x)*(q_w*q_x*r_x + q_w*q_y*r_y + q_w*q_z*r_z - q_x**2*r_w - q_y**2*r_w - q_z**2*r_w)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(3/2))/((q_w**2 + q_x**2 + q_y**2 + q_z**2)**2*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(5/2))

def H_wy_func(q_np, r_np):
    assert q_np.shape == r_np.shape == (4,)
    [q_w, q_x, q_y, q_z] = q_np
    [r_w, r_x, r_y, r_z] = r_np
    return 8*((-2*q_y*(q_w*q_x*r_x + q_w*q_y*r_y + q_w*q_z*r_z - q_x**2*r_w - q_y**2*r_w - q_z**2*r_w)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) + (q_w*r_y - 2*q_y*r_w)*(q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) + (q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w*q_x*r_x + q_w*q_y*r_y + q_w*q_z*r_z - q_x**2*r_w - q_y**2*r_w - q_z**2*r_w)*(q_w*r_w*r_y + q_x*r_x*r_y - q_y*r_w**2 - q_y*r_x**2 - q_y*r_z**2 + q_z*r_y*r_z))*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)*sym.atan2(sym.sqrt((q_w*r_x - q_x*r_w + q_y*r_z - q_z*r_y)**2 + (q_w*r_y - q_x*r_z - q_y*r_w + q_z*r_x)**2 + (q_w*r_z + q_x*r_y - q_y*r_x - q_z*r_w)**2), q_w*r_w + q_x*r_x + q_y*r_y + q_z*r_z) - (q_w**2*r_y - q_w*q_y*r_w + q_x**2*r_y - q_x*q_y*r_x - q_y*q_z*r_z + q_z**2*r_y)*(q_w*q_x*r_x + q_w*q_y*r_y + q_w*q_z*r_z - q_x**2*r_w - q_y**2*r_w - q_z**2*r_w)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(3/2))/((q_w**2 + q_x**2 + q_y**2 + q_z**2)**2*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(5/2))

def H_wz_func(q_np, r_np):
    assert q_np.shape == r_np.shape == (4,)
    [q_w, q_x, q_y, q_z] = q_np
    [r_w, r_x, r_y, r_z] = r_np
    return 8*((-2*q_z*(q_w*q_x*r_x + q_w*q_y*r_y + q_w*q_z*r_z - q_x**2*r_w - q_y**2*r_w - q_z**2*r_w)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) + (q_w*r_z - 2*q_z*r_w)*(q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) + (q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w*q_x*r_x + q_w*q_y*r_y + q_w*q_z*r_z - q_x**2*r_w - q_y**2*r_w - q_z**2*r_w)*(q_w*r_w*r_z + q_x*r_x*r_z + q_y*r_y*r_z - q_z*r_w**2 - q_z*r_x**2 - q_z*r_y**2))*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)*sym.atan2(sym.sqrt((q_w*r_x - q_x*r_w + q_y*r_z - q_z*r_y)**2 + (q_w*r_y - q_x*r_z - q_y*r_w + q_z*r_x)**2 + (q_w*r_z + q_x*r_y - q_y*r_x - q_z*r_w)**2), q_w*r_w + q_x*r_x + q_y*r_y + q_z*r_z) - (q_w**2*r_z - q_w*q_z*r_w + q_x**2*r_z - q_x*q_z*r_x + q_y**2*r_z - q_y*q_z*r_y)*(q_w*q_x*r_x + q_w*q_y*r_y + q_w*q_z*r_z - q_x**2*r_w - q_y**2*r_w - q_z**2*r_w)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(3/2))/((q_w**2 + q_x**2 + q_y**2 + q_z**2)**2*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(5/2))

def H_xy_func(q_np, r_np):
    assert q_np.shape == r_np.shape == (4,)
    [q_w, q_x, q_y, q_z] = q_np
    [r_w, r_x, r_y, r_z] = r_np
    return 8*((2*q_y*(q_w**2*r_x - q_w*q_x*r_w - q_x*q_y*r_y - q_x*q_z*r_z + q_y**2*r_x + q_z**2*r_x)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) + (q_x*r_y - 2*q_y*r_x)*(q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) - (q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w**2*r_x - q_w*q_x*r_w - q_x*q_y*r_y - q_x*q_z*r_z + q_y**2*r_x + q_z**2*r_x)*(q_w*r_w*r_y + q_x*r_x*r_y - q_y*r_w**2 - q_y*r_x**2 - q_y*r_z**2 + q_z*r_y*r_z))*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)*sym.atan2(sym.sqrt((q_w*r_x - q_x*r_w + q_y*r_z - q_z*r_y)**2 + (q_w*r_y - q_x*r_z - q_y*r_w + q_z*r_x)**2 + (q_w*r_z + q_x*r_y - q_y*r_x - q_z*r_w)**2), q_w*r_w + q_x*r_x + q_y*r_y + q_z*r_z) + (q_w**2*r_x - q_w*q_x*r_w - q_x*q_y*r_y - q_x*q_z*r_z + q_y**2*r_x + q_z**2*r_x)*(q_w**2*r_y - q_w*q_y*r_w + q_x**2*r_y - q_x*q_y*r_x - q_y*q_z*r_z + q_z**2*r_y)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(3/2))/((q_w**2 + q_x**2 + q_y**2 + q_z**2)**2*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(5/2))

def H_xz_func(q_np, r_np):
    assert q_np.shape == r_np.shape == (4,)
    [q_w, q_x, q_y, q_z] = q_np
    [r_w, r_x, r_y, r_z] = r_np
    return 8*((2*q_z*(q_w**2*r_x - q_w*q_x*r_w - q_x*q_y*r_y - q_x*q_z*r_z + q_y**2*r_x + q_z**2*r_x)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) + (q_x*r_z - 2*q_z*r_x)*(q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) - (q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w**2*r_x - q_w*q_x*r_w - q_x*q_y*r_y - q_x*q_z*r_z + q_y**2*r_x + q_z**2*r_x)*(q_w*r_w*r_z + q_x*r_x*r_z + q_y*r_y*r_z - q_z*r_w**2 - q_z*r_x**2 - q_z*r_y**2))*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)*sym.atan2(sym.sqrt((q_w*r_x - q_x*r_w + q_y*r_z - q_z*r_y)**2 + (q_w*r_y - q_x*r_z - q_y*r_w + q_z*r_x)**2 + (q_w*r_z + q_x*r_y - q_y*r_x - q_z*r_w)**2), q_w*r_w + q_x*r_x + q_y*r_y + q_z*r_z) + (q_w**2*r_x - q_w*q_x*r_w - q_x*q_y*r_y - q_x*q_z*r_z + q_y**2*r_x + q_z**2*r_x)*(q_w**2*r_z - q_w*q_z*r_w + q_x**2*r_z - q_x*q_z*r_x + q_y**2*r_z - q_y*q_z*r_y)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(3/2))/((q_w**2 + q_x**2 + q_y**2 + q_z**2)**2*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(5/2))

def H_yz_func(q_np, r_np):
    assert q_np.shape == r_np.shape == (4,)
    [q_w, q_x, q_y, q_z] = q_np
    [r_w, r_x, r_y, r_z] = r_np
    return 8*((2*q_z*(q_w**2*r_y - q_w*q_y*r_w + q_x**2*r_y - q_x*q_y*r_x - q_y*q_z*r_z + q_z**2*r_y)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) + (q_y*r_z - 2*q_z*r_y)*(q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2) - (q_w**2 + q_x**2 + q_y**2 + q_z**2)*(q_w**2*r_y - q_w*q_y*r_w + q_x**2*r_y - q_x*q_y*r_x - q_y*q_z*r_z + q_z**2*r_y)*(q_w*r_w*r_z + q_x*r_x*r_z + q_y*r_y*r_z - q_z*r_w**2 - q_z*r_x**2 - q_z*r_y**2))*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)*sym.atan2(sym.sqrt((q_w*r_x - q_x*r_w + q_y*r_z - q_z*r_y)**2 + (q_w*r_y - q_x*r_z - q_y*r_w + q_z*r_x)**2 + (q_w*r_z + q_x*r_y - q_y*r_x - q_z*r_w)**2), q_w*r_w + q_x*r_x + q_y*r_y + q_z*r_z) + (q_w**2*r_y - q_w*q_y*r_w + q_x**2*r_y - q_x*q_y*r_x - q_y*q_z*r_z + q_z**2*r_y)*(q_w**2*r_z - q_w*q_z*r_w + q_x**2*r_z - q_x*q_z*r_x + q_y**2*r_z - q_y*q_z*r_y)*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(3/2))/((q_w**2 + q_x**2 + q_y**2 + q_z**2)**2*(q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - 2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - 2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z + q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - 2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2)**(5/2))

# Combine into one function with 4x4 output.
def H_from_symbolic(q_np, r_np):
    q = q_np.squeeze()
    r = r_np.squeeze()
    assert q.shape == r.shape == (4,)
    return np.array(
        [[H_ww_func(q, r), H_wx_func(q, r), H_wy_func(q, r), H_wz_func(q, r)],
         [H_wx_func(q, r), H_xx_func(q, r), H_xy_func(q, r), H_xz_func(q, r)],
         [H_wy_func(q, r), H_xy_func(q, r), H_yy_func(q, r), H_yz_func(q, r)],
         [H_wz_func(q, r), H_xz_func(q, r), H_yz_func(q, r), H_zz_func(q, r)]])


# This is a manually adjusted version that pre-computes terms that would
# otherwise need to be duplicately computed.  This matches to > 1e-12 precision.
def H_with_manual_edits(q_np, r_np):
    assert q_np.shape == r_np.shape == (4,)

    [q_w, q_x, q_y, q_z] = q_np
    [r_w, r_x, r_y, r_z] = r_np

    exp_1 = np.arctan2(np.sqrt((q_w*r_x - q_x*r_w + q_y*r_z - q_z*r_y)**2 + \
                               (q_w*r_y - q_x*r_z - q_y*r_w + q_z*r_x)**2 + \
                               (q_w*r_z + q_x*r_y - q_y*r_x - q_z*r_w)**2),
                       q_w*r_w + q_x*r_x + q_y*r_y + q_z*r_z)
    exp_2 = q_w**2*r_x**2 + q_w**2*r_y**2 + q_w**2*r_z**2 - \
        2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z + \
        q_x**2*r_w**2 + q_x**2*r_y**2 + q_x**2*r_z**2 - \
        2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z
    exp_3 = q_y**2*r_w**2 + q_y**2*r_x**2 + q_y**2*r_z**2 - \
        2*q_y*q_z*r_y*r_z + q_z**2*r_w**2 + q_z**2*r_x**2 + q_z**2*r_y**2
    exp_4 = q_w**2 + q_x**2 + q_y**2 + q_z**2
    exp_5 = (exp_4)**2*(exp_2 + exp_3)**(5/2)
    exp_6 = (exp_2 + exp_3)**(3/2)
    exp_7 = q_w*q_x*r_x + q_w*q_y*r_y + q_w*q_z*r_z - q_x**2*r_w - q_y**2*r_w \
        - q_z**2*r_w
    exp_8 = q_w**2*r_y - q_w*q_y*r_w + q_x**2*r_y - q_x*q_y*r_x - q_y*q_z*r_z \
        + q_z**2*r_y
    exp_9 = q_w**2*r_x - q_w*q_x*r_w - q_x*q_y*r_y - q_x*q_z*r_z + q_y**2*r_x \
        + q_z**2*r_x
    exp_10 = q_w*r_w*r_z + q_x*r_x*r_z + q_y*r_y*r_z - q_z*r_w**2 - q_z*r_x**2 \
        - q_z*r_y**2
    exp_11 = q_w**2*r_z - q_w*q_z*r_w + q_x**2*r_z - q_x*q_z*r_x + q_y**2*r_z \
        - q_y*q_z*r_y
    exp_12 = q_w*r_w*r_y + q_x*r_x*r_y - q_y*r_w**2 - q_y*r_x**2 - q_y*r_z**2 \
        + q_z*r_y*r_z
    exp_13 = q_w*r_w*r_x - q_x*r_w**2 - q_x*r_y**2 - q_x*r_z**2 + q_y*r_x*r_y \
        + q_z*r_x*r_z

    H_ww = 8*(-(2*q_w*(exp_7)*(exp_2 + exp_3) \
        - (q_x*r_x + q_y*r_y + q_z*r_z)*(exp_4)*(exp_2 + exp_3) \
        + (exp_4)*(q_w*r_x**2 + q_w*r_y**2 + q_w*r_z**2 - q_x*r_w*r_x \
                   - q_y*r_w*r_y - q_z*r_w*r_z)*(exp_7))*(exp_2 + exp_3)*exp_1 \
                    + (exp_7)**2*(exp_6))/(exp_5)
    H_xx = 8*((2*q_x*(exp_9)*(exp_2 + exp_3) \
        + (q_w*r_w + q_y*r_y + q_z*r_z)*(exp_4)*(exp_2 + exp_3) \
        - (exp_4)*(exp_9)*(exp_13))*(exp_2 + exp_3)*exp_1 \
            + (exp_9)**2*(exp_6))/(exp_5)
    H_yy = 8*((2*q_y*(exp_8)*(exp_2 + exp_3) \
        + (q_w*r_w + q_x*r_x + q_z*r_z)*(exp_4)*(exp_2 + exp_3) \
        - (exp_4)*(exp_8)*(exp_12))*(exp_2 + exp_3)*exp_1 \
            + (exp_8)**2*(exp_6))/(exp_5)
    H_zz = 8*((2*q_z*(exp_11)*(exp_2 + exp_3) \
        + (q_w*r_w + q_x*r_x + q_y*r_y)*(exp_4)*(exp_2 + exp_3) \
        - (exp_4)*(exp_11)*((exp_10)))*(exp_2 + exp_3)*exp_1 \
            + (exp_11)**2*(exp_6))/(exp_5)
    H_wx = 8*((-2*q_x*(exp_7)*(exp_2 + exp_3) \
        + (q_w*r_x - 2*q_x*r_w)*(exp_4)*(exp_2 + exp_3) \
        + (exp_4)*(exp_7)*(exp_13))*(exp_2 + exp_3)*exp_1 \
            - (exp_9)*(exp_7)*(exp_6))/(exp_5)
    H_wy = 8*((-2*q_y*(exp_7)*(exp_2 + exp_3) \
        + (q_w*r_y - 2*q_y*r_w)*(exp_4)*(exp_2 + exp_3) \
        + (exp_4)*(exp_7)*(exp_12))*(exp_2 + exp_3)*exp_1 \
            - (exp_8)*(exp_7)*(exp_6))/(exp_5)
    H_wz = 8*((-2*q_z*(exp_7)*(exp_2 + exp_3) \
        + (q_w*r_z - 2*q_z*r_w)*(exp_4)*(exp_2 + exp_3) \
        + (exp_4)*(exp_7)*((exp_10)))*(exp_2 + exp_3)*exp_1 \
            - (exp_11)*(exp_7)*(exp_6))/(exp_5)
    H_xy = 8*((2*q_y*(exp_9)*(exp_2 + exp_3) \
        + (q_x*r_y - 2*q_y*r_x)*(exp_4)*(exp_2 + exp_3) \
        - (exp_4)*(exp_9)*(exp_12))*(exp_2 + exp_3)*exp_1 \
            + (exp_9)*(exp_8)*(exp_6))/(exp_5)
    H_xz = 8*((2*q_z*(exp_9)*(exp_2 + exp_3) \
        + (q_x*r_z - 2*q_z*r_x)*(exp_4)*(exp_2 + exp_3) \
        - (exp_4)*(exp_9)*((exp_10)))*(exp_2 + exp_3)*exp_1 \
            + (exp_9)*(exp_11)*(exp_6))/(exp_5)
    H_yz = 8*((2*q_z*(exp_8)*(exp_2 + exp_3) \
        + (q_y*r_z - 2*q_z*r_y)*(exp_4)*(exp_2 + exp_3) \
        - (exp_4)*(exp_8)*((exp_10)))*(exp_2 + exp_3)*exp_1 \
            + (exp_8)*(exp_11)*(exp_6))/(exp_5)

    H = np.array([[H_ww, H_wx, H_wy, H_wz],
                  [H_wx, H_xx, H_xy, H_xz],
                  [H_wy, H_xy, H_yy, H_yz],
                  [H_wz, H_xz, H_yz, H_zz]])
    return H


# Perform tests to ensure the simplified version is correct and faster to
# evaluate.
original_durations = []
simplified_durations = []
for _ in range(20):
    q_np = np.random.rand(4)
    r_np = np.random.rand(4)
    r_np = r_np / np.linalg.norm(r_np)  # Ensure the desired quaternion is unit
                                        # length.

    # Time the original.
    start = time.time()
    H_original = np.array(H_from_symbolic(q_np, r_np), dtype=np.float64)
    original_durations.append(time.time() - start)

    # Time the simplified version.
    start = time.time()
    H_simplified = H_with_manual_edits(q_np, r_np)
    simplified_durations.append(time.time() - start)

    # Compare.
    print(f'Error: {np.linalg.norm(H_original - H_simplified):.10f},', end=' ')
    print(f'Duration: {original_durations[-1]*1e6:.2f} µs versus ' + \
          f'{simplified_durations[-1]*1e6:.2f} µs,', end=' ')
    print(f'Determinant: {np.linalg.det(H_original):.2f}')
