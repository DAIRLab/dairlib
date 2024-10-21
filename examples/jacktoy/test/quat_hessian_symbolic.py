'''Symbolic computation of the Hessian of the squared angle between a current
and a desired quaternion.  This file can take around a half hour to run.'''

import sympy as sym
import numpy as np


def sym_norm(q):
    return sym.sqrt(np.dot(q, q))


# 4D definition.
q_w, q_x, q_y, q_z, r_w, r_x, r_y, r_z, gamma = \
    sym.symbols('q_w q_x q_y q_z r_w r_x r_y r_z gamma')
q_np = np.array([q_w, q_x, q_y, q_z])  # Norm of q is gamma (this doesn't end up
                                       # affecting anything, but included for
                                       # clarity).
r_np = np.array([r_w, r_x, r_y, r_z])  # Norm of r is 1.

# Define some simplifications.
subs_dict = {sym_norm(r_np): 1,
             sym_norm(r_np)**2: 1,
             sym_norm(q_np): gamma,
             sym_norm(q_np)**2: gamma**2}

# Use sympy's quaternion library to compute the angle difference.
q = sym.Quaternion(q_w, q_x, q_y, q_z)
r = sym.Quaternion(r_w, r_x, r_y, r_z)

q_rel = q.mul(r.inverse())
q_rel = q_rel.subs(subs_dict).simplify()
theta = q_rel.angle()

# Solve for partial derivatives of the angle.
print('Solving for dth_dw...', end=' ', flush=True)
dth_dw = sym.diff(theta, q_w).simplify()
print('Done.')
print('Solving for dth_dx...', end=' ', flush=True)
dth_dx = sym.diff(theta, q_x).simplify()
print('Done.')
print('Solving for dth_dy...', end=' ', flush=True)
dth_dy = sym.diff(theta, q_y).simplify()
print('Done.')
print('Solving for dth_dz...', end=' ', flush=True)
dth_dz = sym.diff(theta, q_z).simplify()
print('Done.')

print('Solving for ddth_dww...', end=' ', flush=True)
ddth_dww = sym.diff(dth_dw, q_w)
print('Done.')
print('Solving for ddth_dwx...', end=' ', flush=True)
ddth_dwx = sym.diff(dth_dw, q_x)
print('Done.')
print('Solving for ddth_dwy...', end=' ', flush=True)
ddth_dwy = sym.diff(dth_dw, q_y)
print('Done.')
print('Solving for ddth_dwz...', end=' ', flush=True)
ddth_dwz = sym.diff(dth_dw, q_z)
print('Done.')
print('Solving for ddth_dxx...', end=' ', flush=True)
ddth_dxx = sym.diff(dth_dx, q_x)
print('Done.')
print('Solving for ddth_dyy...', end=' ', flush=True)
ddth_dyy = sym.diff(dth_dy, q_y)
print('Done.')
print('Solving for ddth_dzz...', end=' ', flush=True)
ddth_dzz = sym.diff(dth_dz, q_z)
print('Done.')
print('Solving for ddth_dxy...', end=' ', flush=True)
ddth_dxy = sym.diff(dth_dx, q_y)
print('Done.')
print('Solving for ddth_dxz...', end=' ', flush=True)
ddth_dxz = sym.diff(dth_dx, q_z)
print('Done.')
print('Solving for ddth_dyz...', end=' ', flush=True)
ddth_dyz = sym.diff(dth_dy, q_z)
print('Done.')

print(f'\nUNSIMPLIFIED DERIVATIVE RESULTS:')
print(f'{dth_dw=}\n')
print(f'{dth_dx=}\n')
print(f'{dth_dy=}\n')
print(f'{dth_dz=}\n')
print(f'{ddth_dww=}\n')
print(f'{ddth_dwx=}\n')
print(f'{ddth_dwy=}\n')
print(f'{ddth_dwz=}\n')
print(f'{ddth_dxx=}\n')
print(f'{ddth_dyy=}\n')
print(f'{ddth_dzz=}\n')
print(f'{ddth_dxy=}\n')
print(f'{ddth_dxz=}\n')
print(f'{ddth_dyz=}\n')

# These simplifications take ~2 minutes each to run.
print('Simplifying dth_dw...', end=' ', flush=True)
dth_dw = dth_dw.simplify()
print('Done.')
print('Simplifying dth_dx...', end=' ', flush=True)
dth_dx = dth_dx.simplify()
print('Done.')
print('Simplifying dth_dy...', end=' ', flush=True)
dth_dy = dth_dy.simplify()
print('Done.')
print('Simplifying dth_dz...', end=' ', flush=True)
dth_dz = dth_dz.simplify()
print('Done.')
print('Simplifying ddth_dww...', end=' ', flush=True)
ddth_dww = ddth_dww.simplify()
print('Done.')
print('Simplifying ddth_dwx...', end=' ', flush=True)
ddth_dwx = ddth_dwx.simplify()
print('Done.')
print('Simplifying ddth_dwy...', end=' ', flush=True)
ddth_dwy = ddth_dwy.simplify()
print('Done.')
print('Simplifying ddth_dwz...', end=' ', flush=True)
ddth_dwz = ddth_dwz.simplify()
print('Done.')
print('Simplifying ddth_dxx...', end=' ', flush=True)
ddth_dxx = ddth_dxx.simplify()
print('Done.')
print('Simplifying ddth_dyy...', end=' ', flush=True)
ddth_dyy = ddth_dyy.simplify()
print('Done.')
print('Simplifying ddth_dzz...', end=' ', flush=True)
ddth_dzz = ddth_dzz.simplify()
print('Done.')
print('Simplifying ddth_dxy...', end=' ', flush=True)
ddth_dxy = ddth_dxy.simplify()
print('Done.')
print('Simplifying ddth_dxz...', end=' ', flush=True)
ddth_dxz = ddth_dxz.simplify()
print('Done.')
print('Simplifying ddth_dyz...', end=' ', flush=True)
ddth_dyz = ddth_dyz.simplify()
print('Done.')

print(f'\nSIMPLIFIED DERIVATIVE RESULTS:')
print(f'{dth_dw=}\n')
print(f'{dth_dx=}\n')
print(f'{dth_dy=}\n')
print(f'{dth_dz=}\n')
print(f'{ddth_dww=}\n')
print(f'{ddth_dwx=}\n')
print(f'{ddth_dwy=}\n')
print(f'{ddth_dwz=}\n')
print(f'{ddth_dxx=}\n')
print(f'{ddth_dyy=}\n')
print(f'{ddth_dzz=}\n')
print(f'{ddth_dxy=}\n')
print(f'{ddth_dxz=}\n')
print(f'{ddth_dyz=}\n')


# Build the Hessian of theta^2.
H_ww = 2*dth_dw**2 + 2*theta*ddth_dww
H_xx = 2*dth_dx**2 + 2*theta*ddth_dxx
H_yy = 2*dth_dy**2 + 2*theta*ddth_dyy
H_zz = 2*dth_dz**2 + 2*theta*ddth_dzz

H_wx = 2*dth_dw*dth_dx + 2*theta*ddth_dwx
H_wy = 2*dth_dw*dth_dy + 2*theta*ddth_dwy
H_wz = 2*dth_dw*dth_dz + 2*theta*ddth_dwz

H_xy = 2*dth_dx*dth_dy + 2*theta*ddth_dxy
H_xz = 2*dth_dx*dth_dz + 2*theta*ddth_dxz

H_yz = 2*dth_dy*dth_dz + 2*theta*ddth_dyz

print(f'\nUNSIMPLIFIED HESSIAN RESULTS:')
print(f'{H_ww=}\n')
print(f'{H_xx=}\n')
print(f'{H_yy=}\n')
print(f'{H_zz=}\n')
print(f'{H_wx=}\n')
print(f'{H_wy=}\n')
print(f'{H_wz=}\n')
print(f'{H_xy=}\n')
print(f'{H_xz=}\n')
print(f'{H_yz=}\n')


print('Simplifying H_ww...', end=' ', flush=True)
H_ww = H_ww.simplify()
print('Done.')
print('Simplifying H_xx...', end=' ', flush=True)
H_xx = H_xx.simplify()
print('Done.')
print('Simplifying H_yy...', end=' ', flush=True)
H_yy = H_yy.simplify()
print('Done.')
print('Simplifying H_zz...', end=' ', flush=True)
H_zz = H_zz.simplify()
print('Done.')
print('Simplifying H_wx...', end=' ', flush=True)
H_wx = H_wx.simplify()
print('Done.')
print('Simplifying H_wy...', end=' ', flush=True)
H_wy = H_wy.simplify()
print('Done.')
print('Simplifying H_wz...', end=' ', flush=True)
H_wz = H_wz.simplify()
print('Done.')
print('Simplifying H_xy...', end=' ', flush=True)
H_xy = H_xy.simplify()
print('Done.')
print('Simplifying H_xz...', end=' ', flush=True)
H_xz = H_xz.simplify()
print('Done.')
print('Simplifying H_yz...', end=' ', flush=True)
H_yz = H_yz.simplify()
print('Done.')

print(f'\nSIMPLIFIED HESSIAN RESULTS:')
print(f'{H_ww=}\n')
print(f'{H_xx=}\n')
print(f'{H_yy=}\n')
print(f'{H_zz=}\n')
print(f'{H_wx=}\n')
print(f'{H_wy=}\n')
print(f'{H_wz=}\n')
print(f'{H_xy=}\n')
print(f'{H_xz=}\n')
print(f'{H_yz=}\n')

# Convert those expressions to functions, always taking current quaternion (q)
# first, and desired quaternion (r) second.
H_ww_np_func = sym.lambdify((q_np, r_np), H_ww)
H_xx_np_func = sym.lambdify((q_np, r_np), H_xx)
H_yy_np_func = sym.lambdify((q_np, r_np), H_yy)
H_zz_np_func = sym.lambdify((q_np, r_np), H_zz)
H_wx_np_func = sym.lambdify((q_np, r_np), H_wx)
H_wy_np_func = sym.lambdify((q_np, r_np), H_wy)
H_wz_np_func = sym.lambdify((q_np, r_np), H_wz)
H_xy_np_func = sym.lambdify((q_np, r_np), H_xy)
H_xz_np_func = sym.lambdify((q_np, r_np), H_xz)
H_yz_np_func = sym.lambdify((q_np, r_np), H_yz)

import inspect
print(f'\nH_ww function:')
print(inspect.getsource(H_ww_np_func))
print(f'\nH_xx function:')
print(inspect.getsource(H_xx_np_func))
print(f'\nH_yy function:')
print(inspect.getsource(H_yy_np_func))
print(f'\nH_zz function:')
print(inspect.getsource(H_zz_np_func))
print(f'\nH_wx function:')
print(inspect.getsource(H_wx_np_func))
print(f'\nH_wy function:')
print(inspect.getsource(H_wy_np_func))
print(f'\nH_wz function:')
print(inspect.getsource(H_wz_np_func))
print(f'\nH_xy function:')
print(inspect.getsource(H_xy_np_func))
print(f'\nH_xz function:')
print(inspect.getsource(H_xz_np_func))
print(f'\nH_yz function:')
print(inspect.getsource(H_yz_np_func))


def hessian_func(q, r):
    q = q.squeeze()
    r = r.squeeze()
    assert q.shape == r.shape == (4,)
    return np.array(
        [[H_ww_np_func(*q, *r), H_wx_np_func(*q, *r), H_wy_np_func(*q, *r),
            H_wz_np_func(*q, *r)],
         [H_wx_np_func(*q, *r), H_xx_np_func(*q, *r), H_xy_np_func(*q, *r),
            H_xz_np_func(*q, *r)],
         [H_wy_np_func(*q, *r), H_xy_np_func(*q, *r), H_yy_np_func(*q, *r),
            H_yz_np_func(*q, *r)],
         [H_wz_np_func(*q, *r), H_xz_np_func(*q, *r), H_yz_np_func(*q, *r),
            H_zz_np_func(*q, *r)]])

breakpoint()
