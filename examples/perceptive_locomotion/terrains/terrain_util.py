import numpy as np
from dataclasses import dataclass

@dataclass
class BoxParams:
    xdim: float
    ydim: float
    zdim: float = 0.2


def zero_height_fun(t):
    return 0.0


def make_arc(r: float, n: int, theta_start: float, theta_end,
             bp: BoxParams, height_fun=zero_height_fun):
    if theta_end < theta_start:
        print('ERROR: Please parameterize the arc assuming theta_ end is ccw '
              'from theta_start')
        exit(-1)

    angles = np.linspace(theta_start, theta_end, n)
    for a in angles:
        x = r * np.cos(a)
        y = r * np.sin(a)
        z = height_fun(a)
        yaw = a - np.pi / 2
        print(f''
              f'- [[{x:.2f}, {y:.2f}, {z:.4f}], [0 ,0, 1],  [{bp.xdim:.2f}, {bp.ydim:.2f}, {bp.zdim:.2f}], [{yaw:.2f}]]')


if __name__ == '__main__':
    def hfun(t):
        return 0.2 * np.sin(2 * t)
    make_arc(2.0, 8, np.pi/2, 3*np.pi/2, BoxParams(0.5, 2.0), height_fun=hfun)
