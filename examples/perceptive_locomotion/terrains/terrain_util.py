import numpy as np
from dataclasses import dataclass
from matplotlib import pyplot as plt

@dataclass
class BoxParams:
    xdim: float
    ydim: float
    zdim: float = 0.2


def zero_height_fun(t):
    return 0.0

import numpy as np


def interpolant(t):
    return t*t*t*(t*(t*6 - 15) + 10)


def generate_perlin_noise_2d(
    shape, res, tileable=(False, False), interpolant=interpolant
):
    """Generate a 2D numpy array of perlin noise.

    Args:
        shape: The shape of the generated array (tuple of two ints).
            This must be a multple of res.
        res: The number of periods of noise to generate along each
            axis (tuple of two ints). Note shape must be a multiple of
            res.
        tileable: If the noise should be tileable along each axis
            (tuple of two bools). Defaults to (False, False).
        interpolant: The interpolation function, defaults to
            t*t*t*(t*(t*6 - 15) + 10).

    Returns:
        A numpy array of shape shape with the generated noise.

    Raises:
        ValueError: If shape is not a multiple of res.
    """
    delta = (res[0] / shape[0], res[1] / shape[1])
    d = (shape[0] // res[0], shape[1] // res[1])
    grid = np.mgrid[0:res[0]:delta[0], 0:res[1]:delta[1]] \
               .transpose(1, 2, 0) % 1
    # Gradients
    angles = 2*np.pi*np.random.rand(res[0]+1, res[1]+1)
    gradients = np.dstack((np.cos(angles), np.sin(angles)))
    if tileable[0]:
        gradients[-1,:] = gradients[0,:]
    if tileable[1]:
        gradients[:,-1] = gradients[:,0]
    gradients = gradients.repeat(d[0], 0).repeat(d[1], 1)
    g00 = gradients[    :-d[0],    :-d[1]]
    g10 = gradients[d[0]:     ,    :-d[1]]
    g01 = gradients[    :-d[0],d[1]:     ]
    g11 = gradients[d[0]:     ,d[1]:     ]
    # Ramps
    n00 = np.sum(np.dstack((grid[:,:,0]  , grid[:,:,1]  )) * g00, 2)
    n10 = np.sum(np.dstack((grid[:,:,0]-1, grid[:,:,1]  )) * g10, 2)
    n01 = np.sum(np.dstack((grid[:,:,0]  , grid[:,:,1]-1)) * g01, 2)
    n11 = np.sum(np.dstack((grid[:,:,0]-1, grid[:,:,1]-1)) * g11, 2)
    # Interpolation
    t = interpolant(grid)
    n0 = n00*(1-t[:,:,0]) + t[:,:,0]*n10
    n1 = n01*(1-t[:,:,0]) + t[:,:,0]*n11
    return np.sqrt(2)*((1-t[:,:,1])*n0 + t[:,:,1]*n1)


def generate_fractal_noise_2d(
    shape, res, octaves=1, persistence=0.5,
    lacunarity=2, tileable=(False, False),
    interpolant=interpolant
):
    """Generate a 2D numpy array of fractal noise.

    Args:
        shape: The shape of the generated array (tuple of two ints).
            This must be a multiple of lacunarity**(octaves-1)*res.
        res: The number of periods of noise to generate along each
            axis (tuple of two ints). Note shape must be a multiple of
            (lacunarity**(octaves-1)*res).
        octaves: The number of octaves in the noise. Defaults to 1.
        persistence: The scaling factor between two octaves.
        lacunarity: The frequency factor between two octaves.
        tileable: If the noise should be tileable along each axis
            (tuple of two bools). Defaults to (False, False).
        interpolant: The, interpolation function, defaults to
            t*t*t*(t*(t*6 - 15) + 10).

    Returns:
        A numpy array of fractal noise and of shape shape generated by
        combining several octaves of perlin noise.

    Raises:
        ValueError: If shape is not a multiple of
            (lacunarity**(octaves-1)*res).
    """
    noise = np.zeros(shape)
    frequency = 1
    amplitude = 1
    for _ in range(octaves):
        noise += amplitude * generate_perlin_noise_2d(
            shape, (frequency*res[0], frequency*res[1]), tileable, interpolant
        )
        frequency *= lacunarity
        amplitude *= persistence
    return noise


def make_arc(r: float, n: int, theta_start: float, theta_end, offset: np.ndarray,
             bp: BoxParams, height_fun=zero_height_fun):
    if theta_end < theta_start:
        print('ERROR: Please parameterize the arc assuming theta_ end is ccw '
              'from theta_start')
        exit(-1)

    angles = np.linspace(theta_start, theta_end, n)
    for a in angles:
        x = r * np.cos(a) + offset[0]
        y = r * np.sin(a) + offset[1]
        z = height_fun(a) + offset[2]
        yaw = a - np.pi / 2
        print(f''
              f'- [[{x:.2f}, {y:.2f}, {z:.4f}], [0 ,0, 1],  [{bp.xdim:.2f}, {bp.ydim:.2f}, {bp.zdim:.2f}], [{yaw:.2f}]]')


def make_stair_curriculum(n: int, length: float):
    # Chebyshev nodes
    x_extents = (length / 2) * np.array(
        [np.cos(np.pi * float(2 * k - 1) / float(2 * n + 2)) for k in range(1, n+2)]
    )
    x_extents = np.flip(x_extents)
    x_lengths = np.diff(x_extents)
    x_knots = [0.5 * (x_extents[i] + x_extents[i-1]) for i in range(1, n+1)]

    z_knots = [0.04 * (2.1 * x + 0.011 * x ** 3) for x in x_knots]
    zmax = np.max(np.diff(z_knots))

    for x, z, xlen in zip(x_knots, z_knots, x_lengths):
        print(f'- [[{x: .9f}, 0, {z: .4f}], [0, 0, 1], [{xlen: .9f}, {length}, {zmax : .2f}], [0]]')
        print(f'- [[0, {x: .9f}, {z: .4f}], [0, 0, 1], [{xlen: .9f}, '
              f'{length}, {zmax : .2f}], [{np.pi / 2 :.4f}]]')

    plt.stairs(z_knots, x_extents)
    plt.show()


def make_block_perlin(grid_size, bins):
    assert(bins % 2 == 1) # need an odd number of bins

    res = grid_size / bins
    resint = int(res) + 2
    terrain = 2.0 * generate_fractal_noise_2d(
        (bins + 1, bins + 1), (resint, resint), 2
    )
    terrain = terrain[:-1, :-1]
    middle_idx = int(bins / 2)
    terrain -= terrain[middle_idx, middle_idx]
    diffx = np.diff(terrain, axis=0)
    diffy = np.diff(terrain, axis=1)
    zmax = np.maximum(np.max(diffx), np.max(diffy))
    for i in range(bins):
        for j in range(bins):
            xbin = grid_size * float(i - bins) / 2.0
            ybin = grid_size * float(j - bins) / 2.0
            print(f'- [[{xbin}, {ybin}, {terrain[i,j]}], [0, 0, 1], ['
                  f'{res, res, zmax}], [0]')

    x = np.linspace(-grid_size / 2, grid_size / 2, bins)
    y = np.linspace(-grid_size / 2, grid_size / 2, bins)
    X, Y = np.meshgrid(x, y)
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax.plot_surface(X, Y, terrain)
    print(f'max step: {zmax}')
    plt.show()



if __name__ == '__main__':
    make_block_perlin(25.0, 35)
    # make_stair_curriculum(81, 40)
    # def hfun(t):
    #     return 0.2 * np.sin(2 * t)
    # make_arc(2.5, 8, np.pi/2, 3*np.pi/2, np.array([-1.0, 0, 0]), BoxParams(0.75, 2.0), height_fun=hfun)
