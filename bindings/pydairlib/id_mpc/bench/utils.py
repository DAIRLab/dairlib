import numpy as np


# Utils for generating random stepping stones
def get_block_string(x, y, z, n, lx, ly, lz, yaw):
    return f'  - [[{x}, {y}, {z}], [{n[0]}, {n[1]}, {n[2]}], ' \
           f'[{lx}, {ly}, {lz}], [{yaw}]]'


def get_blocks_string(x, y, z, normal, lx, ly, lz, yaw):
    out = 'stones:\n'
    for i in range(len(x)):
        n = normal[i]
        out = out + get_block_string(
            x[i], y[i], z[i], n, lx[i], ly[i], lz[i], yaw[i]
        ) + '\n'
    return out


def random_stepping_stones(seed, min_sidelength, savefile=None):
    rng = np.random.default_rng(seed)
    rows = 5
    cols = 3
    base_len = 1.0 + 0.21 - min_sidelength
    y_variation = 0.05
    x_variation = 0.05
    z_variation = 0.075
    x_dist = min_sidelength + 0.21
    y_dist = min_sidelength + 0.21

    # initialize stepping stone geometry with the start, end, and floor blocks
    xs = [0.0, base_len * 2 + (rows + 1) * x_dist, 5.0]
    ys = [0.0, 0.0, 0.0]
    zs = [0.0, 0.0, -2.0]
    normals = [np.array([0, 0, 1]), np.array([0, 0, 1]), np.array([0, 0, 1])]
    lxs = [2.0, 2.0, 20.0]
    lys = [2.5, 2.5, 20.0]
    lzs = [0.2, 0.2, 0.2]
    yaws = [0.0, 0.0, 0.0]

    for r in range(rows):
        for c in range(cols):
            x = base_len + x_dist * (r + 1) + rng.uniform(-x_variation, x_variation)
            y = rng.uniform(-y_variation, y_variation) + y_dist * (c - 0.5 * (cols - 1))
            z = rng.uniform(-z_variation, z_variation)
            xs.append(x)
            ys.append(y)
            zs.append(z)
            normals.append(np.array([0, 0, 1]))
            lxs.append(rng.uniform(min_sidelength, min_sidelength + 0.05))
            lys.append(rng.uniform(min_sidelength, min_sidelength + 0.05))
            lzs.append(0.3)
            yaws.append(0)

    block_str = get_blocks_string(xs, ys, zs, normals, lxs, lys, lzs, yaws)
    if savefile is None:
        print(block_str)
    else:
        with open(savefile, 'w') as fp:
            fp.write(block_str)