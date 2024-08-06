import numpy as np
import matplotlib.pyplot as plt

from typing import List


def whittling_cost(a: np.ndarray, v: np.ndarray, verts: List[np.ndarray]):
    b = a.dot(v)
    cost = 0
    for vert in verts:
        cost += np.maximum(0, a.dot(vert) - b) ** 2

    return cost


def grad_whittling_cost(a: np.ndarray, v: np.ndarray, verts: List[np.ndarray]):
    grad = np.zeros((2,))
    b = a.dot(v)
    for vert in verts:
        if a.dot(vert) > b:
            c = v - vert
            grad += 2 * np.linalg.outer(c, c) @ a - 2 * c
    return grad


def angle(va, vb):
    scale = np.linalg.norm(va) * np.linalg.norm(vb)
    y = va[0] * vb[1] - va[1] * vb[0] / scale
    x = va.dot(vb) / scale
    return y * np.sign(x)


def make_cost_grid(v, verts):
    nx, ny = (100, 100)
    x = np.linspace(-1.5, 1.5, nx)
    y = np.linspace(-1.5, 1.5, ny)
    z = np.zeros((nx, ny))
    x, y = np.meshgrid(x, y, indexing='ij')

    for i in range(nx):
        for j in range(ny):
            z[i, j] = whittling_cost(
                np.array([x[i, j], y[i, j]]),
                v, verts)
    return x, y, z


def test_cost():
    t = np.linspace(0, 2 * np.pi, 100)
    test_directions = [np.array([np.cos(ti), np.sin(ti)]) for ti in t]
    test_vertices = [np.array([3 * np.cos(ti), np.sin(ti)]) for ti in np.linspace(0, 2 * np.pi, 100)]

    v = np.array([2.9, 0.01])
    costs = [whittling_cost(a, v, test_vertices) for a in test_directions]
    grads = [grad_whittling_cost(a, v, test_vertices) for a in test_directions]

    angles = [angle(a, g) for a, g in zip(test_directions, grads)]

    fig = plt.figure()
    ax1 = fig.add_subplot(121, projection='3d')
    x, y, z = make_cost_grid(v, test_vertices)
    ax1.plot_surface(x, y, z, alpha=0.5, cmap='jet')
    ax1.plot(
        [td[0] for td in test_directions],
        [td[1] for td in test_directions],
        costs
    )
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.plot(
        [td[0] for td in test_directions],
        [td[1] for td in test_directions],
        angles
    )


    plt.show()

if __name__ == '__main__':
    test_cost()