import numpy as np
import trimesh
from shapely.geometry import Point, Polygon

def load_mesh(mesh_path):
    mesh = trimesh.load(mesh_path, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise RuntimeError("Expected a single mesh.")
    return mesh

def sample_mesh_outside_slice(
    mesh, R, t,
    buffer_distance, z_height,
    min_sep, max_samples=300, max_tries=5000
):
    verts, faces = mesh.vertices, mesh.faces
    tris = verts[faces]
    normals = mesh.face_normals
    areas = mesh.area_faces
    cum_areas = np.cumsum(areas)

    # Build the 2D slice polygon at z_height
    section = mesh.section(plane_origin=[0, 0, z_height], plane_normal=[0, 0, 1])
    if section is None:
        raise RuntimeError(f"No cross-section at z = {z_height}")
    slice_2D, to_3D = section.to_2D()
    poly = slice_2D.polygons_full[0]
    samples = []
    tries = 0

    while len(samples) < max_samples and tries < max_tries:
        tries += 1

        # Sample a triangle by area
        r = np.random.rand() * cum_areas[-1]
        idx = np.searchsorted(cum_areas, r)
        tri = (tris[idx] @ R.T) + t
        n = normals[idx] @ R.T

        # Corner-biased barycentric sampling
        u, v = np.random.rand(), np.random.rand()
        α = 1.0
        a, b = u**α, v**α
        if a + b > 1:
            a, b = 1-a, 1-b
        pt = (1 - a - b)*tri[0] + a*tri[1] + b*tri[2]

        # Random yaw on the normal
        # yaw = np.random.uniform(-np.pi/2, np.pi/2)
        # c, s = np.cos(yaw), np.sin(yaw)
        # Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        # dir_vec = Rz.dot(n)

        # Project and clamp z
        proj = pt + buffer_distance * n
        # print(pt, proj)
        proj[2] = -z_height

        if poly.contains(Point(proj[0], proj[1])):
            continue

        from trimesh.proximity import ProximityQuery

        pq = ProximityQuery(mesh)
        distances = pq.on_surface([proj])[1]
        if distances[0] < min_sep:
            continue
                
        samples.append(proj[:2])  # store only (x, y)

    return np.array(samples), poly, to_3D

if __name__ == "__main__":
    mesh = load_mesh("examples/sampling_c3/urdf/push_t_white.obj")
    R = np.eye(3)
    t = np.zeros(3)

    points2d, poly, to_3D = sample_mesh_outside_slice(
        mesh, R, t=[0, 0, 0],
        buffer_distance=0.06,
        z_height=0,
        min_sep=0.03,
        max_samples=200
    )

    # Plot 2D results
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6, 6))
    coords2d = np.array(poly.exterior.coords)
    homo = np.hstack([coords2d, np.zeros((len(coords2d),1)), np.ones((len(coords2d),1))])
    world = homo.dot(to_3D.T)
    px, py = world[:,0], world[:,1]
    ax.plot(px, py, '-', c='black', linewidth=1, label='Slice Boundary')

    if len(points2d):
        ax.scatter(points2d[:,0], points2d[:,1], s=15, c='blue', alpha=0.2, label='Samples Outside Slice')
    # px, py = poly.exterior.xy
    # ax.plot(px, py, '-', c='black', linewidth=1, label='Slice Boundary')
    ax.set_aspect('equal', 'box')
    ax.set_title("Accepted Samples Outside the Slice")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.legend()
    plt.show()
