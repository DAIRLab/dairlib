#!/usr/bin/env python3
# pip install meshio numpy

import numpy as np
import meshio
import sys

def signed_tet_volume(a, b, c, d):
    return np.dot(np.cross(b - a, c - a), d - a) / 6.0

def fix_tetra_mesh(input_vtk: str, output_vtk: str):
    mesh = meshio.read(input_vtk)
    if "tetra" not in mesh.cells_dict:
        raise RuntimeError("No tetrahedral cells found in input VTK.")

    points = mesh.points
    tets = mesh.cells_dict["tetra"].copy()
    num_tets = tets.shape[0]

    volumes = np.array([
        signed_tet_volume(points[i0], points[i1], points[i2], points[i3])
        for i0, i1, i2, i3 in tets
    ])
    bad_indices = np.where(volumes <= 0)[0]
    print(f"Found {len(bad_indices)} invalid tetrahedra out of {num_tets}.")

    if bad_indices.size > 0:
        print("Correcting orientations for indices:", bad_indices.tolist())
        sub = tets[bad_indices, :].copy()
        sub[:, [0,1]] = sub[:, [1,0]]
        tets[bad_indices, :] = sub

        volumes_fixed = np.array([
            signed_tet_volume(points[i0], points[i1], points[i2], points[i3])
            for i0, i1, i2, i3 in tets
        ])
        failed = np.where(volumes_fixed <= 0)[0]
        if failed.size > 0:
            raise RuntimeError(f"Unable to fix tets at indices: {failed.tolist()}")
        print("✅ All tetrahedra now have positive volume.")

    meshio.write_points_cells(
        output_vtk,
        points,
        {"tetra": tets}
    )
    print(f"✅ Cleaned mesh written to: {output_vtk}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        description="Fix non-positive-volume tetrahedra in a VTK mesh."
    )
    parser.add_argument("input_vtk", help="Input tetrahedral VTK file")
    parser.add_argument("output_vtk", help="Output fixed tetrahedral VTK file")
    args = parser.parse_args()
    fix_tetra_mesh(args.input_vtk, args.output_vtk)
