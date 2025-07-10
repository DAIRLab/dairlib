import trimesh

#fast_simplification
#pyglet

# Load your mesh
mesh = trimesh.load("examples/sampling_c3/urdf/repaired.obj")

# Simplify: target face count
simplified = mesh.simplify_quadric_decimation(0.8)

# Save or visualize
simplified.export("examples/sampling_c3/urdf/repaired_coarse.obj")