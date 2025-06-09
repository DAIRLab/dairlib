import sys
import os
import glob

def get_min_z_from_obj(obj_path):
    min_z = None
    with open(obj_path, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.strip().split()
                if len(parts) == 4:
                    z = float(parts[3])
                    if min_z is None or z < min_z:
                        min_z = z
    return min_z

def main(patterns):
    obj_files = []
    for pat in patterns:
        expanded = glob.glob(pat)
        if not expanded:
            print(f"No files found for pattern: {pat}")
        obj_files.extend(expanded)

    if not obj_files:
        print("No OBJ files matched. Exiting.")
        sys.exit(1)

    global_min_z = None
    for obj_path in obj_files:
        min_z = get_min_z_from_obj(obj_path)
        print(f"File: {os.path.basename(obj_path)} | Min Z: {min_z:.8f}" if min_z is not None else f"File: {os.path.basename(obj_path)} | No vertices found")
        if min_z is not None and (global_min_z is None or min_z < global_min_z):
            global_min_z = min_z

    if global_min_z is None:
        print("No valid vertices found in the input OBJ files.")
        sys.exit(1)

    print(f"\nGlobal minimum Z among all files: {global_min_z:.8f}")

    # Print the recommended plane for the object to lay flat
    print(f"\nRecommended plane for objects to lay flat: z = {global_min_z:.8f}")
    print(f"Plane equation: 0 0 1 -{global_min_z:.8f}  (i.e., z - {global_min_z:.8f} = 0)")

    # Optional: Output an SDF snippet
    print("\nSDF plane element snippet:")
    print(f"""<plane>
  <normal>0 0 1</normal>
  <offset>{-global_min_z:.8f}</offset>
</plane>""")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python find_min_z_plane.py path_*")
        sys.exit(1)
    main(sys.argv[1:])