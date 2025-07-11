#!/usr/bin/env python3
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

def get_max_z_from_obj(obj_path):
    max_z = None
    with open(obj_path, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.strip().split()
                if len(parts) == 4:
                    z = float(parts[3])
                    if max_z is None or z > max_z:
                        max_z = z
    return max_z

def main(patterns, mode):
    obj_files = []
    for pat in patterns:
        expanded = glob.glob(pat)
        if not expanded:
            print(f"No files found for pattern: {pat}")
        obj_files.extend(expanded)

    if not obj_files:
        print("No OBJ files matched. Exiting.")
        sys.exit(1)

    if mode == 'min':
        global_z = None
        for path in obj_files:
            val = get_min_z_from_obj(path)
            print(f"File: {os.path.basename(path)} | Min Z: {val:.8f}" if val is not None else f"File: {os.path.basename(path)} | No vertices found")
            if val is not None and (global_z is None or val < global_z):
                global_z = val
        if global_z is None:
            sys.exit(1)
        print(f"\nGlobal minimum Z among all files: {global_z:.8f}")
        print(f"\nRecommended plane for objects to lay flat: z = {global_z:.8f}")
        print(f"Plane equation: 0 0 1 -{global_z:.8f} = 0")
        print("\nSDF plane element snippet:")
        print(f"<plane>\n  <normal>0 0 1</normal>\n  <offset>{-global_z:.8f}</offset>\n</plane>")

    elif mode == 'max':
        global_z = None
        for path in obj_files:
            val = get_max_z_from_obj(path)
            print(f"File: {os.path.basename(path)} | Max Z: {val:.8f}" if val is not None else f"File: {os.path.basename(path)} | No vertices found")
            if val is not None and (global_z is None or val > global_z):
                global_z = val
        if global_z is None:
            sys.exit(1)
        print(f"\nGlobal maximum Z among all files: {global_z:.8f}")
        print(f"\nRecommended plane to cap objects from above: z = {global_z:.8f}")
        print(f"Plane equation: 0 0 -1 {global_z:.8f} = 0")
        print("\nSDF plane element snippet:")
        print(f"<plane>\n  <normal>0 0 -1</normal>\n  <offset>{global_z:.8f}</offset>\n</plane>")

    else:
        print("Unknown mode. Use 'min' or 'max'")
        sys.exit(1)

if __name__ == "__main__":
    prog = os.path.basename(sys.argv[0])
    if 'min' in prog:
        mode = 'min'
    elif 'max' in prog:
        mode = 'max'
    else:
        print("Please invoke as 'obj_min_z_plane.py' or 'obj_max_z_plane.py'")
        sys.exit(1)
    main(sys.argv[1:], mode)
