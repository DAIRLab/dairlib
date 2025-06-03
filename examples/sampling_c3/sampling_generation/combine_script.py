import os
import glob

def combine_obj_files(input_pattern, output_file):
    v_lines = []
    f_lines = []
    v_count = 0

    # Find all matching files
    files = sorted(glob.glob(input_pattern))
    for file in files:
        with open(file, 'r') as f:
            lines = f.readlines()
            for line in lines:
                if line.startswith('v'):
                    v_lines.append(line)
                elif line.startswith('f'):
                    # Adjust face indices by current v_count
                    parts = line.strip().split()
                    new_parts = [parts[0]]
                    for part in parts[1:]:
                        idx = part.split('/')
                        idx[0] = str(int(idx[0]) + v_count)
                        new_parts.append('/'.join(idx))
                    f_lines.append(' '.join(new_parts) + '\n')
        # Update v_count for next file
        v_count = len(v_lines)

    # Write combined file
    with open(output_file, 'w') as out:
        out.writelines(v_lines)
        out.writelines(f_lines)

if __name__ == "__main__":
    # Adjust the path as needed
    
    input_pattern = "examples/sampling_c3/urdf/push_t_white_convex_*.obj"
    print("Files found:", glob.glob(input_pattern))
    output_file = "examples/sampling_c3/urdf/push_t_white_convex.obj"
    combine_obj_files(input_pattern, output_file)
    print(f"Combined OBJ written to {output_file}")

