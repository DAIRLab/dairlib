import os
import glob
import sys

def combine_obj_files(input_pattern, output_file):
    v_lines = []
    f_lines = []
    v_count = 0

    # Find all matching files
    files = sorted(glob.glob(input_pattern))
    if not files:
        print(f"No files matched pattern: {input_pattern}")
        return

    for file in files:
        with open(file, 'r') as f:
            lines = f.readlines()
            for line in lines:
                if line.startswith('v '):
                    v_lines.append(line)
                elif line.startswith('f '):
                    parts = line.strip().split()
                    new_parts = [parts[0]]
                    for part in parts[1:]:
                        idx = part.split('/')
                        idx[0] = str(int(idx[0]) + v_count)
                        new_parts.append('/'.join(idx))
                    f_lines.append(' '.join(new_parts) + '\n')
        v_count = len(v_lines)

    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    with open(output_file, 'w') as out:
        out.writelines(v_lines)
        out.writelines(f_lines)

    print(f"✅ Combined OBJ written to {output_file}")

if __name__ == "__main__":
    if len(sys.argv) not in [2, 3]:
        print("Usage: python combine_script.py <base_name> [output_dir]")
        sys.exit(1)

    base_name = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) == 3 else "examples/sampling_c3/urdf"
    input_pattern = f"examples/sampling_c3/urdf/{base_name}_*.obj"
    output_file = os.path.join(output_dir, f"{base_name}.obj")

    print(f"🔍 Searching for files matching: {input_pattern}")
    combine_obj_files(input_pattern, output_file)
