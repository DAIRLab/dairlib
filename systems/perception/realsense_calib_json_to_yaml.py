import sys
import json
import yaml


def extract_depth_camera_infos_from_dict(dict):
    i = 0
    camera_infos = {}
    while f'rectified.{i}.fx' in dict:
        camera_infos[i] = {
            'width': int(dict[f'rectified.{i}.width']),
            'height': int(dict[f'rectified.{i}.height']),
            'focal_x': float(dict[f'rectified.{i}.fx']),
            'focal_y': float(dict[f'rectified.{i}.fy']),
            'center_x': float(dict[f'rectified.{i}.ppx']),
            'center_y': float(dict[f'rectified.{i}.ppy']),
        }
        i += 1
    return camera_infos


def process_json_main(json_path, save_path):
    with open(json_path, 'r') as jp:
        dict = json.load(jp)
        camera_infos = extract_depth_camera_infos_from_dict(dict)
    with open(save_path, 'w') as yp:
        yaml.dump(camera_infos, yp)


def main():
    json_file = sys.argv[1]
    save_file = sys.argv[2]
    process_json_main(json_file, save_file)


if __name__ == '__main__':
    main()
