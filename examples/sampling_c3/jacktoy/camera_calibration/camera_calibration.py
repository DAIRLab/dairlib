"""Script to calibrate the extrinsics (and detect the intrinsics reported by the
RealSense) of a RealSense D455 camera based on a pre-defined Aruco tag board.
Requires virtual environment at:

~/workspace/dairlib/cam_venv/

E.g., run via:

cd ~/workspace/dairlib
source cam_venv/bin/activate
python examples/sampling_c3/jacktoy/camera_calibration/camera_calibration.py
"""

import cv2
from cv2 import aruco

from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
import os
import os.path as op
import pyrealsense2 as rs


COMPUTE_EXTRINSICS = True
SHOW_INTRINSICS = False
RECORD_RGBD_IMAGE = True


BOARD_T_WORLD = np.array([[0, 0, 1, 0.0082],
                          [1, 0, 0, -0.09355],
                          [0, 1, 0, -0.18656],
                          [0, 0, 0, 1]])
WORLD_T_POINT = np.array([[1, 0, 0, 0.07855],
                          [0, 1, 0, 0],
                          [0, 0, 1, -0.0282],
                          [0, 0, 0, 1]])


timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

def get_filepath(filename):
    dir_path = op.dirname(os.path.realpath(__file__))
    cam_cal_dir = op.join(dir_path, 'cam_cals', timestamp)
    os.makedirs(cam_cal_dir, exist_ok=True)
    print(f'Making file: {op.join(cam_cal_dir, filename)}')
    return op.join(cam_cal_dir, filename)


if COMPUTE_EXTRINSICS:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 800, rs.format.bgr8, 30)
    pipeline.start(config)

    # Get the image a few times to allow auto-exposure to balance.
    for _ in range(150):
        frame = pipeline.wait_for_frames()
        color_frame = frame.get_color_frame()

    np_color_image_bgr = np.asanyarray(color_frame.get_data())
    np_color_image = np_color_image_bgr[:, :, ::-1]
    plt.imshow(np_color_image)
    plt.savefig(get_filepath('color_image.png'), dpi=300)
    plt.show()

    # Get the camera calibration parameters from pyrealsense.
    intrinsics = pipeline.get_active_profile().get_stream(rs.stream.color
        ).as_video_stream_profile().get_intrinsics()
    camera_matrix = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                              [0, intrinsics.fy, intrinsics.ppy],
                              [0, 0, 1]])
    distortion_coefficients = np.array(intrinsics.coeffs)

    # Aruco tag definitions.
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
    board = aruco.CharucoBoard((12,9), 0.03, 0.023, aruco_dict)
    charuco_detector_params = aruco.CharucoParameters()
    charuco_detector_params.cameraMatrix = camera_matrix
    charuco_detector_params.distCoeffs = distortion_coefficients
    charuco_detector = aruco.CharucoDetector(
        board, charucoParams=charuco_detector_params)

    # Get board pose.
    charuco_corners, charuco_ids, marker_corners, marker_ids = \
        charuco_detector.detectBoard(np_color_image_bgr)
    if len(charuco_corners) == 0:
        raise Exception('No charuco corners detected!')
    obj_points, img_points = board.matchImagePoints(
        charuco_corners, charuco_ids)

    # Get the pose of the camera.
    ret, rvec, tvec = cv2.solvePnP(
        obj_points, img_points, camera_matrix, distortion_coefficients)
    if not ret:
        raise Exception('Could not solve PnP!')
    
    # Convert transformation matrix.
    C_R_B, _ = cv2.Rodrigues(rvec)
    C_T_B = np.concatenate((C_R_B, tvec), axis=1)
    C_T_B = np.concatenate((C_T_B, np.array([[0, 0, 0, 1]])), axis=0)

    # Define the board to world transformation.
    B_T_W = BOARD_T_WORLD
    C_T_W = C_T_B @ B_T_W

    # Add a point against the Franka platform on the table surface.
    W_T_P = WORLD_T_POINT
    C_T_P = C_T_W @ W_T_P

    # Debugging plot.
    image_debug_viz = cv2.drawFrameAxes(
        np_color_image_bgr,
        camera_matrix,
        distortion_coefficients,
        C_T_B[:3, :3],
        C_T_B[:3, 3:],
        0.1
    )
    image_debug_viz = cv2.drawFrameAxes(
        image_debug_viz,
        camera_matrix,
        distortion_coefficients,
        C_T_W[:3, :3],
        C_T_W[:3, 3:],
        0.08
    )
    image_debug_viz = cv2.drawFrameAxes(
        image_debug_viz,
        camera_matrix,
        distortion_coefficients,
        C_T_P[:3, :3],
        C_T_P[:3, 3:],
        0.08
    )
    # Show the debug image in window.
    plt.imshow(image_debug_viz[:, :, ::-1])
    plt.savefig(get_filepath('debug_image.png'), dpi=300)
    plt.show()

    # Save the camera calibration parameters.
    np.save(get_filepath('camera_matrix.npy'), camera_matrix)
    np.save(get_filepath('distortion_coefficients.npy'),
                         distortion_coefficients)
    np.save(get_filepath('color_tf_world.npy'), C_T_W)

if RECORD_RGBD_IMAGE:
    assert COMPUTE_EXTRINSICS, 'Need to compute extrinsics first!'
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    # aligned to color
    align_to = rs.stream.color
    align = rs.align(align_to)
    # start streaming
    pipeline.start(config)

    # Get the camera calibration parameters from pyrealsense
    intrinsics = pipeline.get_active_profile().get_stream(rs.stream.color
        ).as_video_stream_profile().get_intrinsics()
    fx = intrinsics.fx
    fy = intrinsics.fy
    cx = intrinsics.ppx
    cy = intrinsics.ppy

    # get camera intrinsics
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()

    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    np.save(get_filepath('color_image_for_depth_inspection.npy'), color_image)
    np.save(get_filepath('depth_image_for_depth_inspection.npy'), depth_image)

    height, width = depth_image.shape

    # Generate pixel grid.
    x = np.arange(0, width)
    y = np.arange(0, height)
    xv, yv = np.meshgrid(x, y)

    # Calculate corresponding 3D coordinates.
    X = (xv - cx) * depth_image / fx
    Y = (yv - cy) * depth_image / fy
    Z = depth_image

    # Stack the coordinates and reshape.
    point_cloud = np.stack((X, Y, Z), axis=-1)
    point_cloud = point_cloud.reshape((-1, 3))

    # Filter out the non-returns and convert millimeters to meters.
    point_cloud = point_cloud[np.any(point_cloud != 0, axis=1)] / 1000.0

    # Convert to represented in world frame.
    points_camera_h = np.hstack((point_cloud,
                                 np.ones((point_cloud.shape[0], 1))))
    points_world_h = (np.linalg.inv(C_T_W) @ points_camera_h.T).T
    points_world = points_world_h[:, :3]

    # Cut out the points that are too far away.
    points_world = points_world[points_world[:, 0] > -0.4]
    points_world = points_world[points_world[:, 0] < 1]
    points_world = points_world[points_world[:, 1] > -0.5]
    points_world = points_world[points_world[:, 1] < 0.5]
    points_world = points_world[points_world[:, 2] > -0.1]
    points_world = points_world[points_world[:, 2] < 1]

    # Create some points whose world locations we think we know.
    table_xs = np.linspace(0.07855, 0.4, 50)
    table_ys = np.linspace(-0.3, 0.3, 50)
    x_grid, y_grid = np.meshgrid(table_xs, table_ys)
    table_zs = -0.0282 * np.ones_like(x_grid.ravel())
    points_table = np.column_stack((x_grid.ravel(), y_grid.ravel(), table_zs))
    
    board_xs = np.linspace(0.07855, 0.07855+0.3, 50)
    board_zs = np.linspace(-0.0282, -0.0282+0.4, 50)
    x_grid, z_grid = np.meshgrid(board_xs, board_zs)
    board_ys = 0.18656 * np.ones_like(x_grid.ravel())
    points_board = np.column_stack((x_grid.ravel(), board_ys, z_grid.ravel()))

    # Make a 3d plot.
    plt.figure()
    ax = plt.axes(projection='3d')
    ax.scatter(points_world[:, 0], points_world[:, 1], points_world[:, 2],
               c=points_world[:, 2], cmap='viridis', s=0.1, label='Depth')
    ax.scatter(points_table[:, 0], points_table[:, 1], points_table[:, 2],
               c='r', s=1, label='Table')
    ax.scatter(points_board[:, 0], points_board[:, 1], points_board[:, 2],
               c='g', s=1, label='Board')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.savefig(get_filepath('point_cloud.png'), dpi=300)
    plt.show()
    plt.close()

    # Project known world points onto a more cropped image.
    points_world_b = points_world[points_world[:, 0] > 0.05]
    points_world_b = points_world_b[points_world_b[:, 0] < 0.4]
    points_world_b = points_world_b[points_world_b[:, 1] > 0.165]
    points_world_b = points_world_b[points_world_b[:, 1] < 0.195]
    points_world_b = points_world_b[points_world_b[:, 2] > -0.1]
    points_world_b = points_world_b[points_world_b[:, 2] < 0.4]

    plt.figure()
    ax = plt.axes(projection='3d')
    ax.scatter(points_world_b[:, 0], points_world_b[:, 1], points_world_b[:, 2],
               c=points_world_b[:, 2], cmap='viridis', s=0.1, label='Depth')
    ax.scatter(points_board[:, 0], points_board[:, 1], points_board[:, 2],
               c='g', s=1, label='Board')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    ax.view_init(elev=0, azim=0, roll=90)
    plt.savefig(get_filepath('point_cloud_board_cropped.png'), dpi=300)
    plt.show()
    plt.close()

    # Make a 3d plot again but with more points cropped out.
    points_world_t = points_world[points_world[:, 0] < 0.4]
    points_world_t = points_world_t[points_world_t[:, 0] > 0.08]
    points_world_t = points_world_t[points_world_t[:, 1] > -0.4]
    points_world_t = points_world_t[points_world_t[:, 1] < 0.4]
    points_world_t = points_world_t[points_world_t[:, 2] > -0.04]
    points_world_t = points_world_t[points_world_t[:, 2] < -0.02]

    plt.figure()
    ax = plt.axes(projection='3d')
    ax.scatter(points_world_t[:, 0], points_world_t[:, 1], points_world_t[:, 2],
               c=points_world_t[:, 2], cmap='viridis', s=0.1, label='Depth')
    ax.scatter(points_table[:, 0], points_table[:, 1], points_table[:, 2],
               c='r', s=1, label='Table')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    ax.view_init(elev=0, azim=-90)
    plt.savefig(get_filepath('point_cloud_table_cropped.png'), dpi=300)
    plt.show()
    plt.close()


if SHOW_INTRINSICS:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    # aligned to color
    align_to = rs.stream.color
    align = rs.align(align_to)
    # start streaming
    pipeline.start(config)

    # get camera intrinsics
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
    color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
    unaligned_depth_frame = frames.get_depth_frame()
    unaligned_depth_intrin = unaligned_depth_frame.profile.as_video_stream_profile().intrinsics
    # save camera intrinsics
    # convert to dictionary
    depth_intrin_dict = {
        "width": depth_intrin.width,
        "height": depth_intrin.height,
        "ppx": depth_intrin.ppx,
        "ppy": depth_intrin.ppy,
        "fx": depth_intrin.fx,
        "fy": depth_intrin.fy,
        "model": depth_intrin.model,
        "coeffs": depth_intrin.coeffs
    }
    color_intrin_dict = {
        "width": color_intrin.width,
        "height": color_intrin.height,
        "ppx": color_intrin.ppx,
        "ppy": color_intrin.ppy,
        "fx": color_intrin.fx,
        "fy": color_intrin.fy,
        "model": color_intrin.model,
        "coeffs": color_intrin.coeffs
    }
    unaligned_depth_intrin_dict = {
        "width": unaligned_depth_intrin.width,
        "height": unaligned_depth_intrin.height,
        "ppx": unaligned_depth_intrin.ppx,
        "ppy": unaligned_depth_intrin.ppy,
        "fx": unaligned_depth_intrin.fx,
        "fy": unaligned_depth_intrin.fy,
        "model": unaligned_depth_intrin.model,
        "coeffs": unaligned_depth_intrin.coeffs
    }

    pipeline.stop()

    print(f'\ndepth_intrin_dict:')
    for key, val in depth_intrin_dict.items():
        print(f'{key}: {val}')

    print(f'\ncolor_intrin_dict:')
    for key, val in color_intrin_dict.items():
        print(f'{key}: {val}')

    print(f'\nunaligned_depth_intrin_dict:')
    for key, val in unaligned_depth_intrin_dict.items():
        print(f'{key}: {val}')



breakpoint()
