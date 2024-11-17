import sys
import lcm

import numpy as np

from drake import lcmt_point_cloud, lcmt_point_cloud_field

from pydrake.perception import (
    BaseField,
    Fields,
    PointCloud,
)
from pydrake.solvers import MathematicalProgram, Solve
from pydrake.symbolic import acos, Expression
from pydrake.math import RotationMatrix, RigidTransform

from pydairlib.analysis.process_lcm_log import get_log_data
from pydairlib.analysis.mbp_plotting_utils import process_state_channel
from pydairlib.analysis.cassie_plotting_utils import (
    cassie_default_channels,
    make_plant_and_context,
    get_toe_frames_and_points
)


from typing import List


def binary_search_closest(arr: List[float], target: float) -> int:
    left, right = 0, len(arr) - 1

    # Handle edge cases
    if target <= arr[0]:
        return 0
    if target >= arr[-1]:
        return len(arr) - 1

    while left <= right:
        mid = (left + right) // 2

        # Perfect match
        if arr[mid] == target:
            return mid

        if arr[mid] < target:
            if mid + 1 < len(arr) and arr[mid + 1] > target:
                # Choose the closer timestamp between mid and mid+1
                if abs(target - arr[mid]) < abs(arr[mid + 1] - target):
                    return mid
                return mid + 1
            left = mid + 1
        else:
            if mid - 1 >= 0 and arr[mid - 1] < target:
                # Choose the closer timestamp between mid-1 and mid
                if abs(target - arr[mid - 1]) < abs(arr[mid] - target):
                    return mid - 1
                return mid
            right = mid - 1


class PointCloudSerializer:
    """Displays lcmt_point_cloud into MeshCat."""

    _POINT_CLOUD_FIELDS = (
        # (name, byte_offset, datatype, count)
        ("x", 0, lcmt_point_cloud_field.FLOAT32, 1),
        ("y", 4, lcmt_point_cloud_field.FLOAT32, 1),
        ("z", 8, lcmt_point_cloud_field.FLOAT32, 1),
        ("rgb", 12, lcmt_point_cloud_field.UINT32, 1),
        ("normal_x", 16, lcmt_point_cloud_field.FLOAT32, 1),
        ("normal_y", 20, lcmt_point_cloud_field.FLOAT32, 1),
        ("normal_z", 24, lcmt_point_cloud_field.FLOAT32, 1),
    )
    """The supported fields and their data types of a point cloud. An XYZ,
    XYZRGB, and XYZRGBNormal cloud will have the first three, first four, and
    all the fields in this particular order.
    """

    def _validate_and_get_fields(self, message) -> Fields:
        """Checks the point cloud LCM message and returns the corresponding
        `Fields` for the PointCloud object. Either XYZ, XYZRGB, or XYZRGBNormal
        cloud with the exact data type is supported.
        """
        if message.flags != lcmt_point_cloud.IS_STRICTLY_FINITE:
            return None

        if message.num_fields not in (3, 4, 7):
            return None

        for i in range(message.num_fields):
            (name, byte_offset, datatype, count) = self._POINT_CLOUD_FIELDS[i]
            if (
                    message.fields[i].name != name
                    or message.fields[i].byte_offset != byte_offset
                    or message.fields[i].datatype != datatype
                    or message.fields[i].count != count
            ):
                return None

        if message.num_fields == 3:
            return Fields(BaseField.kXYZs)
        elif message.num_fields == 4:
            return Fields(BaseField.kXYZs | BaseField.kRGBs)
        else:
            return Fields(
                BaseField.kXYZs | BaseField.kRGBs | BaseField.kNormals
            )

    def message_to_point_cloud(self, message) -> PointCloud:
        """Handler for lcmt_point_cloud.

        Validates and converts the lcmt_point_cloud message to a PointCloud
        """
        cloud_fields = self._validate_and_get_fields(message)
        assert cloud_fields is not None

        # Transform the raw data into an N x num_fields array.
        raw_data = np.frombuffer(message.data, dtype=np.float32).reshape(
            -1, message.num_fields
        )
        num_points = raw_data.shape[0]

        cloud = PointCloud(num_points, cloud_fields)
        xyzs = raw_data[:, 0:3]
        cloud.mutable_xyzs()[:] = xyzs.transpose()
        if message.num_fields > 3:
            rgbs_with_padding = (
                raw_data[:, 3].astype(np.float32).view(np.uint8).reshape(-1, 4)
            )
            rgbs = rgbs_with_padding[:, 0:3]
            cloud.mutable_rgbs()[:] = rgbs.transpose()
        if message.num_fields > 4:
            normals = raw_data[:, 4:]
            cloud.mutable_normals()[:] = normals.transpose()

        return cloud


def process_lcm_data(data, state_channel, point_cloud_channel):
    plant, _ = make_plant_and_context()
    robot_output = process_state_channel(data[state_channel], plant)
    point_cloud_converter = PointCloudSerializer()
    clouds = {
        't': [],
        'cloud': {}
    }
    for msg in data[point_cloud_channel]:
        clouds['t'].append(1e-6 * msg.utime)
        clouds['cloud'][clouds['t'][-1]] = point_cloud_converter.message_to_point_cloud(msg)

    # assemble data needed for calibration optimization
    data = clouds
    data['pelvis_pose'] = {}
    data['foot_height'] = {}

    plant, context = make_plant_and_context()
    pelvis = plant.GetBodyByName('pelvis')
    frames, points = get_toe_frames_and_points(plant)
    for t in clouds['t']:
        idx_state = binary_search_closest(robot_output['t_x'], t)
        plant.SetPositions(context, robot_output['q'][idx_state])
        data['pelvis_pose'][t] = pelvis.EvalPoseInWorld(context)
        data['foot_height'][t] = 0.5 * (
                frames['right'].EvalPoseInWorld(context).multiply(points['mid']) +
                frames['right'].EvalPoseInWorld(context).multiply(points['mid'])
        ).ravel()[2]

    return data


def solve_calibration(data):
    prog = MathematicalProgram()
    R = prog.NewContinuousVariables(3, 3, "R")
    p = prog.NewContinuousVariables(3, 1, "p")
    X_PC = np.hstack([R, p])

    orthonormal_constraint = (R.T @ R).reshape((-1, 1)) - np.eye(3).reshape((-1, 1))
    orthonormal_constraint = orthonormal_constraint.ravel()
    [prog.AddConstraint(orthonormal_constraint[i], 0, 0) for i in range(9)]

    total_points = 0
    for t in data['t']:
        cloud = data['cloud'][t]
        print(f'Adding cloud at t = {t} with {cloud.size()} points')
        total_points += cloud.size()
        pelvis_xyz_homogenous = np.vstack([R @ cloud.xyzs() + p, np.ones((1, cloud.size()))])
        world_z = data['pelvis_pose'][t].GetAsMatrix34()[-1, :] @ pelvis_xyz_homogenous
        prog.AddQuadraticCost(np.sum((world_z - data['foot_height'][t])**2))

    prior_R = np.array([
     [-0.000921537986509092, 0.9623567747901539, 0.2717877642265745],
     [0.9999098100938174, 0.004528347357798382, -0.012643802766864482],
     [-0.013398598655628901, 0.27175159996909126, -0.9622741529670035]]
    )
    prior_p = np.array([0.19915678083188187, -0.03634579979866665, 0.16435201748704])
    prog.AddCost(
        (np.trace(prior_R.T @ R) - 3)**2
    )
    prog.AddQuadraticCost(np.dot(prior_p - p.ravel(), prior_p - p.ravel()))
    prog.AddQuadraticCost(total_points * (prior_p[0] - p.ravel()[0]) ** 2)

    prog.SetInitialGuess(R, prior_R)
    prog.SetInitialGuess(p, prior_p)

    sol = Solve(prog)
    return RigidTransform(
        RotationMatrix(sol.GetSolution(R)),
        sol.GetSolution(p)
    )


def main():
    logfile = sys.argv[1]

    state_channel = 'CASSIE_STATE_DISPATCHER'
    pc_channel = 'CALIBRATION_PC'

    log = lcm.EventLog(logfile, "r")
    lcm_channels = cassie_default_channels
    lcm_channels[pc_channel] = lcmt_point_cloud
    data = get_log_data(
        log,
        lcm_channels,
        0, 5,
        process_lcm_data,
        state_channel,
        pc_channel
    )
    pose = solve_calibration(data)
    print(pose)


if __name__ == '__main__':
    main()

