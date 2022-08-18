import yaml
import numpy as np
from pydantic import BaseModel, Field

from pydrake.systems.sensors import CameraInfo
from pydairlib.cassie.simulators import CassieVisionSimDiagram


def default_array(default):
    return Field(default_factory=lambda: np.array(default))


class StepnetDataClass(BaseModel):
    class Config:
        arbitrary_types_allowed = True


class DomainRandomizationBounds(StepnetDataClass):
    normal: np.ndarray = default_array([0.1, 0.1, 0.0])
    map_yaw: np.ndarray = default_array([-np.pi, np.pi])
    mu: np.ndarray = default_array([0.4, 1.0])


class CassieGymParams(StepnetDataClass):
    """
        Container class for the parameters which could
        be randomized for simulation
    """
    terrain_normal: np.ndarray = default_array([0.0, 0.0, 1.0])
    x_init: np.ndarray = default_array(
        [1, 0, 0, 0, 0, 0, 0.85,
         -0.0358, 0, 0.674, -1.588, -0.0458, 1.909, -0.0382, -1.823,
         0.0358, 0, 0.674, -1.588, -0.0458, 1.909, -0.0382, -1.823,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    map_yaw: float = 0
    mu: float = 0.8
    add_terrain: bool = False

    @staticmethod
    def make_random(ic_file_path, domain=DomainRandomizationBounds()):
        ics = np.load(ic_file_path)
        x = ics[np.random.choice(ics.shape[0], size=1, replace=False)].ravel()
        normal = np.random.uniform(
            low=np.array([0., 0., 1.]) - domain.normal,
            high=np.array([0., 0., 1.]) + domain.normal,
        )
        map_yaw = np.random.uniform(
            low=domain.map_yaw[0],
            high=domain.map_yaw[1]
        )
        mu = np.random.uniform(low=domain.mu[0], high=domain.mu[1])
        return CassieGymParams(
            terrain_normal=normal,
            x_init=x,
            map_yaw=map_yaw,
            mu=mu,
            add_terrain=True
        )

    @staticmethod
    def make_flat(ic_file_path):
        ics = np.load(ic_file_path)
        x = ics[np.random.choice(ics.shape[0], size=1, replace=False)].ravel()
        return CassieGymParams(x_init=x)


class DepthCameraInfo(StepnetDataClass):
    width: int = 640
    height: int = 480
    focal_x: float = 390.743
    focal_y: float = 390.743
    center_x: float = 323.332
    center_y: float = 241.387

    def as_drake_camera_info(self):
        return CameraInfo(
            width=self.width,
            height=self.height,
            focal_x=self.focal_x,
            focal_y=self.focal_y,
            center_x=self.center_x,
            center_y=self.center_y)

    def get_pelvis_to_image_tf(self):
        X_CP = CassieVisionSimDiagram.default_camera_transform()
        return self.as_drake_camera_info().intrinsic_matrix() @ \
               X_CP.inverse().GetAsMatrix34()


class DataGeneratorParams(StepnetDataClass):
    """ Depth Camera Info """
    depth_camera_info: DepthCameraInfo = DepthCameraInfo()

    """ How much noise to add to target footsteps """
    target_xyz_noise_bound: np.ndarray = default_array([1.0, 1.0, 0.0])
    target_yaw_noise_bound: float = np.pi / 2
    target_time_bounds: np.ndarray = default_array([0.1, 0.6])
    randomize_time: bool = False
    randomize_yaw: bool = False

    """ Bounds on footstep coordinates"""
    target_lb: np.ndarray = default_array([-2.0, -2.0, -0.5])
    target_ub: np.ndarray = default_array([2.0, 2.0, 0.5])

    """ Bounds on radio commands """
    radio_bound: np.ndarray = default_array([1.0, 1.0])

    """ simulation params """
    depth_var_z: float = 0.01
    sim_duration: float = 0.35
    max_error: float = 1.0


class DataCollectionParams(StepnetDataClass):
    """ Dataset size"""
    nmaps: int
    nsteps: int
    nthreads: int
    robot_path: str
    depth_path: str
    has_terrain: bool

    """ For flat ground, a static transform from world to errormap coordinates.
    For terrain, the transform from pelvis frame to depth frame """
    generator_params: DataGeneratorParams = DataGeneratorParams()
    target_to_map_tf: np.ndarray = None

    """ Depth scaling for conversion to .png  """
    depth_scale: float = 6553.5 # (2^16 - 1) / 2

    """ Terrain and terrain parameters """
    randomization_bounds: DomainRandomizationBounds = \
        DomainRandomizationBounds()


def recursive_to_list_traverse(top_level_dict):
    for key, value in top_level_dict.items():
        print(key)
        if type(value) is dict:
            print('recurse')
            recursive_to_list_traverse(value)
        if type(value) is np.ndarray:
            print('ndarray')
            top_level_dict[key] = value.tolist()


def save_config(config, filepath):
    m_dict = config.dict()
    recursive_to_list_traverse(m_dict)
    with open(filepath, 'w') as yp:
        yaml.dump(m_dict, yp)
