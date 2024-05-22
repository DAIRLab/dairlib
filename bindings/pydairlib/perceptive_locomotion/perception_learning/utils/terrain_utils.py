import numpy as np

from pydairlib.multibody import (
    SquareSteppingStoneList,
    LoadSteppingStonesFromYaml,
    ReExpressWorldVector3InBodyYawFrame,
    ReExpressBodyYawVector3InWorldFrame,
)


def random_stairs(min_depth: float, max_depth: float, max_height: float) \
        -> SquareSteppingStoneList:
    return make_stairs(
        5.0,
        (max_depth - min_depth) * np.random.random_sample() + min_depth,
        max_height * np.random.random_sample(),
        11,
        'up' if np.random.random_sample() > 0.5 else 'down'
    )


def make_stairs(width: float, depth: float, height: float, n: int,
                direction: str) -> SquareSteppingStoneList:
    """
        Make a staircase centered at the origin.
    """
    assert (n % 2 == 1)  # must have an odd number of steps
    assert (direction == 'up' or direction == 'down')
    signed_height = height if direction == 'up' else -height
    stones = []
    for i in range(n):
        steps_from_center = i - ((n - 1) / 2)
        x_center = depth * steps_from_center
        y_center = 0
        z_center = signed_height * steps_from_center
        stone = [
            [x_center, y_center, z_center],
            [0., 0., 1.],
            [depth, width, height],
            [0.0]
        ]
        stones.append(stone)

    stepping_stone_list = SquareSteppingStoneList([], [], [])
    stepping_stone_list.stones = stones
    stepping_stone_list.footholds, stepping_stone_list.cubes = \
        SquareSteppingStoneList.GetFootholdsWithMargin(stones, 0)

    return stepping_stone_list
