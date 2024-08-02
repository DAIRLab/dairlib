import matplotlib.pyplot as plt
from mpl_interactions import ioff, panhandler, zoom_factory
from mpl_toolkits.mplot3d import Axes3D
import yaml
import numpy as np

def flat_terrain(init_y = 10, init_x = -2.5, end_x = 31):
    '''
    Flat terrain which y increase as x increases.
    init_y : The initial y value
    init_x : The initial x value (where the terrain starts)
    end_x  : The end x value (where the terrain ends)
    '''
    stones = []
    y = init_y
    x_positions = np.arange(init_x, end_x+1, 1.0)

    for x in x_positions:
        stone = [[x, 0, 0.0], [0.0, 0.0, 1.0], [1., y, 0.1], [0.0]]
        #y = round(y + 0.3, 2)
        stones.append(stone)

    output_file_name = f'flat_new.yaml'

    with open(output_file_name, 'w') as outfile:
        outfile.write("# Yaml description of rectangular stepping stones\n")
        outfile.write("stones:\n")
        outfile.write("#  [ CENTER XYZ      NORMAL       DIMENSIONS XYZ        YAW ]\n")
        for data in stones:
            outfile.write(f"  - {data}\n")


def flat_random_blocks(num_blocks: int = 250, Gaussian: bool = False):
    '''
    To Create random blocks with values up to 4 decimal point
    '''
    for i in range(num_blocks):
        with open('flat_new.yaml', 'r') as file:
            data = yaml.safe_load(file)

        # Number of cubes
        # n = np.random.randint(10, 15)
        #n = np.random.randint(20, 30) # Easy terrain
        n = np.random.randint(20, 40) # Medium terrain
        #n = 50
        cubes = []
        for stone in data['stones']:
            center, _, dimensions, _ = stone
    
            if Gaussian:
                dimensions = np.array(dimensions)
                positions = np.random.normal(loc=center, scale=dimensions / 6, size=(n, 3))
            else:
                positions = np.random.uniform(-0.5, 0.5, size=(n, 3)) * dimensions + center

            normals = np.random.uniform(-1, 1, size=(n, 3))
            normals[:, 2] = np.maximum(normals[:, 2], 0.1)
            normals /= np.linalg.norm(normals, axis=1, keepdims=True)
            
            yaws = np.random.uniform(0, np.pi, size=n) #np.zeros((n,))
            
            for pos, norm, yaw in zip(positions, normals, yaws):
                pos = np.round(pos, 4).tolist()
                norm = np.round(norm, 4).tolist()
                #size = np.round(np.random.uniform(0.15, 0.35, size=3), 4).tolist()
                #size = np.round(np.random.uniform(0.15, 0.3, size=3), 4).tolist() # Easy terrain
                #size = np.round(np.random.uniform(0.15, 0.3, size=3), 4).tolist() # Medium terrain
                size = np.round(np.random.uniform(0.15, 0.4, size=3), 4).tolist() # Medium+ terrain
                #size = np.round(np.random.uniform(0.25, 0.4, size=3), 4).tolist() # Hard terrain
                yaw = round(yaw, 4)

                # Reject if stones are within [0.5, 0.5]
                # if abs(pos[0]) > 1:
                #print(abs(pos[0]))
                # if abs(pos[0]) > 0.5 :
                #     cubes.append([pos, norm, size, [yaw]])
                if (abs(pos[0]) < 9.5 and abs(pos[1]) < 9.5) and (abs(pos[0]) > 0.5 or abs(pos[1]) > 0.5):
                    cubes.append([pos, norm, size, [yaw]])

        output_file_name = f'terrain/flat_{i+1000}.yaml'

        with open(output_file_name, 'w') as outfile:
            outfile.write("stones:\n")
            for data in data['stones']:
                outfile.write(f"  - {data}\n")
            for cube in cubes:
                outfile.write(f"  - {cube}\n")
        print(f'YAML file "{output_file_name}" has been created.')

def staircase(start_x=-4.5, end_x=30., initial_y_dim = 10., y_increment=0.3):
    '''
    Make staircase terrain with random x and z values
    '''

    initial_z_center = 0.0

    stones = []
    y_dim = initial_y_dim
    z_center = initial_z_center
    x_center = start_x
    x_dim = 1.
    z_dim = .1

    while x_center <= end_x:
        stone = [[round(x_center, 3), 0.0, round(z_center, 3)], [0.0, 0.0, 1.0],
                [round(x_dim, 3), round(y_dim, 3), round(z_dim, 3)], [0.0]]
        stones.append(stone)
        prev_x_dim = x_dim
        x_dim = round(np.random.uniform(2.0, 3.0), 3)
        avg_x_dim = (prev_x_dim + x_dim) / 2
        x_center = round(x_center + avg_x_dim, 3)
        
        y_dim = round(y_dim + y_increment, 3)
        
        z_dim = round(np.random.uniform(0.2, 0.45), 3)
        z_center = round(z_center + z_dim/2, 3)
        
    output_file_name = f'stair.yaml'

    with open(output_file_name, 'w') as outfile:
        outfile.write("# Yaml description of rectangular stepping stones\n")
        outfile.write("stones:\n")
        outfile.write("#  [ CENTER XYZ      NORMAL       DIMENSIONS XYZ        YAW ]\n")
        for data in stones:
            outfile.write(f"  - {data}\n")

def flat_stair(start_x=-3.5, end_x=30., initial_y_dim=15., y_increment=0.3, flat_terrain=4.0):
    '''
    Make flat-staircase (up)
    flat_terrain: Flat terrain phase in meters
    '''
    initial_z_center = 0.0

    stones = []
    y_dim = initial_y_dim
    z_center = initial_z_center
    x_center = start_x
    x_dim = 1.0
    z_dim = 0.1

    while x_center <= end_x:
        if x_center <= flat_terrain:
            z_center = initial_z_center
            z_dim = 0.1
        else:
            z_dim = round(np.random.uniform(0.2, 0.35), 5)
            z_center = round(z_center + z_dim / 2, 5)

        stone = [
                [round(x_center, 5), 0.0, round(z_center, 5)],
                [0.0, 0.0, 1.0],
                [round(x_dim, 5), round(y_dim, 5), round(z_dim, 5)],
                [0.0]
                ]

        stones.append(stone)

        prev_x_dim = x_dim
        x_dim = round(np.random.uniform(1.5, 3.0), 5) if x_center > 4.0 else 1.0
        avg_x_dim = (prev_x_dim + x_dim) / 2
        x_center = round(x_center + avg_x_dim, 5)

        if x_center > flat_terrain:
            y_dim = round(y_dim + y_increment, 5)        

    output_file_name = 'flat_stair_flat.yaml'
    with open(output_file_name, 'w') as outfile:
        outfile.write("# Yaml description of rectangular stepping stones\n")
        outfile.write("stones:\n")
        outfile.write("#  [ CENTER XYZ      NORMAL       DIMENSIONS XYZ        YAW ]\n")
        for stone in stones:
            outfile.write(f"  - {stone}\n")
            
    print(f'YAML file "{output_file_name}" has been created.')

def flat_stair_flat(num_blocks = 250, start_x=-3.5, end_x=30., initial_y_dim=15.,
                    y_increment=0.3,start_flat=4., stair_up=15., end_flat=4.0, Gaussian=True):
    initial_z_center = 0.0

    stones = []
    y_dim = initial_y_dim
    z_center = initial_z_center
    x_center = start_x
    x_dim = 1.0
    z_dim = 0.1
    
    # Flat terrain phase
    end_flat = end_x - end_flat

    for i in range(num_blocks):
        while x_center <= end_x:
            # Flat phase
            if x_center <= start_flat:
                z_center = initial_z_center
                z_dim = 0.1

            # Up phase
            elif start_flat < x_center <= stair_up:
                #z_dim = round(np.random.uniform(0.15, 0.25), 5) # Easy stair height
                #z_dim = round(np.random.uniform(0.15, 0.3), 5) # Medium stair height
                z_dim = round(np.random.uniform(0.15, 0.4), 5) # Hard stair height
                
                z_center = round(z_center + z_dim / 2, 5) # + for upstair | - for downstair

            # Down phase
            elif stair_up < x_center <= end_flat:
                #z_dim = round(np.random.uniform(0.15, 0.25), 5) # Easy stair height
                #z_dim = round(np.random.uniform(0.15, 0.3), 5) # Medium stair height
                z_dim = round(np.random.uniform(0.15, 0.4), 5) # Hard stair height
                
                z_center = round(z_center - z_dim / 2, 5) # - for upstair | + for downstair

            # Flat phase
            else:
                z_dim = 0.1

            stone = [[round(x_center, 5), 0.0, round(z_center, 5)],
                    [0.0, 0.0, 1.0],
                    [round(x_dim, 5), round(y_dim, 5), round(z_dim, 5)],
                    [0.0]]

            stones.append(stone)

            prev_x_dim = x_dim
            x_dim = round(np.random.uniform(.5, 1.5), 5) if x_center > start_flat else 1.0 # stair length 50 cm ~ 1.5m
            avg_x_dim = (prev_x_dim + x_dim) / 2
            x_center = round(x_center + avg_x_dim, 5)

            if x_center > start_flat:
                y_dim = round(y_dim + y_increment, 5)

        for stone in stones:
            # Number of cubes
            # n = np.random.randint(10, 15)
            #n = np.random.randint(1, 10) # Easy terrain
            #n = np.random.randint(5, 10) # Medium terrain
            n = np.random.randint(5, 15) # Hard terrain
            
            cubes = []
            for stone in stones:
                center, _, dimensions, _ = stone

                if Gaussian:
                    dimensions = np.array(dimensions)
                    positions = np.random.normal(loc=center, scale=dimensions / 6, size=(n, 3))
                else:
                    positions = np.random.uniform(-0.5, 0.5, size=(n, 3)) * dimensions + center
                
                normals = np.random.uniform(-1, 1, size=(n, 3))
                normals[:, 2] = np.maximum(normals[:, 2], 0.1)
                normals /= np.linalg.norm(normals, axis=1, keepdims=True)

                yaws = np.random.uniform(0, np.pi, size=n)

                for pos, norm, yaw in zip(positions, normals, yaws):
                    pos = np.round(pos, 4).tolist()
                    norm = np.round(norm, 4).tolist()
                    #size = np.round(np.random.uniform(0.15, 0.3, size=3), 4).tolist()
                    #size = np.round(np.random.uniform(0.1, 0.2, size=3), 4).tolist() # Easy terrain
                    #size = np.round(np.random.uniform(0.15, 0.25, size=3), 4).tolist() # Medium terrain
                    size = np.round(np.random.uniform(0.15, 0.3, size=3), 4).tolist() # Hard terrain

                    yaw = round(yaw, 4)

                    # Reject if stones are within [1., 1.]
                    # if abs(pos[0]) > 1:

                    # Reject if stones are within [0.5, 0.5]
                    if abs(pos[0]) > 0.5:
                        cubes.append([pos, norm, size, [yaw]])

        # ustair for upward staircase
        # dstair for downward staircase
        output_file_name = f'terrain/ustair_{i}.yaml'

        with open(output_file_name, 'w') as outfile:
            outfile.write("stones:\n")
            for data in stones:
                outfile.write(f"  - {data}\n")
            for cube in cubes:
                outfile.write(f"  - {cube}\n")

        print(f'YAML file "{output_file_name}" has been created.')

def down_flat_up(num_blocks = 250, start_x=0., end_x=7., initial_y_dim=20.,
                    y_increment=0.0,start_flat=0.5, Gaussian=True):

    for i in range(num_blocks):
        big_stones = []
        big_cubes = []
        for j in range(-1, 2):
            stones = []
            initial_z_center = 0.0
            y_dim = initial_y_dim
            z_center = initial_z_center
            x_center = start_x
            x_dim = 1.0
            z_dim = 0.1
            while x_center <= end_x:
                # Flat phase
                if x_center <= start_flat:
                    z_center = initial_z_center
                    z_dim = 0.1
                    
                # Up phase
                elif start_flat < x_center <= end_x:
                    z_dim = round(np.random.uniform(0.15, 0.3), 5) # Easy stair height
                    #z_dim = round(np.random.uniform(0.2, 0.4), 5) # Medium stair height
                    #z_dim = round(np.random.uniform(0.15, 0.4), 5) # Hard stair height
                    z_center = round(z_center + z_dim / 2, 5) # + for upstair | - for downstair

                # Flat phase
                else:
                    z_dim = 0.1

                stone = [[round(x_center, 5), 0.0, round(z_center, 5)],
                        [0.0, 0.0, 1.0],
                        [round(x_dim, 5), round(y_dim, 5), round(z_dim, 5)],
                        [0.0]]

                stones.append(stone)

                prev_x_dim = x_dim
                x_dim = round(np.random.uniform(.3, 1.5), 5) if x_center > start_flat else 1.0 # stair length 50 cm ~ 1.m
                avg_x_dim = (prev_x_dim + x_dim) / 2
                x_center = round(x_center + avg_x_dim, 5)

                if x_center > start_flat:
                    y_dim = round(y_dim + y_increment, 5)

            y_dim = initial_y_dim
            z_center = initial_z_center
            x_center = start_x
            x_dim = 1.0
            z_dim = 0.1

            while -end_x <= x_center:
                # Flat phase
                if x_center >= -start_flat :
                    z_center = initial_z_center
                    z_dim = 0.1
                    
                # Down phase
                elif -end_x < x_center <= -start_flat:
                    z_dim = round(np.random.uniform(0.15, 0.3), 5) # Easy stair height
                    #z_dim = round(np.random.uniform(0.15, 0.4), 5) # Medium stair height
                    #z_dim = round(np.random.uniform(0.15, 0.4), 5) # Hard stair height
                    z_center = round(z_center - z_dim / 2, 5) # - for upstair | + for downstair

                # Flat phase
                else:
                    z_dim = 0.1

                stone = [[round(x_center, 5), 0.0, round(z_center, 5)],
                        [0.0, 0.0, 1.0],
                        [round(x_dim, 5), round(y_dim, 5), round(z_dim, 5)],
                        [0.0]]

                stones.append(stone)

                prev_x_dim = x_dim
                x_dim = round(np.random.uniform(.3, 1.5), 5) if x_center <= -start_flat else 1.0 # stair length 50 cm ~ 1.m
                avg_x_dim = (prev_x_dim + x_dim) / 2
                x_center = round(x_center - avg_x_dim, 5)
                if x_center > start_flat:
                    y_dim = round(y_dim + y_increment, 5)

            for stone in stones:
                # Number of cubes
                # n = np.random.randint(10, 15)
                #n = np.random.randint(1, 10) # Easy terrain
                #n = np.random.randint(5, 10) # Medium terrain
                #n = np.random.randint(15, 20) # Hard terrain
                n = 0
                cubes = []
                for stone in stones:
                    center, _, dimensions, _ = stone

                    if Gaussian:
                        dimensions = np.array(dimensions)
                        positions = np.random.normal(loc=center, scale=dimensions / 6, size=(n, 3))
                    else:
                        positions = np.random.uniform(-0.5, 0.5, size=(n, 3)) * dimensions + center
                    
                    normals = np.random.uniform(-1, 1, size=(n, 3))
                    normals[:, 2] = np.maximum(normals[:, 2], 0.1)
                    normals /= np.linalg.norm(normals, axis=1, keepdims=True)

                    yaws = np.random.uniform(0, np.pi, size=n)

                    for pos, norm, yaw in zip(positions, normals, yaws):
                        pos = np.round(pos, 4).tolist()
                        norm = np.round(norm, 4).tolist()
                        #size = np.round(np.random.uniform(0.15, 0.3, size=3), 4).tolist()
                        #size = np.round(np.random.uniform(0.15, 0.2, size=3), 4).tolist() # Easy terrain
                        size = np.round(np.random.uniform(0.15, 0.25, size=3), 4).tolist() # Medium terrain
                        #size = np.round(np.random.uniform(0.15, 0.3, size=3), 4).tolist() # Hard terrain

                        yaw = round(yaw, 4)

                        # Reject if stones are within [1., 1.]
                        # if abs(pos[0]) > 1:

                        # Reject if stones are within [0.5, x]
                        if abs(pos[0]) > 0.5:
                            cubes.append([pos, norm, size, [yaw]])
            
            for stone in stones:
                stone[0][0] = stone[0][0] + j * (15)
                big_stones.append(stone)
            for cube in cubes:
                cube[0][0] = cube[0][0] + j * (15)
                big_cubes.append(cube)

        output_file_name = f'terrain/dustair_{i+1000}.yaml'

        with open(output_file_name, 'w') as outfile:
            outfile.write("stones:\n")
            for data in big_stones:
                outfile.write(f"  - {data}\n")
            for cube in big_cubes:
                outfile.write(f"  - {cube}\n")

        print(f'YAML file "{output_file_name}" has been created.')

def normalize(vector):
        return np.array(vector) / np.linalg.norm(vector)

def down_flat_up_single(num_blocks = 250, start_x=0., end_x=7., initial_y_dim=20.,
                    y_increment=0.0,start_flat=0.5, Gaussian=True):

    for i in range(num_blocks):
        big_stones = []
        big_cubes = []
        for j in range(-1, 2):
            stones = []
            initial_z_center = 0.0
            y_dim = initial_y_dim
            z_center = initial_z_center
            x_center = start_x
            x_dim = 1.0
            z_dim = 0.1
            while x_center <= end_x:
                # Flat phase
                if x_center <= start_flat:
                    z_center = initial_z_center
                    z_dim = 0.1
                    normal = [0.0, 0.0, 1.0]
                    
                # Up phase
                elif start_flat < x_center <= end_x:
                    z_dim = round(np.random.uniform(0.075, 0.125), 5) # just slope
                    #z_dim = round(np.random.uniform(0.04, 0.08), 5) # slope on staircase
                    z_center = round(z_center + z_dim / 2, 5) # + for upstair | - for downstair
                    tilt_angle = np.random.uniform(low=-0.12, high=-0.06) # just slope
                    #tilt_angle = np.random.uniform(low=0.04, high=0.08) # slope on staircase
                    tilt_vector = normalize([tilt_angle, 0.0, 1.0])
                    normal = tilt_vector.tolist()
                # Flat phase
                else:
                    z_dim = 0.1

                stone = [[round(x_center, 5), 0.0, round(z_center, 5)],
                        normal,
                        [round(x_dim, 5), round(y_dim, 5), round(z_dim+0.05, 5)],
                        [0.0]]

                stones.append(stone)

                prev_x_dim = x_dim
                x_dim = round(np.random.uniform(.4, 1.), 5) if x_center > start_flat else 1.0 # stair length 50 cm ~ 1.m
                avg_x_dim = (prev_x_dim + x_dim) / 2
                x_center = round(x_center + avg_x_dim, 5)

                if x_center > start_flat:
                    y_dim = round(y_dim + y_increment, 5)

            y_dim = initial_y_dim
            z_center = initial_z_center
            x_center = start_x
            x_dim = 1.0
            z_dim = 0.1

            while -end_x <= x_center:
                # Flat phase
                if x_center >= -start_flat :
                    z_center = initial_z_center
                    z_dim = 0.1
                    normal = [0.0, 0.0, 1.0]
                    
                # Down phase
                elif -end_x < x_center <= -start_flat:
                    z_dim = round(np.random.uniform(0.15, 0.2), 5) # just slope
                    #z_dim = round(np.random.uniform(0.1, 0.15), 5)# slope on staircase
                    z_center = round(z_center - z_dim / 2, 5) # - for upstair | + for downstair
                    tilt_angle = np.random.uniform(low=-0.12, high=-0.06) # just slope
                    #tilt_angle = np.random.uniform(low=0.04, high=0.08) # slope on staircase
                    tilt_vector = normalize([tilt_angle, 0.0, 1.0])
                    normal = tilt_vector.tolist()
                # Flat phase
                else:
                    z_dim = 0.1

                stone = [[round(x_center, 5), 0.0, round(z_center, 5)],
                        normal,
                        [round(x_dim, 5), round(y_dim, 5), round(z_dim, 5)],
                        [0.0]]

                stones.append(stone)

                prev_x_dim = x_dim
                x_dim = round(np.random.uniform(.4, 1.), 5) if x_center <= -start_flat else 1.0 # stair length 50 cm ~ 1.m
                avg_x_dim = (prev_x_dim + x_dim) / 2
                x_center = round(x_center - avg_x_dim, 5)
                if x_center > start_flat:
                    y_dim = round(y_dim + y_increment, 5)

                # Number of cubes
                # n = np.random.randint(10, 15)
                #n = np.random.randint(1, 10) # Easy terrain
                #n = np.random.randint(5, 10) # Medium terrain
                #n = np.random.randint(15, 20) # Hard terrain
            n = 0
            cubes = []
            for stone in stones:
                center, _, dimensions, _ = stone

                if Gaussian:
                    dimensions = np.array(dimensions)
                    positions = np.random.normal(loc=center, scale=dimensions / 6, size=(n, 3))
                else:
                    positions = np.random.uniform(-0.5, 0.5, size=(n, 3)) * dimensions + center
                
                normals = np.random.uniform(-1, 1, size=(n, 3))
                normals[:, 2] = np.maximum(normals[:, 2], 0.1)
                normals /= np.linalg.norm(normals, axis=1, keepdims=True)

                yaws = np.random.uniform(0, np.pi, size=n)

                for pos, norm, yaw in zip(positions, normals, yaws):
                    pos = np.round(pos, 4).tolist()
                    norm = np.round(norm, 4).tolist()
                    #size = np.round(np.random.uniform(0.15, 0.3, size=3), 4).tolist()
                    size = np.round(np.random.uniform(0.15, 0.2, size=3), 4).tolist() # Easy terrain
                    #size = np.round(np.random.uniform(0.15, 0.25, size=3), 4).tolist() # Medium terrain
                    #size = np.round(np.random.uniform(0.15, 0.3, size=3), 4).tolist() # Hard terrain

                    yaw = round(yaw, 4)

                    # Reject if stones are within [1., 1.]
                    # if abs(pos[0]) > 1:

                    # Reject if stones are within [0.5, x]
                    if abs(pos[0]) > 0.5:
                        cubes.append([pos, norm, size, [yaw]])

                    
            for stone in stones:
                stone[0][0] = stone[0][0] + j * (15)
                big_stones.append(stone)
            for cube in cubes:
                cube[0][0] = cube[0][0] + j * (15)
                big_cubes.append(cube)

        output_file_name = f'terrain/stair_{i+500}.yaml'

        with open(output_file_name, 'w') as outfile:
            outfile.write("stones:\n")
            for data in big_stones:
                outfile.write(f"  - {data}\n")
            for cube in big_cubes:
                outfile.write(f"  - {cube}\n")

        print(f'YAML file "{output_file_name}" has been created.')

def down_flat_up_slope(num_blocks = 250, start_x=0., end_x=7., initial_y_dim=20.,
                    y_increment=0.0,start_flat=0.5, Gaussian=True):

    for i in range(num_blocks):
        big_stones = []
        big_cubes = []
        for j in range(-1, 2):
            stones = []
            initial_z_center = 0.0
            y_dim = initial_y_dim
            z_center = initial_z_center
            x_center = start_x
            x_dim = 1.0
            z_dim = 0.1
            while x_center <= end_x:
                # Flat phase
                if x_center <= start_flat:
                    z_center = initial_z_center
                    z_dim = 0.1
                    normal = [0.0, 0.0, 1.0]
                    
                # Up phase
                elif start_flat < x_center <= end_x:
                    z_dim = round(np.random.uniform(0.075, 0.125), 5) # just slope
                    #z_dim = round(np.random.uniform(0.04, 0.08), 5) # slope on staircase
                    z_center = round(z_center + z_dim / 2, 5) # + for upstair | - for downstair
                    tilt_angle = np.random.uniform(low=-0.1, high=0.1) # just slope
                    #tilt_angle = np.random.uniform(low=0.04, high=0.08) # slope on staircase
                    tilt_vector = normalize([0.0, tilt_angle, 1.0])
                    normal = tilt_vector.tolist()
                # Flat phase
                else:
                    z_dim = 0.1

                stone = [[round(x_center, 5), 0.0, round(z_center, 5)],
                        normal,
                        [round(x_dim, 5), round(y_dim, 5), round(z_dim, 5)],
                        [0.0]]

                stones.append(stone)

                prev_x_dim = x_dim
                x_dim = round(np.random.uniform(.4, 1.), 5) if x_center > start_flat else 1.0 # stair length 50 cm ~ 1.m
                avg_x_dim = (prev_x_dim + x_dim) / 2
                x_center = round(x_center + avg_x_dim, 5)

                if x_center > start_flat:
                    y_dim = round(y_dim + y_increment, 5)

            y_dim = initial_y_dim
            z_center = initial_z_center
            x_center = start_x
            x_dim = 1.0
            z_dim = 0.1

            while -end_x <= x_center:
                # Flat phase
                if x_center >= -start_flat :
                    z_center = initial_z_center
                    z_dim = 0.1
                    normal = [0.0, 0.0, 1.0]
                    
                # Down phase
                elif -end_x < x_center <= -start_flat:
                    z_dim = round(np.random.uniform(0.15, 0.2), 5) # just slope
                    #z_dim = round(np.random.uniform(0.1, 0.15), 5)# slope on staircase
                    z_center = round(z_center - z_dim / 2, 5) # - for upstair | + for downstair
                    tilt_angle = np.random.uniform(low=-0.12, high=-0.06) # just slope
                    #tilt_angle = np.random.uniform(low=0.04, high=0.08) # slope on staircase
                    tilt_vector = normalize([tilt_angle, 0.0, 1.0])
                    normal = tilt_vector.tolist()
                # Flat phase
                else:
                    z_dim = 0.1

                stone = [[round(x_center, 5), 0.0, round(z_center, 5)],
                        normal,
                        [round(x_dim, 5), round(y_dim, 5), round(z_dim, 5)],
                        [0.0]]

                stones.append(stone)

                prev_x_dim = x_dim
                x_dim = round(np.random.uniform(.4, 1.), 5) if x_center <= -start_flat else 1.0 # stair length 50 cm ~ 1.m
                avg_x_dim = (prev_x_dim + x_dim) / 2
                x_center = round(x_center - avg_x_dim, 5)
                if x_center > start_flat:
                    y_dim = round(y_dim + y_increment, 5)

                # Number of cubes
                # n = np.random.randint(10, 15)
                #n = np.random.randint(1, 10) # Easy terrain
                #n = np.random.randint(5, 10) # Medium terrain
                #n = np.random.randint(15, 20) # Hard terrain
            n = 0
            cubes = []
            for stone in stones:
                center, _, dimensions, _ = stone

                if Gaussian:
                    dimensions = np.array(dimensions)
                    positions = np.random.normal(loc=center, scale=dimensions / 6, size=(n, 3))
                else:
                    positions = np.random.uniform(-0.5, 0.5, size=(n, 3)) * dimensions + center
                
                normals = np.random.uniform(-1, 1, size=(n, 3))
                normals[:, 2] = np.maximum(normals[:, 2], 0.1)
                normals /= np.linalg.norm(normals, axis=1, keepdims=True)

                yaws = np.random.uniform(0, np.pi, size=n)

                for pos, norm, yaw in zip(positions, normals, yaws):
                    pos = np.round(pos, 4).tolist()
                    norm = np.round(norm, 4).tolist()
                    #size = np.round(np.random.uniform(0.15, 0.3, size=3), 4).tolist()
                    size = np.round(np.random.uniform(0.15, 0.2, size=3), 4).tolist() # Easy terrain
                    #size = np.round(np.random.uniform(0.15, 0.25, size=3), 4).tolist() # Medium terrain
                    #size = np.round(np.random.uniform(0.15, 0.3, size=3), 4).tolist() # Hard terrain

                    yaw = round(yaw, 4)

                    # Reject if stones are within [1., 1.]
                    # if abs(pos[0]) > 1:

                    # Reject if stones are within [0.5, x]
                    if abs(pos[0]) > 0.5:
                        cubes.append([pos, norm, size, [yaw]])

                    
            for stone in stones:
                stone[0][0] = stone[0][0] + j * (15)
                big_stones.append(stone)
            for cube in cubes:
                cube[0][0] = cube[0][0] + j * (15)
                big_cubes.append(cube)

        output_file_name = f'terrain/stair_{i+500}.yaml'

        with open(output_file_name, 'w') as outfile:
            outfile.write("stones:\n")
            for data in big_stones:
                outfile.write(f"  - {data}\n")
            for cube in big_cubes:
                outfile.write(f"  - {cube}\n")

        print(f'YAML file "{output_file_name}" has been created.')

def plot_terrain(file_path: str = './flat_stair_flat.yaml'):

    # Load YAML file
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)

    data = data['stones']

    with plt.ioff():
        fig, ax = plt.subplots(subplot_kw={'projection': '3d'})

    min_x, min_y, min_z = np.inf, np.inf, np.inf
    max_x, max_y, max_z = -np.inf, -np.inf, -np.inf

    # Plot each point in the terrain data
    for point in data:
        center, normal, dimensions, yaw = point
        x, y, z = center
        dx, dy, dz = dimensions
        min_x = min(min_x, x - dx/2)
        max_x = max(max_x, x + dx/2)
        min_y = min(min_y, y - dy/2)
        max_y = max(max_y, y + dy/2)
        min_z = min(min_z, z)
        max_z = max(max_z, z)
        ax.plot_surface(np.array([[x-dx/2, x+dx/2], [x-dx/2, x+dx/2]]), 
                        np.array([[y-dy/2, y-dy/2], [y+dy/2, y+dy/2]]), 
                        np.array([[z, z], [z, z]]),
                        alpha=0.8)
                        
    # Set axis limits and labels
    ax.set_xlim(min_x-5, max_x+5)
    ax.set_ylim(min_y-3, max_y+3)
    ax.set_zlim(min_z-3, max_z+3)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Set the aspect ratio of the plot to be equal
    ax.set_box_aspect([1,1,1])

    disconnect_zoom = zoom_factory(ax)
    pan_handler = panhandler(fig)
    plt.show()

if __name__ == '__main__':
    #file_path = './terrain/flat_0.yaml'
    #file_path = './terrain/dustair_0.yaml'
    #file_path = './flat_new.yaml'
    #file_path = './terrain/stair_0.yaml'
    #plot_terrain(file_path=file_path)
    
    down_flat_up(num_blocks=500, start_x=0., end_x=7., initial_y_dim=20.,
                    y_increment=0.0,start_flat=0.5, Gaussian=False)
    #file_path = './terrain/dustair_0.yaml'
    #plot_terrain(file_path=file_path)
    
    #down_flat_up_single(num_blocks=500,start_x=0.,end_x=10,initial_y_dim=20.,y_increment=0.0,start_flat=0.5, Gaussian=False)
    #down_flat_up_slope(num_blocks=1,start_x=0.,end_x=10,initial_y_dim=20.,y_increment=0.0,start_flat=0.5, Gaussian=False)

    #staircase(start_x=-10, end_x=10., initial_y_dim = 21., y_increment=0.)
    #flat_stair(start_x=-3.5, end_x=30., initial_y_dim=15., y_increment=0.3, flat_terrain=4.0)
    #flat_terrain(init_y = 21, init_x = -10, end_x = 10)
    #flat_random_blocks(num_blocks=500, Gaussian=False)
    
    #flat_stair_flat(num_blocks=1, start_x=-3.5, end_x=12., initial_y_dim=15., y_increment=0.3, start_flat=1., stair_up=5., end_flat=2.0, Gaussian=False)