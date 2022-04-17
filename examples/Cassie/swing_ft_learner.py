import numpy as np
import yaml
from swing_ft_env import CassieSwingFootEnv

def get_default_params(gains_file = "examples/Cassie/osc/osc_walking_gains_alip.yaml"):
    with open(gains_file, 'r') as f:
        gains = yaml.safe_load(f)
    
    n_knot = 5
    knots = []
    for i in range(n_knot):
        t = i/(n_knot - 1)
        x = [0.5 * (np.sin(np.pi * (t - 0.5)) + 1),
                      0.5 * (np.sin(np.pi * (t - 0.5)) + 1),
                      (gains["mid_foot_height"] / 0.58) * np.cos(np.pi * (t - 0.5))*t]
        knots += x
    vel_initial = [0, 0, 0]
    vel_final = [0, 0, gains["final_foot_velocity_z"]]
    return [n_knot] + knots + vel_initial + vel_final


class RandomExplorer:
    def __init__(self, nominal_swing, sigmas):
        """
        param nominal_swing: 1 + 3 * n_knots + 3 + 3 array
        param sigmas: 3 * n_knots + 3 + 3 array
        """
        self.nominal_swing = nominal_swing
        if len(sigmas) == 3 * nominal_swing[0] + 6:
            self.sigmas = [0] + sigmas
        elif len(sigmas) == 3 * nominal_swing[0] + 7:
            self.sigmas = sigmas
        else:
            print(f"sigmas must be of the correct length! is currently {len(sigmas)}")
            print(f"needs to be {3 * nominal_swing[0] + 6} or {3 * nominal_swing[0] + 7}")
            self.sigmas = [0] * (3 * nominal_swing[0] + 7)
        
    def select_action(self):
        """ Outputs a random deviation from the nominal
        (excluding the n_knots).
        """
        deviation = np.random.normal(np.zeros(1 + 3 * self.nominal_swing[0] + 6), self.sigmas)
        return self.nominal_swing + deviation
         
    def collect_data(self, n_steps, cassie_env):
        """ Runs the random explorer for n_steps steps,
        gathering n_steps (s, a, r, s') tuples 
        """
        data = []
        done = False
        s = cassie_env.reset()
        print("Reset method finished!")
        for _ in range(n_steps):
            a = self.select_action()
            s_prime, r, d = cassie_env.step(a)
            data.append([s, a, r, s_prime])
            s = s_prime
            done = d
        return data
        

def main():
    nominal_swing = get_default_params()
    explorer = RandomExplorer(nominal_swing, np.zeros(len(nominal_swing)))
    cassie_env = CassieSwingFootEnv(nominal_swing)
    data = explorer.collect_data(10, cassie_env)
    

if __name__ == "__main__":
    main()
