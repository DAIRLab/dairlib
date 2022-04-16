class RandomExplorer:
    def __init__(self, nominal_swing, sigmas):
        """
        param nominal_swing: 1 + 3 * n_knots + 3 + 3 array
        param sigmas: 3 * n_knots + 3 + 3 array
        """
        self.nominal_swing = nominal_swing
        self.sigmas = sigmas
        
    def select_action(self):
        """ Outputs a random deviation from the nominal
        (excluding the n_knots).
        """

    def collect_data(self, n_steps, cassie_env):
        """ Runs the random explorer for n_steps steps,
        gathering n_steps (s, a, r, s') tuples 
        """


if __name__ == "__main__":
    main()
