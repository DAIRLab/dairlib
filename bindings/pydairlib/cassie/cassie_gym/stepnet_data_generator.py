from drake_cassie_gym import DrakeCassieGym


class StepnetDataGenerator(DrakeCassieGym):

    '''
    TODO: Add domain randomization for the following:
        Surface Normal
        Initial State
        Footstep target (need to add input port)
    '''
    def __init__(self, visualize=False):
        super(StepnetDataGenerator, self).__init__(visualize=visualize)
        self.depth_image_output_port = None
        self.foot_target_input_port = None

    def make(self, controller, urdf='examples/Cassie/urdf/cassie_v2.urdf'):
        super(StepnetDataGenerator, self).make(controller, urdf)
        self.depth_image_output_port = \
            self.cassie_sim.get_camera_out_output_port()


    # TODO: modify reset to change initial state, heightmap
    def reset(self):
        super(self).reset()

    def get_robot_centric_state(self):
        x = self.plant.GetPositionsAndVelocities(
                self.plant.GetMyMutableContextFromRoot(
                    self.sim.get_context()))
        # TODO: Transform to world-yaw view frame

    # TODO: Test and insure depth image output is being updated during sim
    def get_current_depth_image(self):
        return self.depth_image_output_port.Eval(self.cassie_sim_context)


def test_data_collection():
    generator = StepnetDataGenerator()
