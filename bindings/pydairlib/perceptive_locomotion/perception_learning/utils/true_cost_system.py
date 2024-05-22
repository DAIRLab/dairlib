import numpy as np

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

from pydrake.systems.all import (
    DiagramBuilder,
    BasicVector,
    LeafSystem,
    Context,
    InputPort,
    OutputPort,
    EventStatus
)

from pydrake.systems.framework import (
    DiscreteValues,
)

from pydairlib.perceptive_locomotion.systems.alip_lqr import (
    AlipFootstepLQROptions)


# This is a system that accumulates cost over time, once per stride.
class CumulativeCost(LeafSystem):
    def __init__(self, alip_params: AlipFootstepLQROptions):
        super().__init__()

        self.params = alip_params
        self.this_stride_finished = self.DeclareDiscreteState(1)
        self.cumulative_cost = self.DeclareDiscreteState(1)
        self.DeclarePerStepDiscreteUpdateEvent(self.calculate_cumulative_cost)

        self.input_port_indices = {
            'lqr_reference': self.DeclareVectorInputPort(
                "xd_ud[x,y]", 6
            ).get_index(),
            'x': self.DeclareVectorInputPort(
                'x', 4
            ).get_index(),
            'fsm': self.DeclareVectorInputPort(
                "fsm", 1
            ).get_index(),
            'time_until_switch': self.DeclareVectorInputPort(
                "time_until_switch", 1
            ).get_index(),
            'footstep_command': self.DeclareVectorInputPort(
                'footstep_command', 3
            ).get_index()
        }

        self.output_port_indices = {
            'cost': self.DeclareStateOutputPort(
                "cost", self.cumulative_cost).get_index()
        }

    def get_input_port_by_name(self, name: str) -> InputPort:
        assert (name in self.input_port_indices)
        return self.get_input_port(self.input_port_indices[name])

    def get_output_port_by_name(self, name: str) -> OutputPort:
        assert (name in self.output_port_indices)
        return self.get_output_port(self.output_port_indices[name])
    
    def calculate_cumulative_cost(self, context: Context, discrete_state: DiscreteValues) -> None:
        fsm = int(self.EvalVectorInput(context, self.input_port_indices['fsm'])[0])
        time_until_switch = self.EvalVectorInput(context, self.input_port_indices['time_until_switch'])[0]
        t_threshold = 0.025

        if time_until_switch > t_threshold:
            discrete_state.set_value(self.this_stride_finished, np.array([0]))

        elif discrete_state.value(self.this_stride_finished)[0] == 0 and fsm <= 1:
            x = self.EvalVectorInput(context, self.input_port_indices['x']).value()
            u = self.EvalVectorInput(context, self.input_port_indices['footstep_command']).value()[:2]
            xd_ud = self.EvalVectorInput(context, self.input_port_indices['lqr_reference'])
            xd = xd_ud.value()[:4]
            ud = xd_ud.value()[4:]

            cost_so_far = discrete_state.value(self.cumulative_cost)[0]
            curr_cost = (x - xd).T @ self.params.Q @ (x - xd) + (u - ud).T @ self.params.R @ (u - ud)
            cost_so_far += curr_cost

            discrete_state.set_value(self.cumulative_cost, np.array([cost_so_far]))
            discrete_state.set_value(self.this_stride_finished, np.array([1]))

        return EventStatus.Succeeded()

    @staticmethod
    def AddToBuilder(builder: DiagramBuilder, sim_env, footstep_controller) -> "CumulativeCost":
        cost_system = CumulativeCost(footstep_controller.params)
        builder.AddSystem(cost_system)

        for controller_port in ['lqr_reference', 'x', 'footstep_command']:
            builder.Connect(
                footstep_controller.get_output_port_by_name(controller_port),
                cost_system.get_input_port_by_name(controller_port)
            )
        for sim_port in ['fsm', 'time_until_switch']:
            builder.Connect(
                sim_env.get_output_port_by_name(sim_port),
                cost_system.get_input_port_by_name(sim_port)
            )
        return cost_system
