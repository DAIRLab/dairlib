import numpy as np
from typing import Tuple, overload
from dataclasses import dataclass, field

import io
from yaml import load, dump

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

from pydrake.systems.all import (
    DiscreteTimeLinearQuadraticRegulator,
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

from pydrake.multibody.plant import MultibodyPlant

from pydairlib.perceptive_locomotion.perception_learning.alip_lqr import (
    AlipFootstepLQROptions)


# This is a system that accumulates cost over time, once per stride.
class CumulativeCost(LeafSystem):
    def __init__(self, alip_params: AlipFootstepLQROptions):
        super().__init__()

        self.params = alip_params
        self.calculated_cost_this_stride = self.DeclareDiscreteState(1)
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
            ).get_index()
        }

        self.output_port_indices = {
            'cost' : self.DeclareStateOutputPort(
                "cost", self.cumulative_cost).get_index()
        }

    def get_input_port_by_name(self, name: str) -> InputPort:
        assert (name in self.input_port_indices)
        return self.get_input_port(self.input_port_indices[name])

    def get_output_port_by_name(self, name: str) -> OutputPort:
        assert (name in self.output_port_indices)
        return self.get_output_port(self.output_port_indices[name])
    
    def calculate_cumulative_cost(self, context: Context, output_cost : DiscreteValues) -> None:
        fsm = self.EvalVectorInput(context, self.input_port_indices['fsm'])
        time_until_switch = self.EvalVectorInput(context, self.input_port_indices['time_until_switch'])
        t_threshold = 0.1

        x = self.EvalVectorInput(context, self.input_port_indices['x'])
        x = x.value()
        xd_ud = self.EvalVectorInput(context, self.input_port_indices['lqr_reference'])
        xd = xd_ud.value()[:4]
        ud = xd_ud.value()[4:]

        if time_until_switch[0] > t_threshold:
            context.get_mutable_discrete_state(self.calculated_cost_this_stride).set_value(np.array([0]))
        elif ((context.get_mutable_discrete_state(self.calculated_cost_this_stride).get_value()[0] == 0) & (int(fsm[0]) == 0 or int(fsm[0]) == 1)):
            cost_so_far = context.get_mutable_discrete_state(self.cumulative_cost).get_value()[0]
            curr_cost = (x - xd).T @ self.params.Q @ (x - xd) + ud.T @ self.params.R @ ud
            cost_so_far += curr_cost
            
            context.get_mutable_discrete_state(self.cumulative_cost).set_value(np.array([cost_so_far]))
            context.get_mutable_discrete_state(self.calculated_cost_this_stride).set_value(np.array([1]))

        return EventStatus.Succeeded()