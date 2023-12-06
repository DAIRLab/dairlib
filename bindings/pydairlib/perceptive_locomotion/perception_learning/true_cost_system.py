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
    OutputPort
)

from pydrake.multibody.plant import MultibodyPlant

from pydairlib.perceptive_locomotion.perception_learning.alip_lqr import (
    AlipFootstepLQROptions)

# define another leaf system that takes alip_state as input evaluates the time 
# between impacts and computes the cumulative cost
class CumulativeCost(LeafSystem):
    def __init__(self, alip_params: AlipFootstepLQROptions):
        super().__init__()

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
            'cumulative_cost': self.DeclareVectorOutputPort(
                "cumulative_cost", 1, self.calculate_cumulative_cost
            ).get_index()
        }

        self.params = alip_params
        self.cost = 0
        self.prev_stance = None
        self.curr_stance = None
        # This discrete state is used to make sure to update the cost only once in every stance
        self.isCostComputed = self.DeclareDiscreteState(1)
        self.DeclarePerStepDiscreteUpdateEvent(self.calculate_cumulative_cost)

    def get_input_port_by_name(self, name: str) -> InputPort:
        assert (name in self.input_port_indices)
        return self.get_input_port(self.input_port_indices[name])

    def get_output_port_by_name(self, name: str) -> OutputPort:
        assert (name in self.output_port_indices)
        return self.get_output_port(self.output_port_indices[name])
    
    def calculate_cumulative_cost(self, context: Context, cumulative_cost: BasicVector) -> None:
        fsm = self.EvalVectorInput(context, self.input_port_indices['fsm'])
        time_until_switch = self.EvalVectorInput(context, self.input_port_indices['time_until_switch'])

        self.curr_stance = "Left" if fsm == 0 or fsm == 3 else "Right"

        # if prev stance is none, then it is in the first stance
        # if curr stance is the same as prev stance and time until switch is less than epsilon, then update cost and update the discrete state
        # Only compute cost once in every stance

        if self.prev_stance is None:
            self.prev_stance = self.curr_stance
            # TODO: Figure out how to set cost to x0.T @ Q @ x0 here.
        else:
            if ((self.curr_stance == self.prev_stance) & (time_until_switch[0] < 0.5) & (self.isCostComputed.get_value() == 0)):
                cost = BasicVector(1)
                cost = np.array([300])
                self.prev_stance = self.curr_stance
                # print("directly printing cost = ", cost)
                # update ComputeCost
                self.DeclarePerStepDiscreteUpdateEvent()
                cumulative_cost.set_value(cost)
                self.isCostComputed.set_value(1)
                print("updated isCostComputed = ", self.get_mutable_vector(isCostComputed).get_value))

        if self.prev_stance != self.curr_stance:
            self.prev_stance = self.curr_stance
            self.isCostComputed.set_value(0)
            print("resetting isComputedFlag = ", context.get_discrete_state_vector()())
            

                
        

        if time_until_switch[0] < 0.5:
            # print("need to calculate cost")
            # set output port value
            cost = BasicVector(1)
            cost = np.array([300])
            # print("directly printing cost = ", cost)
            cumulative_cost.set_value(cost)

        x = self.EvalVectorInput(context, self.input_port_indices['x'])
        xd_ud = self.EvalVectorInput(context, self.input_port_indices['lqr_reference'])
        # self.cost += (x - xd_ud[:4]).dot(self.params.Q).dot(x - xd_ud[:4]) + \
        #     0.5 * xd_ud[4:].dot(self.params.R).dot(xd_ud[4:])
        # self.get_mutable_output_port(self.output_port_indices['cumulative_cost']). \
        #     set_value(self.cost)

    # def calculate_time_between_impacts(self, context: Context, time_between_impacts: BasicVector) -> None:
    #     x = self.EvalVectorInput(context, self.input_port_indices['x'])
    #     time_between_impacts.SetAtIndex(0, x[3])


     