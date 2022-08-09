import copy

import numpy as np
import math

from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import *
from pydrake.systems.analysis import Simulator

from pydairlib.common import FindResourceOrThrow
from pydairlib.cassie.cassie_utils import *
from pydairlib.multibody import *
from pydairlib.systems.primitives import *
from dairlib import lcmt_cassie_out
from pydairlib.cassie.gym_envs.cassie_env_state import CassieEnvState, quat_to_rotation, \
    reexpress_state_local_to_global_omega, reexpress_state_global_to_local_omega
from pydairlib.cassie.mujoco.drake_to_mujoco_converter import DrakeToMujocoConverter
# from drake_to_mujoco_converter import DrakeToMujocoConverter

from pydairlib.cassie.mujoco.cassiemujoco import *
from pydairlib.cassie.mujoco.mujoco_lcm_utils import *
from pydairlib.cassie.mujoco.drake_to_mujoco_converter import DrakeToMujocoConverter

from pydairlib.systems import (RobotOutputSender, RobotOutputReceiver)


class MuJoCoCassieGym():
    def __init__(self, reward_func, visualize=False, model_xml='cassie.xml', dynamics_randomization=True):
        self.sim_dt = 8e-5
        self.gym_dt = 1e-3
        self.visualize = visualize
        self.reward_func = reward_func
        # hardware logs are 50ms long and start approximately 5ms before impact
        # the simulator will check to make sure ground reaction forces are first detected within 3-7ms
        self.start_time = 0.00
        self.current_time = 0.00
        self.end_time = 7.5

        self.default_model_directory = '/home/yangwill/workspace/cassie-mujoco-sim/model/'
        self.default_model_file = '/home/yangwill/workspace/cassie-mujoco-sim/model/cassie.xml'

        self.simulator = CassieSim(self.default_model_directory + model_xml)
        if self.visualize:
            self.cassie_vis = CassieVis(self.simulator)

        '''
        Copied from apex/cassie.py 
        '''
        self.max_simrate = 1.3 * self.gym_dt
        self.min_simrate = 0.7 * self.gym_dt
        self.dynamics_randomization = dynamics_randomization
        self.dynamics_randomization = dynamics_randomization
        self.slope_rand = dynamics_randomization
        self.joint_rand = dynamics_randomization

        self.max_pitch_incline = 0.03
        self.max_roll_incline = 0.03

        self.encoder_noise = 0.01

        # self.damping_low = 0.3
        self.damping_low = 0.8
        # self.damping_high = 5.0
        self.damping_high = 2.0

        # self.mass_low = 0.5
        self.mass_low = 0.8
        # self.mass_high = 1.5
        self.mass_high = 1.2

        # self.fric_low = 0.4
        self.fric_low = 0.6
        self.fric_high = 1.1

        self.speed = 0
        self.side_speed = 0
        self.orient_add = 0

        self.default_damping = self.simulator.get_dof_damping()
        self.default_mass = self.simulator.get_body_mass()
        self.default_ipos = self.simulator.get_body_ipos()
        self.default_fric = self.simulator.get_geom_friction()
        self.default_rgba = self.simulator.get_geom_rgba()
        self.default_quat = self.simulator.get_geom_quat()

        self.motor_encoder_noise = np.zeros(10)
        self.joint_encoder_noise = np.zeros(6)

        self.cassie_in = cassie_user_in_t()
        self.cassie_out = cassie_out_t()
        self.cassie_out_lcm = lcmt_cassie_out()
        self.action_dim = 10
        self.state_dim = 45
        self.x_init = np.array(
            [1, 0, 0, 0, 0, 0, 0.85, -0.0358636, 0, 0.67432, -1.588, -0.0458742, 1.90918,
             -0.0381073, -1.82312, 0.0358636, 0, 0.67432, -1.588, -0.0457885, 1.90919, -0.0382424, -1.82321,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.prev_cassie_state = None
        self.controller = None
        self.terminated = False
        self.initialized = False
        self.actuator_index_map = {'hip_roll_left_motor': 0,
                                   'hip_yaw_left_motor': 1,
                                   'hip_pitch_left_motor': 2,
                                   'knee_left_motor': 3,
                                   'toe_left_motor': 4,
                                   'hip_roll_right_motor': 5,
                                   'hip_yaw_right_motor': 6,
                                   'hip_pitch_right_motor': 7,
                                   'knee_right_motor': 8,
                                   'toe_right_motor': 9}
        # sim states
        self.l_foot_frc = 0
        self.r_foot_frc = 0
        foot_pos = np.zeros(6)
        self.l_foot_pos = np.zeros(3)
        self.r_foot_pos = np.zeros(3)
        self.l_foot_orient_cost = 0
        self.r_foot_orient_cost = 0
        self.neutral_foot_orient = np.array(
            [-0.24790886454547323, -0.24679713195445646, -0.6609396704367185, 0.663921021343526])

    def make(self, controller):
        self.builder = DiagramBuilder()
        self.controller = controller

        self.builder.AddSystem(self.controller)
        self.robot_output_sender = RobotOutputSender(self.controller.get_plant(), False)
        self.builder.AddSystem(self.robot_output_sender)

        self.builder.Connect(self.robot_output_sender.get_output_port(), self.controller.get_input_port_state())
        self.drake_to_mujoco_converter = DrakeToMujocoConverter(self.sim_dt)

        self.diagram = self.builder.Build()
        self.drake_sim = Simulator(self.diagram)
        self.robot_output_sender_context = self.diagram.GetMutableSubsystemContext(self.robot_output_sender,
                                                                                   self.drake_sim.get_mutable_context())
        self.controller_context = self.diagram.GetMutableSubsystemContext(self.controller,
                                                                          self.drake_sim.get_mutable_context())
        self.controller_output_port = self.controller.get_output_port_torque()
        self.radio_input_port = self.controller.get_input_port_radio()
        self.drake_sim.get_mutable_context().SetTime(self.start_time)
        self.reset()
        self.initialized = True

    def reset(self):
        self.randomize_sim_params()

        q_mujoco, v_mujoco = self.drake_to_mujoco_converter.convert_to_mujoco(
            reexpress_state_global_to_local_omega(self.x_init))
        mujoco_state = self.simulator.get_state()
        mujoco_state.set_qpos(q_mujoco)
        mujoco_state.set_qvel(v_mujoco)
        mujoco_state.set_time(self.start_time)
        self.simulator.set_state(mujoco_state)

        self.drake_sim.get_mutable_context().SetTime(self.start_time)
        u = np.zeros(10)
        self.drake_sim.Initialize()
        self.current_time = self.start_time
        self.prev_cassie_state = CassieEnvState(self.current_time, self.x_init, u, np.zeros(18))
        self.cassie_state = CassieEnvState(self.current_time, self.x_init, u, np.zeros(18))
        cassie_in, u_mujoco = self.pack_input(self.cassie_in, u)
        self.cassie_out = self.simulator.step(cassie_in)
        self.cassie_out_lcm = self.pack_cassie_out(self.cassie_out)
        self.terminated = False
        return

    def advance_to(self, time):
        cumulative_reward = 0
        while self.current_time < time and not self.terminated:
            _, reward = self.step()
            cumulative_reward += reward
        return cumulative_reward

    def check_termination(self):
        return self.cassie_state.get_fb_positions()[2] < 0.4

    def velocity_profile(self, timestep):
        velocity_command = np.zeros(18)
        velocity_command[2] = min(1.0 * timestep, 4.0)
        # velocity_command[2] = 5.0
        return velocity_command

    def step(self, action=np.zeros(18)):
        if not self.initialized:
            print("Call make() before calling step() or advance()")

        if self.dynamics_randomization:
            timestep = np.random.uniform(self.max_simrate, self.min_simrate)
        else:
            timestep = self.gym_dt
        next_timestep = self.drake_sim.get_context().get_time() + timestep

        self.robot_output_sender.get_input_port_state().FixValue(self.robot_output_sender_context, self.cassie_state.x)
        action = self.velocity_profile(next_timestep)
        self.radio_input_port.FixValue(self.controller_context, action)

        u = self.controller_output_port.Eval(self.controller_context)[:-1]  # remove the timestamp
        cassie_in, u_mujoco = self.pack_input(self.cassie_in, u)
        self.drake_sim.AdvanceTo(next_timestep)
        # self.reward_func.reset_clock_reward()
        while self.simulator.time() < next_timestep:
            self.simulator.step(cassie_in)
            foot_pos = self.simulator.foot_pos()
            # self.reward_func.update_clock_reward(self.simulator.get_foot_forces(), foot_pos,
            #                                      self.simulator.xquat("left-foot"), self.simulator.xquat("right-foot"))

        if self.visualize:
            self.cassie_vis.draw(self.simulator)

        self.current_time = next_timestep
        t = self.simulator.time()
        # q = np.copy()
        # v = np.copy()
        q, v = self.drake_to_mujoco_converter.convert_to_drake(self.simulator.qpos(), self.simulator.qvel())
        self.current_time = t
        x = np.hstack((q, v))
        x = reexpress_state_local_to_global_omega(x)
        self.cassie_state = CassieEnvState(self.current_time, x, u, action)
        reward = self.reward_func.compute_reward(timestep, self.cassie_state, self.prev_cassie_state)
        self.terminated = self.check_termination()
        self.prev_cassie_state = self.cassie_state
        return self.cassie_state, reward

    def pack_input(self, cassie_in, u_drake):
        act_map = self.drake_to_mujoco_converter.act_map
        # Set control parameters
        u_mujoco = np.zeros(10)
        for u_name in act_map:
            cassie_in.torque[self.actuator_index_map[u_name]] = u_drake[act_map[u_name]]
            u_mujoco[self.actuator_index_map[u_name]] = u_drake[act_map[u_name]]
        return cassie_in, u_mujoco

    def pack_cassie_out(self, cassie_out):
        cassie_out_lcm = lcmt_cassie_out()
        cassie_out_lcm.utime = self.current_time * 1e6
        cassie_out_lcm.pelvis.targetPc.etherCatStatus = cassie_out.pelvis.targetPc.etherCatStatus
        cassie_out_lcm.pelvis.targetPc.etherCatNotifications = cassie_out.pelvis.targetPc.etherCatNotifications
        cassie_out_lcm.pelvis.targetPc.taskExecutionTime = cassie_out.pelvis.targetPc.taskExecutionTime
        cassie_out_lcm.pelvis.targetPc.overloadCounter = cassie_out.pelvis.targetPc.overloadCounter
        cassie_out_lcm.pelvis.targetPc.cpuTemperature = cassie_out.pelvis.targetPc.cpuTemperature
        cassie_out_lcm.pelvis.battery.dataGood = cassie_out.pelvis.battery.dataGood
        cassie_out_lcm.pelvis.battery.stateOfCharge = cassie_out.pelvis.battery.stateOfCharge
        cassie_out_lcm.pelvis.battery.current = cassie_out.pelvis.battery.current
        cassie_out_lcm.pelvis.battery.voltage = cassie_out.pelvis.battery.voltage
        cassie_out_lcm.pelvis.battery.temperature = cassie_out.pelvis.battery.temperature
        cassie_out_lcm.pelvis.radio.radioReceiverSignalGood = cassie_out.pelvis.radio.radioReceiverSignalGood
        cassie_out_lcm.pelvis.radio.receiverMedullaSignalGood = cassie_out.pelvis.radio.receiverMedullaSignalGood
        cassie_out_lcm.pelvis.radio.channel = cassie_out.pelvis.radio.channel
        cassie_out_lcm.pelvis.radio.receiverMedullaSignalGood = cassie_out.pelvis.radio.receiverMedullaSignalGood
        cassie_out_lcm.pelvis.battery.temperature = cassie_out.pelvis.battery.temperature
        cassie_out_lcm.pelvis.vectorNav.dataGood = cassie_out.pelvis.vectorNav.dataGood
        cassie_out_lcm.pelvis.vectorNav.vpeStatus = cassie_out.pelvis.vectorNav.vpeStatus
        cassie_out_lcm.pelvis.vectorNav.pressure = cassie_out.pelvis.vectorNav.pressure
        cassie_out_lcm.pelvis.vectorNav.temperature = cassie_out.pelvis.vectorNav.temperature
        cassie_out_lcm.pelvis.vectorNav.magneticField = cassie_out.pelvis.vectorNav.magneticField
        cassie_out_lcm.pelvis.vectorNav.angularVelocity = cassie_out.pelvis.vectorNav.angularVelocity
        cassie_out_lcm.pelvis.vectorNav.linearAcceleration = cassie_out.pelvis.vectorNav.linearAcceleration
        cassie_out_lcm.pelvis.vectorNav.orientation = cassie_out.pelvis.vectorNav.orientation
        cassie_out_lcm.pelvis.medullaCounter = cassie_out.pelvis.medullaCounter
        cassie_out_lcm.pelvis.medullaCpuLoad = cassie_out.pelvis.medullaCpuLoad
        cassie_out_lcm.pelvis.bleederState = cassie_out.pelvis.bleederState
        cassie_out_lcm.pelvis.leftReedSwitchState = cassie_out.pelvis.leftReedSwitchState
        cassie_out_lcm.pelvis.rightReedSwitchState = cassie_out.pelvis.rightReedSwitchState
        cassie_out_lcm.pelvis.vtmTemperature = cassie_out.pelvis.vtmTemperature

        cassie_out_lcm.isCalibrated = cassie_out.isCalibrated
        cassie_out_lcm.pelvis.vectorNav.temperature = cassie_out.pelvis.vectorNav.temperature

        self.copy_leg(cassie_out_lcm.leftLeg, cassie_out.leftLeg)
        self.copy_leg(cassie_out_lcm.rightLeg, cassie_out.rightLeg)
        return cassie_out_lcm

    def copy_leg(self, cassie_leg_out_lcm, cassie_leg_out):
        self.copy_elmo(cassie_leg_out_lcm.hipRollDrive, cassie_leg_out.hipRollDrive)
        self.copy_elmo(cassie_leg_out_lcm.hipYawDrive, cassie_leg_out.hipYawDrive)
        self.copy_elmo(cassie_leg_out_lcm.hipPitchDrive, cassie_leg_out.hipPitchDrive)
        self.copy_elmo(cassie_leg_out_lcm.kneeDrive, cassie_leg_out.kneeDrive)
        self.copy_elmo(cassie_leg_out_lcm.footDrive, cassie_leg_out.footDrive)
        cassie_leg_out_lcm.shinJoint.position = cassie_leg_out.shinJoint.position
        cassie_leg_out_lcm.shinJoint.velocity = cassie_leg_out.shinJoint.velocity
        cassie_leg_out_lcm.tarsusJoint.position = cassie_leg_out.tarsusJoint.position
        cassie_leg_out_lcm.tarsusJoint.velocity = cassie_leg_out.tarsusJoint.velocity
        cassie_leg_out_lcm.footJoint.position = cassie_leg_out.footJoint.position
        cassie_leg_out_lcm.footJoint.velocity = cassie_leg_out.footJoint.velocity
        cassie_leg_out_lcm.medullaCounter = cassie_leg_out.medullaCounter
        cassie_leg_out_lcm.medullaCpuLoad = cassie_leg_out.medullaCpuLoad
        cassie_leg_out_lcm.reedSwitchState = cassie_leg_out.reedSwitchState

    def copy_elmo(self, elmo_out_lcm, elmo_out):
        elmo_out_lcm.statusWord = elmo_out.statusWord
        elmo_out_lcm.position = elmo_out.position
        elmo_out_lcm.velocity = elmo_out.velocity
        elmo_out_lcm.torque = elmo_out.torque
        elmo_out_lcm.driveTemperature = elmo_out.driveTemperature
        elmo_out_lcm.dcLinkVoltage = elmo_out.dcLinkVoltage
        elmo_out_lcm.torqueLimit = elmo_out.torqueLimit
        elmo_out_lcm.gearRatio = elmo_out.gearRatio

    def randomize_sim_params(self):
        if self.dynamics_randomization:
            damp = self.default_damping

            pelvis_damp_range = [[damp[0], damp[0]],
                                 [damp[1], damp[1]],
                                 [damp[2], damp[2]],
                                 [damp[3], damp[3]],
                                 [damp[4], damp[4]],
                                 [damp[5], damp[5]]]  # 0->5

            hip_damp_range = [[damp[6] * self.damping_low, damp[6] * self.damping_high],
                              [damp[7] * self.damping_low, damp[7] * self.damping_high],
                              [damp[8] * self.damping_low, damp[8] * self.damping_high]]  # 6->8 and 19->21

            achilles_damp_range = [[damp[9] * self.damping_low, damp[9] * self.damping_high],
                                   [damp[10] * self.damping_low, damp[10] * self.damping_high],
                                   [damp[11] * self.damping_low, damp[11] * self.damping_high]]  # 9->11 and 22->24

            knee_damp_range = [[damp[12] * self.damping_low, damp[12] * self.damping_high]]  # 12 and 25
            shin_damp_range = [[damp[13] * self.damping_low, damp[13] * self.damping_high]]  # 13 and 26
            tarsus_damp_range = [[damp[14] * self.damping_low, damp[14] * self.damping_high]]  # 14 and 27

            heel_damp_range = [[damp[15], damp[15]]]  # 15 and 28
            fcrank_damp_range = [[damp[16] * self.damping_low, damp[16] * self.damping_high]]  # 16 and 29
            prod_damp_range = [[damp[17], damp[17]]]  # 17 and 30
            foot_damp_range = [[damp[18] * self.damping_low, damp[18] * self.damping_high]]  # 18 and 31

            side_damp = hip_damp_range + achilles_damp_range + knee_damp_range + shin_damp_range + tarsus_damp_range + heel_damp_range + fcrank_damp_range + prod_damp_range + foot_damp_range
            damp_range = pelvis_damp_range + side_damp + side_damp
            damp_noise = [np.random.uniform(a, b) for a, b in damp_range]

            m = self.default_mass
            pelvis_mass_range = [[self.mass_low * m[1], self.mass_high * m[1]]]  # 1
            hip_mass_range = [[self.mass_low * m[2], self.mass_high * m[2]],  # 2->4 and 14->16
                              [self.mass_low * m[3], self.mass_high * m[3]],
                              [self.mass_low * m[4], self.mass_high * m[4]]]

            achilles_mass_range = [[self.mass_low * m[5], self.mass_high * m[5]]]  # 5 and 17
            knee_mass_range = [[self.mass_low * m[6], self.mass_high * m[6]]]  # 6 and 18
            knee_spring_mass_range = [[self.mass_low * m[7], self.mass_high * m[7]]]  # 7 and 19
            shin_mass_range = [[self.mass_low * m[8], self.mass_high * m[8]]]  # 8 and 20
            tarsus_mass_range = [[self.mass_low * m[9], self.mass_high * m[9]]]  # 9 and 21
            heel_spring_mass_range = [[self.mass_low * m[10], self.mass_high * m[10]]]  # 10 and 22
            fcrank_mass_range = [[self.mass_low * m[11], self.mass_high * m[11]]]  # 11 and 23
            prod_mass_range = [[self.mass_low * m[12], self.mass_high * m[12]]]  # 12 and 24
            foot_mass_range = [[self.mass_low * m[13], self.mass_high * m[13]]]  # 13 and 25

            side_mass = hip_mass_range + achilles_mass_range \
                        + knee_mass_range + knee_spring_mass_range \
                        + shin_mass_range + tarsus_mass_range \
                        + heel_spring_mass_range + fcrank_mass_range \
                        + prod_mass_range + foot_mass_range

            mass_range = [[0, 0]] + pelvis_mass_range + side_mass + side_mass
            mass_noise = [np.random.uniform(a, b) for a, b in mass_range]

            delta = 0.0
            com_noise = [0, 0, 0] + [np.random.uniform(val - delta, val + delta) for val in self.default_ipos[3:]]

            fric_noise = []
            translational = np.random.uniform(self.fric_low, self.fric_high)
            torsional = np.random.uniform(1e-4, 5e-4)
            rolling = np.random.uniform(1e-4, 2e-4)
            for _ in range(int(len(self.default_fric) / 3)):
                fric_noise += [translational, torsional, rolling]

            self.simulator.set_dof_damping(np.clip(damp_noise, 0, None))
            self.simulator.set_body_mass(np.clip(mass_noise, 0, None))
            self.simulator.set_body_ipos(com_noise)
            self.simulator.set_geom_friction(np.clip(fric_noise, 0, None))
        else:
            self.simulator.set_body_mass(self.default_mass)
            self.simulator.set_body_ipos(self.default_ipos)
            self.simulator.set_dof_damping(self.default_damping)
            self.simulator.set_geom_friction(self.default_fric)

            if self.slope_rand:
                geom_plane = [np.random.uniform(-self.max_roll_incline, self.max_roll_incline),
                              np.random.uniform(-self.max_pitch_incline, self.max_pitch_incline), 0]
                quat_plane = euler2quat(z=geom_plane[2], y=geom_plane[1], x=geom_plane[0])
                geom_quat = list(quat_plane) + list(self.default_quat[4:])
                self.simulator.set_geom_quat(geom_quat)
            else:
                self.simulator.set_geom_quat(self.default_quat)

            if self.joint_rand:
                self.motor_encoder_noise = np.random.uniform(-self.encoder_noise, self.encoder_noise, size=10)
                self.joint_encoder_noise = np.random.uniform(-self.encoder_noise, self.encoder_noise, size=6)
            else:
                self.motor_encoder_noise = np.zeros(10)
                self.joint_encoder_noise = np.zeros(6)

    def free_sim(self):
        del self.simulator
        if self.visualize:
            del self.cassie_vis


def euler2quat(z=0, y=0, x=0):
    z = z / 2.0
    y = y / 2.0
    x = x / 2.0
    cz = math.cos(z)
    sz = math.sin(z)
    cy = math.cos(y)
    sy = math.sin(y)
    cx = math.cos(x)
    sx = math.sin(x)
    result = np.array([
        cx * cy * cz - sx * sy * sz,
        cx * sy * sz + cy * cz * sx,
        cx * cz * sy - sx * cy * sz,
        cx * cy * sz + sx * cz * sy])
    if result[0] < 0:
        result = -result
    return result
