# -*- coding: utf-8 -*-
#
# TARGET arch is: ['-I/usr/include/clang/6.0/include', '-Iinclude']
# WORD_SIZE is: 8
# POINTER_SIZE is: 8
# LONGDOUBLE_SIZE is: 16
#
import ctypes
import os
_dir_path = os.path.dirname(os.path.realpath(__file__))


_libraries = {}
_libraries['./libcassiemujoco.so'] = ctypes.CDLL(_dir_path + '/libcassiemujoco.so')
# if local wordsize is same as target, keep ctypes pointer function.
if ctypes.sizeof(ctypes.c_void_p) == 8:
    POINTER_T = ctypes.POINTER
else:
    # required to access _ctypes
    import _ctypes
    # Emulate a pointer class using the approriate c_int32/c_int64 type
    # The new class should have :
    # ['__module__', 'from_param', '_type_', '__dict__', '__weakref__', '__doc__']
    # but the class should be submitted to a unique instance for each base type
    # to that if A == B, POINTER_T(A) == POINTER_T(B)
    ctypes._pointer_t_type_cache = {}
    def POINTER_T(pointee):
        # a pointer should have the same length as LONG
        fake_ptr_base_type = ctypes.c_uint64 
        # specific case for c_void_p
        if pointee is None: # VOID pointer type. c_void_p.
            pointee = type(None) # ctypes.c_void_p # ctypes.c_ulong
            clsname = 'c_void'
        else:
            clsname = pointee.__name__
        if clsname in ctypes._pointer_t_type_cache:
            return ctypes._pointer_t_type_cache[clsname]
        # make template
        class _T(_ctypes._SimpleCData,):
            _type_ = 'L'
            _subtype_ = pointee
            def _sub_addr_(self):
                return self.value
            def __repr__(self):
                return '%s(%d)'%(clsname, self.value)
            def contents(self):
                raise TypeError('This is not a ctypes pointer.')
            def __init__(self, **args):
                raise TypeError('This is not a ctypes pointer. It is not instanciable.')
        _class = type('LP_%d_%s'%(8, clsname), (_T,),{}) 
        ctypes._pointer_t_type_cache[clsname] = _class
        return _class

c_int128 = ctypes.c_ubyte*16
c_uint128 = c_int128
void = None
if ctypes.sizeof(ctypes.c_longdouble) == 16:
    c_long_double_t = ctypes.c_longdouble
else:
    c_long_double_t = ctypes.c_ubyte*16



size_t = ctypes.c_uint64
socklen_t = ctypes.c_uint32
class struct_sockaddr(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('sa_family', ctypes.c_uint16),
    ('sa_data', ctypes.c_char * 14),
     ]

ssize_t = ctypes.c_int64
class struct_CassieCoreSim(ctypes.Structure):
    pass

cassie_core_sim_t = struct_CassieCoreSim
cassie_core_sim_alloc = _libraries['./libcassiemujoco.so'].cassie_core_sim_alloc
cassie_core_sim_alloc.restype = POINTER_T(struct_CassieCoreSim)
cassie_core_sim_alloc.argtypes = []
cassie_core_sim_copy = _libraries['./libcassiemujoco.so'].cassie_core_sim_copy
cassie_core_sim_copy.restype = None
cassie_core_sim_copy.argtypes = [POINTER_T(struct_CassieCoreSim), POINTER_T(struct_CassieCoreSim)]
cassie_core_sim_free = _libraries['./libcassiemujoco.so'].cassie_core_sim_free
cassie_core_sim_free.restype = None
cassie_core_sim_free.argtypes = [POINTER_T(struct_CassieCoreSim)]
cassie_core_sim_setup = _libraries['./libcassiemujoco.so'].cassie_core_sim_setup
cassie_core_sim_setup.restype = None
cassie_core_sim_setup.argtypes = [POINTER_T(struct_CassieCoreSim)]
class struct_c__SA_cassie_user_in_t(ctypes.Structure):
    pass

class struct_c__SA_cassie_out_t(ctypes.Structure):
    pass

class struct_c__SA_cassie_in_t(ctypes.Structure):
    pass

cassie_core_sim_step = _libraries['./libcassiemujoco.so'].cassie_core_sim_step
cassie_core_sim_step.restype = None
cassie_core_sim_step.argtypes = [POINTER_T(struct_CassieCoreSim), POINTER_T(struct_c__SA_cassie_user_in_t), POINTER_T(struct_c__SA_cassie_out_t), POINTER_T(struct_c__SA_cassie_in_t)]
class struct_c__SA_elmo_in_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('controlWord', ctypes.c_uint16),
    ('PADDING_0', ctypes.c_ubyte * 6),
    ('torque', ctypes.c_double),
     ]

elmo_in_t = struct_c__SA_elmo_in_t
class struct_c__SA_cassie_leg_in_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('hipRollDrive', elmo_in_t),
    ('hipYawDrive', elmo_in_t),
    ('hipPitchDrive', elmo_in_t),
    ('kneeDrive', elmo_in_t),
    ('footDrive', elmo_in_t),
     ]

cassie_leg_in_t = struct_c__SA_cassie_leg_in_t
class struct_c__SA_radio_in_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('channel', ctypes.c_int16 * 14),
     ]

radio_in_t = struct_c__SA_radio_in_t
class struct_c__SA_cassie_pelvis_in_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('radio', radio_in_t),
    ('sto', ctypes.c_bool),
    ('piezoState', ctypes.c_bool),
    ('piezoTone', ctypes.c_ubyte),
    ('PADDING_0', ctypes.c_ubyte),
     ]

cassie_pelvis_in_t = struct_c__SA_cassie_pelvis_in_t
struct_c__SA_cassie_in_t._pack_ = True # source:False
struct_c__SA_cassie_in_t._fields_ = [
    ('pelvis', cassie_pelvis_in_t),
    ('leftLeg', cassie_leg_in_t),
    ('rightLeg', cassie_leg_in_t),
]

cassie_in_t = struct_c__SA_cassie_in_t
pack_cassie_in_t = _libraries['./libcassiemujoco.so'].pack_cassie_in_t
pack_cassie_in_t.restype = None
pack_cassie_in_t.argtypes = [POINTER_T(struct_c__SA_cassie_in_t), POINTER_T(ctypes.c_ubyte)]
unpack_cassie_in_t = _libraries['./libcassiemujoco.so'].unpack_cassie_in_t
unpack_cassie_in_t.restype = None
unpack_cassie_in_t.argtypes = [POINTER_T(ctypes.c_ubyte), POINTER_T(struct_c__SA_cassie_in_t)]
DiagnosticCodes = ctypes.c_int16
class struct_c__SA_battery_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('dataGood', ctypes.c_bool),
    ('PADDING_0', ctypes.c_ubyte * 7),
    ('stateOfCharge', ctypes.c_double),
    ('voltage', ctypes.c_double * 12),
    ('current', ctypes.c_double),
    ('temperature', ctypes.c_double * 4),
     ]

battery_out_t = struct_c__SA_battery_out_t
class struct_c__SA_cassie_joint_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('position', ctypes.c_double),
    ('velocity', ctypes.c_double),
     ]

cassie_joint_out_t = struct_c__SA_cassie_joint_out_t
class struct_c__SA_elmo_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('statusWord', ctypes.c_uint16),
    ('PADDING_0', ctypes.c_ubyte * 6),
    ('position', ctypes.c_double),
    ('velocity', ctypes.c_double),
    ('torque', ctypes.c_double),
    ('driveTemperature', ctypes.c_double),
    ('dcLinkVoltage', ctypes.c_double),
    ('torqueLimit', ctypes.c_double),
    ('gearRatio', ctypes.c_double),
     ]

elmo_out_t = struct_c__SA_elmo_out_t
class struct_c__SA_cassie_leg_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('hipRollDrive', elmo_out_t),
    ('hipYawDrive', elmo_out_t),
    ('hipPitchDrive', elmo_out_t),
    ('kneeDrive', elmo_out_t),
    ('footDrive', elmo_out_t),
    ('shinJoint', cassie_joint_out_t),
    ('tarsusJoint', cassie_joint_out_t),
    ('footJoint', cassie_joint_out_t),
    ('medullaCounter', ctypes.c_ubyte),
    ('PADDING_0', ctypes.c_ubyte),
    ('medullaCpuLoad', ctypes.c_uint16),
    ('reedSwitchState', ctypes.c_bool),
    ('PADDING_1', ctypes.c_ubyte * 3),
     ]

cassie_leg_out_t = struct_c__SA_cassie_leg_out_t
class struct_c__SA_radio_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('radioReceiverSignalGood', ctypes.c_bool),
    ('receiverMedullaSignalGood', ctypes.c_bool),
    ('PADDING_0', ctypes.c_ubyte * 6),
    ('channel', ctypes.c_double * 16),
     ]

radio_out_t = struct_c__SA_radio_out_t
class struct_c__SA_target_pc_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('etherCatStatus', ctypes.c_int32 * 6),
    ('etherCatNotifications', ctypes.c_int32 * 21),
    ('PADDING_0', ctypes.c_ubyte * 4),
    ('taskExecutionTime', ctypes.c_double),
    ('overloadCounter', ctypes.c_uint32),
    ('PADDING_1', ctypes.c_ubyte * 4),
    ('cpuTemperature', ctypes.c_double),
     ]

target_pc_out_t = struct_c__SA_target_pc_out_t
class struct_c__SA_vectornav_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('dataGood', ctypes.c_bool),
    ('PADDING_0', ctypes.c_ubyte),
    ('vpeStatus', ctypes.c_uint16),
    ('PADDING_1', ctypes.c_ubyte * 4),
    ('pressure', ctypes.c_double),
    ('temperature', ctypes.c_double),
    ('magneticField', ctypes.c_double * 3),
    ('angularVelocity', ctypes.c_double * 3),
    ('linearAcceleration', ctypes.c_double * 3),
    ('orientation', ctypes.c_double * 4),
     ]

vectornav_out_t = struct_c__SA_vectornav_out_t
class struct_c__SA_cassie_pelvis_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('targetPc', target_pc_out_t),
    ('battery', battery_out_t),
    ('radio', radio_out_t),
    ('vectorNav', vectornav_out_t),
    ('medullaCounter', ctypes.c_ubyte),
    ('PADDING_0', ctypes.c_ubyte),
    ('medullaCpuLoad', ctypes.c_uint16),
    ('bleederState', ctypes.c_bool),
    ('leftReedSwitchState', ctypes.c_bool),
    ('rightReedSwitchState', ctypes.c_bool),
    ('PADDING_1', ctypes.c_ubyte),
    ('vtmTemperature', ctypes.c_double),
     ]

cassie_pelvis_out_t = struct_c__SA_cassie_pelvis_out_t
struct_c__SA_cassie_out_t._pack_ = True # source:False
struct_c__SA_cassie_out_t._fields_ = [
    ('pelvis', cassie_pelvis_out_t),
    ('leftLeg', cassie_leg_out_t),
    ('rightLeg', cassie_leg_out_t),
    ('isCalibrated', ctypes.c_bool),
    ('PADDING_0', ctypes.c_ubyte),
    ('messages', ctypes.c_int16 * 4),
    ('PADDING_1', ctypes.c_ubyte * 6),
]

cassie_out_t = struct_c__SA_cassie_out_t
pack_cassie_out_t = _libraries['./libcassiemujoco.so'].pack_cassie_out_t
pack_cassie_out_t.restype = None
pack_cassie_out_t.argtypes = [POINTER_T(struct_c__SA_cassie_out_t), POINTER_T(ctypes.c_ubyte)]
unpack_cassie_out_t = _libraries['./libcassiemujoco.so'].unpack_cassie_out_t
unpack_cassie_out_t.restype = None
unpack_cassie_out_t.argtypes = [POINTER_T(ctypes.c_ubyte), POINTER_T(struct_c__SA_cassie_out_t)]
struct_c__SA_cassie_user_in_t._pack_ = True # source:False
struct_c__SA_cassie_user_in_t._fields_ = [
    ('torque', ctypes.c_double * 10),
    ('telemetry', ctypes.c_int16 * 9),
    ('PADDING_0', ctypes.c_ubyte * 6),
]

cassie_user_in_t = struct_c__SA_cassie_user_in_t
pack_cassie_user_in_t = _libraries['./libcassiemujoco.so'].pack_cassie_user_in_t
pack_cassie_user_in_t.restype = None
pack_cassie_user_in_t.argtypes = [POINTER_T(struct_c__SA_cassie_user_in_t), POINTER_T(ctypes.c_ubyte)]
unpack_cassie_user_in_t = _libraries['./libcassiemujoco.so'].unpack_cassie_user_in_t
unpack_cassie_user_in_t.restype = None
unpack_cassie_user_in_t.argtypes = [POINTER_T(ctypes.c_ubyte), POINTER_T(struct_c__SA_cassie_user_in_t)]
class struct_cassie_sim(ctypes.Structure):
    pass

cassie_sim_t = struct_cassie_sim
class struct_cassie_vis(ctypes.Structure):
    pass

cassie_vis_t = struct_cassie_vis
class struct_cassie_state(ctypes.Structure):
    pass

cassie_state_t = struct_cassie_state
cassie_mujoco_init = _libraries['./libcassiemujoco.so'].cassie_mujoco_init
cassie_mujoco_init.restype = ctypes.c_bool
cassie_mujoco_init.argtypes = [POINTER_T(ctypes.c_char)]
cassie_cleanup = _libraries['./libcassiemujoco.so'].cassie_cleanup
cassie_cleanup.restype = None
cassie_cleanup.argtypes = []
cassie_sim_init = _libraries['./libcassiemujoco.so'].cassie_sim_init
cassie_sim_init.restype = POINTER_T(struct_cassie_sim)
cassie_sim_init.argtypes = [ctypes.c_char_p]
cassie_sim_duplicate = _libraries['./libcassiemujoco.so'].cassie_sim_duplicate
cassie_sim_duplicate.restype = POINTER_T(struct_cassie_sim)
cassie_sim_duplicate.argtypes = [POINTER_T(struct_cassie_sim)]
cassie_sim_copy = _libraries['./libcassiemujoco.so'].cassie_sim_copy
cassie_sim_copy.restype = None
cassie_sim_copy.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(struct_cassie_sim)]
cassie_sim_free = _libraries['./libcassiemujoco.so'].cassie_sim_free
cassie_sim_free.restype = None
cassie_sim_free.argtypes = [POINTER_T(struct_cassie_sim)]
cassie_sim_step_ethercat = _libraries['./libcassiemujoco.so'].cassie_sim_step_ethercat
cassie_sim_step_ethercat.restype = None
cassie_sim_step_ethercat.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(struct_c__SA_cassie_out_t), POINTER_T(struct_c__SA_cassie_in_t)]
cassie_sim_step = _libraries['./libcassiemujoco.so'].cassie_sim_step
cassie_sim_step.restype = None
cassie_sim_step.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(struct_c__SA_cassie_out_t), POINTER_T(struct_c__SA_cassie_user_in_t)]
class struct_c__SA_state_out_t(ctypes.Structure):
    pass

class struct_c__SA_pd_in_t(ctypes.Structure):
    pass

cassie_sim_step_pd = _libraries['./libcassiemujoco.so'].cassie_sim_step_pd
cassie_sim_step_pd.restype = None
cassie_sim_step_pd.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(struct_c__SA_state_out_t), POINTER_T(struct_c__SA_pd_in_t)]
cassie_sim_time = _libraries['./libcassiemujoco.so'].cassie_sim_time
cassie_sim_time.restype = POINTER_T(ctypes.c_double)
cassie_sim_time.argtypes = [POINTER_T(struct_cassie_sim)]
cassie_sim_qpos = _libraries['./libcassiemujoco.so'].cassie_sim_qpos
cassie_sim_qpos.restype = POINTER_T(ctypes.c_double)
cassie_sim_qpos.argtypes = [POINTER_T(struct_cassie_sim)]
cassie_sim_qvel = _libraries['./libcassiemujoco.so'].cassie_sim_qvel
cassie_sim_qvel.restype = POINTER_T(ctypes.c_double)
cassie_sim_qvel.argtypes = [POINTER_T(struct_cassie_sim)]
cassie_sim_mjmodel = _libraries['./libcassiemujoco.so'].cassie_sim_mjmodel
cassie_sim_mjmodel.restype = POINTER_T(None)
cassie_sim_mjmodel.argtypes = [POINTER_T(struct_cassie_sim)]
cassie_sim_mjdata = _libraries['./libcassiemujoco.so'].cassie_sim_mjdata
cassie_sim_mjdata.restype = POINTER_T(None)
cassie_sim_mjdata.argtypes = [POINTER_T(struct_cassie_sim)]
cassie_sim_check_obstacle_collision = _libraries['./libcassiemujoco.so'].cassie_sim_check_obstacle_collision
cassie_sim_check_obstacle_collision.restype = ctypes.c_bool
cassie_sim_check_obstacle_collision.argtypes = [POINTER_T(struct_cassie_sim)]
cassie_sim_check_self_collision = _libraries['./libcassiemujoco.so'].cassie_sim_check_self_collision
cassie_sim_check_self_collision.restype = ctypes.c_bool
cassie_sim_check_self_collision.argtypes = [POINTER_T(struct_cassie_sim)]
cassie_sim_foot_forces = _libraries['./libcassiemujoco.so'].cassie_sim_foot_forces
cassie_sim_foot_forces.restype = None
cassie_sim_foot_forces.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 12]
cassie_sim_foot_positions = _libraries['./libcassiemujoco.so'].cassie_sim_foot_positions
cassie_sim_foot_positions.restype = None
cassie_sim_foot_positions.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 6]
cassie_sim_apply_force = _libraries['./libcassiemujoco.so'].cassie_sim_apply_force
cassie_sim_apply_force.restype = None
cassie_sim_apply_force.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 6, ctypes.c_int32]
cassie_sim_clear_forces = _libraries['./libcassiemujoco.so'].cassie_sim_clear_forces
cassie_sim_clear_forces.restype = None
cassie_sim_clear_forces.argtypes = [POINTER_T(struct_cassie_sim)]
cassie_sim_hold = _libraries['./libcassiemujoco.so'].cassie_sim_hold
cassie_sim_hold.restype = None
cassie_sim_hold.argtypes = [POINTER_T(struct_cassie_sim)]
cassie_sim_release = _libraries['./libcassiemujoco.so'].cassie_sim_release
cassie_sim_release.restype = None
cassie_sim_release.argtypes = [POINTER_T(struct_cassie_sim)]
cassie_sim_radio = _libraries['./libcassiemujoco.so'].cassie_sim_radio
cassie_sim_radio.restype = None
cassie_sim_radio.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_double * 16]
cassie_vis_init = _libraries['./libcassiemujoco.so'].cassie_vis_init
cassie_vis_init.restype = POINTER_T(struct_cassie_vis)
cassie_vis_init.argtypes = [POINTER_T(struct_cassie_sim), ctypes.c_char_p]
cassie_vis_close = _libraries['./libcassiemujoco.so'].cassie_vis_close
cassie_vis_close.restype = None
cassie_vis_close.argtypes = [POINTER_T(struct_cassie_vis)]
cassie_vis_free = _libraries['./libcassiemujoco.so'].cassie_vis_free
cassie_vis_free.restype = None
cassie_vis_free.argtypes = [POINTER_T(struct_cassie_vis)]
cassie_vis_draw = _libraries['./libcassiemujoco.so'].cassie_vis_draw
cassie_vis_draw.restype = ctypes.c_bool
cassie_vis_draw.argtypes = [POINTER_T(struct_cassie_vis), POINTER_T(struct_cassie_sim)]
cassie_vis_valid = _libraries['./libcassiemujoco.so'].cassie_vis_valid
cassie_vis_valid.restype = ctypes.c_bool
cassie_vis_valid.argtypes = [POINTER_T(struct_cassie_vis)]
cassie_vis_paused = _libraries['./libcassiemujoco.so'].cassie_vis_paused
cassie_vis_paused.restype = ctypes.c_bool
cassie_vis_paused.argtypes = [POINTER_T(struct_cassie_vis)]
cassie_state_alloc = _libraries['./libcassiemujoco.so'].cassie_state_alloc
cassie_state_alloc.restype = POINTER_T(struct_cassie_state)
cassie_state_alloc.argtypes = []
cassie_state_duplicate = _libraries['./libcassiemujoco.so'].cassie_state_duplicate
cassie_state_duplicate.restype = POINTER_T(struct_cassie_state)
cassie_state_duplicate.argtypes = [POINTER_T(struct_cassie_state)]
cassie_state_copy = _libraries['./libcassiemujoco.so'].cassie_state_copy
cassie_state_copy.restype = None
cassie_state_copy.argtypes = [POINTER_T(struct_cassie_state), POINTER_T(struct_cassie_state)]
cassie_state_free = _libraries['./libcassiemujoco.so'].cassie_state_free
cassie_state_free.restype = None
cassie_state_free.argtypes = [POINTER_T(struct_cassie_state)]
cassie_state_time = _libraries['./libcassiemujoco.so'].cassie_state_time
cassie_state_time.restype = POINTER_T(ctypes.c_double)
cassie_state_time.argtypes = [POINTER_T(struct_cassie_state)]
cassie_state_qpos = _libraries['./libcassiemujoco.so'].cassie_state_qpos
cassie_state_qpos.restype = POINTER_T(ctypes.c_double)
cassie_state_qpos.argtypes = [POINTER_T(struct_cassie_state)]
cassie_state_qvel = _libraries['./libcassiemujoco.so'].cassie_state_qvel
cassie_state_qvel.restype = POINTER_T(ctypes.c_double)
cassie_state_qvel.argtypes = [POINTER_T(struct_cassie_state)]
cassie_get_state = _libraries['./libcassiemujoco.so'].cassie_get_state
cassie_get_state.restype = None
cassie_get_state.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(struct_cassie_state)]
cassie_set_state = _libraries['./libcassiemujoco.so'].cassie_set_state
cassie_set_state.restype = None
cassie_set_state.argtypes = [POINTER_T(struct_cassie_sim), POINTER_T(struct_cassie_state)]
class struct_c__SA_pd_motor_in_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('torque', ctypes.c_double * 5),
    ('pTarget', ctypes.c_double * 5),
    ('dTarget', ctypes.c_double * 5),
    ('pGain', ctypes.c_double * 5),
    ('dGain', ctypes.c_double * 5),
     ]

pd_motor_in_t = struct_c__SA_pd_motor_in_t
class struct_c__SA_pd_task_in_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('torque', ctypes.c_double * 6),
    ('pTarget', ctypes.c_double * 6),
    ('dTarget', ctypes.c_double * 6),
    ('pGain', ctypes.c_double * 6),
    ('dGain', ctypes.c_double * 6),
     ]

pd_task_in_t = struct_c__SA_pd_task_in_t
class struct_c__SA_pd_leg_in_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('taskPd', pd_task_in_t),
    ('motorPd', pd_motor_in_t),
     ]

pd_leg_in_t = struct_c__SA_pd_leg_in_t
struct_c__SA_pd_in_t._pack_ = True # source:False
struct_c__SA_pd_in_t._fields_ = [
    ('leftLeg', pd_leg_in_t),
    ('rightLeg', pd_leg_in_t),
    ('telemetry', ctypes.c_double * 9),
]

pd_in_t = struct_c__SA_pd_in_t
pack_pd_in_t = _libraries['./libcassiemujoco.so'].pack_pd_in_t
pack_pd_in_t.restype = None
pack_pd_in_t.argtypes = [POINTER_T(struct_c__SA_pd_in_t), POINTER_T(ctypes.c_ubyte)]
unpack_pd_in_t = _libraries['./libcassiemujoco.so'].unpack_pd_in_t
unpack_pd_in_t.restype = None
unpack_pd_in_t.argtypes = [POINTER_T(ctypes.c_ubyte), POINTER_T(struct_c__SA_pd_in_t)]
class struct_PdInput(ctypes.Structure):
    pass

pd_input_t = struct_PdInput
pd_input_alloc = _libraries['./libcassiemujoco.so'].pd_input_alloc
pd_input_alloc.restype = POINTER_T(struct_PdInput)
pd_input_alloc.argtypes = []
pd_input_copy = _libraries['./libcassiemujoco.so'].pd_input_copy
pd_input_copy.restype = None
pd_input_copy.argtypes = [POINTER_T(struct_PdInput), POINTER_T(struct_PdInput)]
pd_input_free = _libraries['./libcassiemujoco.so'].pd_input_free
pd_input_free.restype = None
pd_input_free.argtypes = [POINTER_T(struct_PdInput)]
pd_input_setup = _libraries['./libcassiemujoco.so'].pd_input_setup
pd_input_setup.restype = None
pd_input_setup.argtypes = [POINTER_T(struct_PdInput)]
pd_input_step = _libraries['./libcassiemujoco.so'].pd_input_step
pd_input_step.restype = None
pd_input_step.argtypes = [POINTER_T(struct_PdInput), POINTER_T(struct_c__SA_pd_in_t), POINTER_T(struct_c__SA_cassie_out_t), POINTER_T(struct_c__SA_cassie_user_in_t)]
class struct_c__SA_state_battery_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('stateOfCharge', ctypes.c_double),
    ('current', ctypes.c_double),
     ]

state_battery_out_t = struct_c__SA_state_battery_out_t
class struct_c__SA_state_foot_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('position', ctypes.c_double * 3),
    ('orientation', ctypes.c_double * 4),
    ('footRotationalVelocity', ctypes.c_double * 3),
    ('footTranslationalVelocity', ctypes.c_double * 3),
    ('toeForce', ctypes.c_double * 3),
    ('heelForce', ctypes.c_double * 3),
     ]

state_foot_out_t = struct_c__SA_state_foot_out_t
class struct_c__SA_state_joint_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('position', ctypes.c_double * 6),
    ('velocity', ctypes.c_double * 6),
     ]

state_joint_out_t = struct_c__SA_state_joint_out_t
class struct_c__SA_state_motor_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('position', ctypes.c_double * 10),
    ('velocity', ctypes.c_double * 10),
    ('torque', ctypes.c_double * 10),
     ]

state_motor_out_t = struct_c__SA_state_motor_out_t
class struct_c__SA_state_pelvis_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('position', ctypes.c_double * 3),
    ('orientation', ctypes.c_double * 4),
    ('rotationalVelocity', ctypes.c_double * 3),
    ('translationalVelocity', ctypes.c_double * 3),
    ('translationalAcceleration', ctypes.c_double * 3),
    ('externalMoment', ctypes.c_double * 3),
    ('externalForce', ctypes.c_double * 3),
     ]

state_pelvis_out_t = struct_c__SA_state_pelvis_out_t
class struct_c__SA_state_radio_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('channel', ctypes.c_double * 16),
    ('signalGood', ctypes.c_bool),
    ('PADDING_0', ctypes.c_ubyte * 7),
     ]

state_radio_out_t = struct_c__SA_state_radio_out_t
class struct_c__SA_state_terrain_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('height', ctypes.c_double),
    ('slope', ctypes.c_double * 2),
     ]

state_terrain_out_t = struct_c__SA_state_terrain_out_t
struct_c__SA_state_out_t._pack_ = True # source:False
struct_c__SA_state_out_t._fields_ = [
    ('pelvis', state_pelvis_out_t),
    ('leftFoot', state_foot_out_t),
    ('rightFoot', state_foot_out_t),
    ('terrain', state_terrain_out_t),
    ('motor', state_motor_out_t),
    ('joint', state_joint_out_t),
    ('radio', state_radio_out_t),
    ('battery', state_battery_out_t),
]

state_out_t = struct_c__SA_state_out_t
pack_state_out_t = _libraries['./libcassiemujoco.so'].pack_state_out_t
pack_state_out_t.restype = None
pack_state_out_t.argtypes = [POINTER_T(struct_c__SA_state_out_t), POINTER_T(ctypes.c_ubyte)]
unpack_state_out_t = _libraries['./libcassiemujoco.so'].unpack_state_out_t
unpack_state_out_t.restype = None
unpack_state_out_t.argtypes = [POINTER_T(ctypes.c_ubyte), POINTER_T(struct_c__SA_state_out_t)]
class struct_StateOutput(ctypes.Structure):
    pass

state_output_t = struct_StateOutput
state_output_alloc = _libraries['./libcassiemujoco.so'].state_output_alloc
state_output_alloc.restype = POINTER_T(struct_StateOutput)
state_output_alloc.argtypes = []
state_output_copy = _libraries['./libcassiemujoco.so'].state_output_copy
state_output_copy.restype = None
state_output_copy.argtypes = [POINTER_T(struct_StateOutput), POINTER_T(struct_StateOutput)]
state_output_free = _libraries['./libcassiemujoco.so'].state_output_free
state_output_free.restype = None
state_output_free.argtypes = [POINTER_T(struct_StateOutput)]
state_output_setup = _libraries['./libcassiemujoco.so'].state_output_setup
state_output_setup.restype = None
state_output_setup.argtypes = [POINTER_T(struct_StateOutput)]
state_output_step = _libraries['./libcassiemujoco.so'].state_output_step
state_output_step.restype = None
state_output_step.argtypes = [POINTER_T(struct_StateOutput), POINTER_T(struct_c__SA_cassie_out_t), POINTER_T(struct_c__SA_state_out_t)]
class struct_c__SA_packet_header_info_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('seq_num_out', ctypes.c_char),
    ('seq_num_in_last', ctypes.c_char),
    ('delay', ctypes.c_char),
    ('seq_num_in_diff', ctypes.c_char),
     ]

packet_header_info_t = struct_c__SA_packet_header_info_t
process_packet_header = _libraries['./libcassiemujoco.so'].process_packet_header
process_packet_header.restype = None
process_packet_header.argtypes = [POINTER_T(struct_c__SA_packet_header_info_t), POINTER_T(ctypes.c_ubyte), POINTER_T(ctypes.c_ubyte)]
udp_init_host = _libraries['./libcassiemujoco.so'].udp_init_host
udp_init_host.restype = ctypes.c_int32
udp_init_host.argtypes = [POINTER_T(ctypes.c_char), POINTER_T(ctypes.c_char)]
udp_init_client = _libraries['./libcassiemujoco.so'].udp_init_client
udp_init_client.restype = ctypes.c_int32
udp_init_client.argtypes = [POINTER_T(ctypes.c_char), POINTER_T(ctypes.c_char), POINTER_T(ctypes.c_char), POINTER_T(ctypes.c_char)]
udp_close = _libraries['./libcassiemujoco.so'].udp_close
udp_close.restype = None
udp_close.argtypes = [ctypes.c_int32]
get_newest_packet = _libraries['./libcassiemujoco.so'].get_newest_packet
get_newest_packet.restype = ssize_t
get_newest_packet.argtypes = [ctypes.c_int32, POINTER_T(None), size_t, POINTER_T(struct_sockaddr), POINTER_T(ctypes.c_uint32)]
wait_for_packet = _libraries['./libcassiemujoco.so'].wait_for_packet
wait_for_packet.restype = ssize_t
wait_for_packet.argtypes = [ctypes.c_int32, POINTER_T(None), size_t, POINTER_T(struct_sockaddr), POINTER_T(ctypes.c_uint32)]
send_packet = _libraries['./libcassiemujoco.so'].send_packet
send_packet.restype = ssize_t
send_packet.argtypes = [ctypes.c_int32, POINTER_T(None), size_t, POINTER_T(struct_sockaddr), socklen_t]
__all__ = \
    ['cassie_pelvis_in_t', 'struct_StateOutput', 'cassie_state_t',
    'cassie_sim_check_self_collision', 'cassie_vis_free',
    'cassie_in_t', 'state_terrain_out_t', 'struct_c__SA_pd_leg_in_t',
    'cassie_state_free', 'struct_c__SA_state_battery_out_t',
    'elmo_in_t', 'state_joint_out_t', 'send_packet',
    'cassie_pelvis_out_t', 'cassie_cleanup',
    'struct_c__SA_state_radio_out_t', 'cassie_vis_valid',
    'pd_input_setup', 'pd_leg_in_t', 'cassie_mujoco_init',
    'cassie_state_copy', 'cassie_core_sim_setup', 'battery_out_t',
    'cassie_sim_hold', 'struct_CassieCoreSim', 'cassie_core_sim_step',
    'pack_cassie_out_t', 'cassie_out_t', 'radio_in_t',
    'unpack_cassie_out_t', 'struct_c__SA_pd_task_in_t',
    'struct_PdInput', 'udp_init_client', 'pd_motor_in_t',
    'cassie_sim_t', 'cassie_core_sim_alloc', 'get_newest_packet',
    'size_t', 'struct_c__SA_vectornav_out_t',
    'struct_c__SA_pd_motor_in_t', 'cassie_get_state',
    'state_battery_out_t', 'struct_c__SA_state_pelvis_out_t',
    'cassie_state_qpos', 'cassie_state_qvel', 'state_radio_out_t',
    'struct_c__SA_pd_in_t', 'udp_close', 'state_output_free',
    'cassie_core_sim_free', 'pd_task_in_t', 'packet_header_info_t',
    'pd_in_t', 'struct_cassie_vis', 'struct_c__SA_elmo_out_t',
    'pack_pd_in_t', 'struct_c__SA_radio_out_t', 'pd_input_alloc',
    'DiagnosticCodes', 'unpack_state_out_t', 'target_pc_out_t',
    'cassie_sim_duplicate', 'cassie_state_alloc', 'cassie_sim_init',
    'struct_c__SA_cassie_user_in_t', 'struct_c__SA_radio_in_t',
    'socklen_t', 'cassie_vis_init', 'state_out_t',
    'struct_c__SA_cassie_in_t', 'pd_input_free', 'state_output_alloc',
    'struct_c__SA_cassie_leg_out_t',
    'struct_c__SA_cassie_pelvis_in_t', 'unpack_pd_in_t',
    'cassie_user_in_t', 'cassie_sim_clear_forces', 'cassie_vis_t',
    'struct_c__SA_target_pc_out_t', 'pd_input_step',
    'cassie_set_state', 'struct_c__SA_battery_out_t',
    'vectornav_out_t', 'struct_c__SA_packet_header_info_t',
    'cassie_sim_step_pd', 'struct_sockaddr', 'cassie_vis_draw',
    'cassie_core_sim_copy', 'unpack_cassie_in_t', 'struct_cassie_sim',
    'unpack_cassie_user_in_t', 'cassie_sim_step', 'udp_init_host',
    'state_motor_out_t', 'cassie_core_sim_t', 'pack_state_out_t',
    'cassie_sim_mjdata', 'state_output_setup', 'cassie_sim_mjmodel',
    'state_foot_out_t', 'state_output_t', 'cassie_sim_time',
    'cassie_sim_step_ethercat', 'cassie_sim_check_obstacle_collision',
    'elmo_out_t', 'pack_cassie_in_t', 'cassie_sim_apply_force',
    'cassie_leg_out_t', 'wait_for_packet',
    'struct_c__SA_cassie_leg_in_t', 'struct_c__SA_state_joint_out_t',
    'process_packet_header', 'cassie_sim_release', 'cassie_sim_foot_forces', 
    'cassie_sim_foot_positions', 'struct_c__SA_state_foot_out_t',
    'pd_input_t', 'pack_cassie_user_in_t', 'cassie_state_duplicate',
    'state_pelvis_out_t', 'struct_c__SA_state_terrain_out_t',
    'cassie_sim_free', 'ssize_t', 'state_output_copy',
    'cassie_sim_radio', 'cassie_vis_close', 'cassie_vis_paused', 'radio_out_t',
    'state_output_step', 'struct_c__SA_state_motor_out_t',
    'struct_cassie_state', 'cassie_state_time', 'cassie_sim_qvel',
    'cassie_sim_qpos', 'struct_c__SA_elmo_in_t', 'cassie_joint_out_t',
    'cassie_leg_in_t', 'struct_c__SA_cassie_joint_out_t',
    'struct_c__SA_state_out_t', 'struct_c__SA_cassie_pelvis_out_t',
    'pd_input_copy', 'cassie_sim_copy', 'struct_c__SA_cassie_out_t']
