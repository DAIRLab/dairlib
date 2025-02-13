import numpy as np

from pydairlib.id_mpc import (
    LoadIDMPCParamsFromYaml,
    WalkingReferenceSystem,
    MakeCassieDynamics,
    MakeCassieGaitParams
)

from pydairlib.systems import OutputVector


def print_ref(ref):
    for t, c in zip(ref.knot_times, ref.active_contacts):
        print(f'{t:.3f}: {c}')

    for t, c in zip(ref.knot_times, ref.touchdown_ee_names):
        print(f'{t:.3f}: {c}')

    for t in ref.knot_times:
        print(f'{t:.3f}: {ref.task_space_trajs["swing_foot"].value(t).ravel()[-1]}')

def main():
    params = LoadIDMPCParamsFromYaml(
        "examples/id_mpc/gains/mpc_gains_walking.yaml")
    dynamics = MakeCassieDynamics()
    plant_context = dynamics.MakeContext()
    ref_gen = WalkingReferenceSystem(
        dynamics, plant_context, MakeCassieGaitParams(params))

    q = np.array([1., 0., 0., 0., 0., 0., 0.75, 0.0924283, 0, 0.839764, -1.91927, 2.14352, -1.9375, -0.0924283, 0, 0.839764, -1.91927, 2.14352, -1.9375])
    u = np.array([-2.03951, 2.04169, 0.906345, -0.861539, -5.96077, -6.16527, 45.7984, 45.6304, -3.48936, -3.52897])
    v = np.zeros((dynamics.nv(),))

    state = OutputVector(dynamics.nq(), dynamics.nv(), dynamics.nu())
    state.SetPositions(q)
    state.SetVelocities(v)
    state.SetEfforts(u)
    state.set_timestamp(0.0)

    context = ref_gen.CreateDefaultContext()
    ref_gen.get_input_port_state().FixValue(context, state)
    ref_gen.get_input_port_vdes().FixValue(context, np.array([0.5, 0]))
    ref_gen.CalcForcedUnrestrictedUpdate(context, context.get_mutable_state())

    reference = ref_gen.get_output_port().Eval(context)
    print_ref(reference)
    print()

    state.set_timestamp(0.1731)
    ref_gen.get_input_port_state().FixValue(context, state)
    ref_gen.CalcForcedUnrestrictedUpdate(context, context.get_mutable_state())

    reference = ref_gen.get_output_port().Eval(context)
    print_ref(reference)



if __name__ == '__main__':
    main()
