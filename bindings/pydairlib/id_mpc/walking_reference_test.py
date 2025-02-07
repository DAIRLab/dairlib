from pydairlib.id_mpc import (
    LoadIDMPCParamsFromYaml,
    WalkingReferenceSystem,
    MakeCassieDynamics,
    MakeCassieGaitParams
)

from pydairlib.systems import OutputVector


def main():
    params = LoadIDMPCParamsFromYaml(
        "examples/id_mpc/gains/mpc_gains_walking.yaml")
    dynamics = MakeCassieDynamics()
    plant_context = dynamics.MakeContext()
    ref_gen = WalkingReferenceSystem(
        dynamics, plant_context, MakeCassieGaitParams(params))




if __name__ == '__main__':
    main()
