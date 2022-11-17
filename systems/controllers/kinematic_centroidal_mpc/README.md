This directory contains the necessary LeafSystems (
see `drake::systems::LeafSystem`) to generate kinematic centroidal reference
trajectories based on the current robot state.

The generation is split up into two LeafSystems:
- KinematicCentroidalReferenceGenerator
  - this class is responsible for outputting a `KCReferenceVector` which contains reference trajectories for:
    - force
    - generalized positions
    - generalized velocities
    - center of mass
    - boolean contact activation
  - we separate this reference generator into a separate class in order to support multiple reference generators
  - inputs:
    - target velocity (target center of mass translation velocity as a trajectory)
    - (potentially) gait specification, currently fixed
    - (potentially) gait durations
    - current robot state (used in computing the reference trajectories)
- KinematicCentroidalMPC
  - this class solves the KinematicCentroidal planning problem
  - inputs:
    - current robot state (used in initial state constraint)
    - `KCReferenceVector` (target reference)
  - output:
    - `lcmt_saved_traj` (contains state and contact force trajectories)
    - 