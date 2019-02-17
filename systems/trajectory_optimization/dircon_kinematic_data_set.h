#pragma once

#include <memory>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "systems/trajectory_optimization/dircon_kinematic_data.h"

namespace dairlib {

template <typename T>
class DirconKinematicDataSet {
 public:
  DirconKinematicDataSet(const drake::multibody::MultibodyPlant<T>& plant,
                         std::vector<DirconKinematicData<T>*>* constraints);

  void updateData(const drake::systems::Context<T>& context,
                  const drake::VectorX<T>& forces);

  drake::VectorX<T> getC();
  drake::VectorX<T> getCDot();
  drake::MatrixX<T> getJ();
  drake::VectorX<T> getJdotv();
  drake::VectorX<T> getCDDot();
  drake::VectorX<T> getVDot();
  drake::VectorX<T> getXDot();

  DirconKinematicData<T>* getConstraint(int index);

  int getNumConstraintObjects();
  int countConstraints();

 private:
    const drake::multibody::MultibodyPlant<T>& plant_;
    std::vector<DirconKinematicData<T>*>* constraints_;
    int num_positions_;
    int num_velocities_;
    int constraint_count_;
    drake::VectorX<T> c_;
    drake::VectorX<T> cdot_;
    drake::MatrixX<T> J_;
    drake::VectorX<T> Jdotv_;
    drake::VectorX<T> cddot_;
    drake::VectorX<T> vdot_;
    drake::VectorX<T> xdot_;
    drake::MatrixX<T> M_;
    drake::VectorX<T> right_hand_side_;
};
}  // namespace dairlib
