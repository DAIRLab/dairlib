#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/timestamped_vector.h"

#include "dairlib/lcmt_cassie_state.hpp"
#include "dairlib/lcmt_cassie_input.hpp"
#include "dairlib/lcmt_cassie_pd_config.hpp"
#include "cassie_controller_lcm.h"

namespace dairlib{

using dairlib::systems::TimestampedVector;
using Eigen::VectorXd;

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to Cassie. The classes in this file are based on
/// acrobot_lcm.h

/// Receives the output of an LcmSubsriberSystem that subsribes to the
/// Cassie state channel with LCM type lcmt_cassie_state, and outputs the
/// Cassie states as a Context.
class CassieStateReceiver : public systems::LeafSystem<double> {
 public:
  CassieStateReceiver(RigidBodyTree<double>& tree);


 private:
  void CopyStateOut(const systems::Context<double>& context,
                    TimestampedVector<double>* output) const;
  const RigidBodyTree<double>* tree_;
  const map<string, int> positionIndexMap_;
  const map<string, int> velocityIndexMap_;

}
namespace drake {
class CassieInputReceiver : public systems::LeafSystem<double> {
 public:
  CassieInputReceiver();


 private:
  void CopyInputOut(const systems::Context<double>& context,
                    TimestampedVector<double>* output) const;
  const RigidBodyTree<double>* tree_;
};

/// Receives the output of a controller controller, and outputs it as an LCM
/// message with type lcm_cassie_u. Its output port is usually connected to
/// an LcmPublisherSystem to publish the messages it generates.
class CassieCommandSender : public systems::LeafSystem<double> {
 public:
  CassieCommandSender();

 private:
  void OutputCommand(const systems::Context<double>& context,
                     dairlib::lcmt_cassie_input* output) const;
  const RigidBodyTree<double>* tree_;
};

class CassieStateSender : public systems::LeafSystem<double> {
 public:
  CassieStateSender();


 private:
  void OutputState(const systems::Context<double>& context,
                     dairlib::lcmt_cassie_state* output) const;
  const RigidBodyTree<double>* tree_;
};


/// Implementation of TimestampedVector to store, set, and get PD configuration:
/// desired position/velocity and gains (kp/kd)
class CassiePDConfig : public TimestampedVector<double> {
  public:
    CassiePDConfig(int num_joints);

    //Getters and setters
    VectorXd getDesiredPosition() const {return desired_position_;};
    VectorXd getDesiredVelocity() const {return desired_velocity_;};
    VectorXd getKp() const {return kp_;};
    VectorXd getKd() const {return kd_;};
    double getDesiredPosition(int index) const {return desired_position_(index);};
    double getDesiredVelocity(int index) const {return desired_velocity_(index);};
    double getKp(int index) const {return kp_(index);};
    double getKd(int index) const {return kd_(index);};
    void setDesiredPosition(VectorXd desired_position);
    void setDesiredVelocity(VectorXd desired_velocity);
    void setKp(VectorXd kp);
    void setKd(VectorXd kd);
    void setDesiredPosition(int index, double position);
    void setDesiredVelocity(int index, double velocity);
    void setKp(int index, double kp);
    void setKd(int index, double kd);
  private:
    CassiePDConfig* DoClone() const override;

    VectorXd desired_position_;
    VectorXd desired_velocity_;
    VectorXd kp_;
    VectorXd kd_;
    int num_joints_;
};

/// Receives the output of an LcmSubsriberSystem that subsribes to the
/// Cassie PD configuration channel with LCM type lcmt_cassie_pd_config, 
/// and outputs the CassiePDConfig as Context
class CassiePDConfigReceiver : public systems::LeafSystem<double> {
 public:
  CassiePDConfigReceiver();


 private:
  void CopyConfig(const systems::Context<double>& context,
                    CassiePDConfig* output) const;
};
}
