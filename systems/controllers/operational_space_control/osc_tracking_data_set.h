#pragma once

#include "systems/controllers/operational_space_control/osc_tracking_data.h"
#include <Eigen/Dense>
#include <vector>

namespace dairlib {
namespace systems {
namespace controllers {

// OscTrackingDataSet is just a collection of OscTrackingData
class OscTrackingDataSet {
 public:
  // OscTrackingDataSet();

  OscTrackingDataSet() {}  // Default constructor

  void AddTrackingData(OscTrackingData* tracking_data);

  std::vector<OscTrackingData*> GetAllTrackingData(){
    return tracking_data_vec_;
  }
  OscTrackingData* GetTrackingDataByIndex(int index){
    return tracking_data_vec_[index];
  }

 private:
  std::vector<OscTrackingData*> tracking_data_vec_;
  int num_tracking_data_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
