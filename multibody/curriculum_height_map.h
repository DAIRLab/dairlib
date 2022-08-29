#include "boxy_height_map.h"

namespace dairlib::multibody {
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::MatrixXd;


class CurriculumHeightMap : BoxyHeightMap {
  public: 
    CurriculumHeightMap()=default;
    CurriculumHeightMap(); // TODO(hersh500): parameters here;
    static CurriculumHeightMap MakeMap();
  private:
     
    
}
