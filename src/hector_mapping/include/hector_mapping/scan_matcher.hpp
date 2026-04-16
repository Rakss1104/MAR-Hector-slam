#ifndef HECTOR_MAPPING__SCAN_MATCHER_HPP_
#define HECTOR_MAPPING__SCAN_MATCHER_HPP_

#include <vector>
#include <Eigen/Core>
#include "hector_mapping/occupancy_grid_map.hpp"

namespace hector_mapping
{

/**
 * Gauss-Newton scan matcher that aligns a set of 2-D scan endpoints
 * against an existing OccupancyGridMap by maximising the sum of
 * interpolated occupancy probabilities.
 *
 * Follows the original Hector SLAM paper:
 *   Kohlbrecher et al., "A Flexible and Scalable SLAM System with
 *   Full 3D Motion Estimation", SSRR 2011.
 */
class ScanMatcher
{
public:
  /** Transform a local-frame point by pose (x, y, theta). */
  static Eigen::Vector2d transformPoint(const Eigen::Vector3d & pose,
                                        const Eigen::Vector2d & pt);

  /**
   * Match scan endpoints (in world frame) against `map`.
   */
  static double match(Eigen::Vector3d & pose,
                      const OccupancyGridMap & map,
                      const std::vector<Eigen::Vector2d> & endpoints,
                      int iterations = 20);
};

}  // namespace hector_mapping

#endif  // HECTOR_MAPPING__SCAN_MATCHER_HPP_
