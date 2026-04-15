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
  /**
   * Match scan endpoints (in world frame) against `map`.
   *
   * @param[in,out] pose  current best-guess (x, y, theta); updated in place
   * @param map           the occupancy grid to match against
   * @param endpoints     scan endpoints in the sensor's local frame
   * @param iterations    max Gauss-Newton iterations (default 20)
   * @param translation_weight  weight for translation updates (default 1.0)
   * @param rotation_weight     weight for rotation updates (default 1.0)
   * @return              final sum of matched probabilities
   */
  static double match(Eigen::Vector3d & pose,
                      const OccupancyGridMap & map,
                      const std::vector<Eigen::Vector2d> & endpoints,
                      int iterations = 20,
                      double translation_weight = 1.0,
                      double rotation_weight = 1.0,
                      double covariance_scale = 1.0);

  /** Transform a local-frame point by pose (x, y, theta). */
  static Eigen::Vector2d transformPoint(const Eigen::Vector3d & pose,
                                        const Eigen::Vector2d & pt);
};

}  // namespace hector_mapping

#endif  // HECTOR_MAPPING__SCAN_MATCHER_HPP_
