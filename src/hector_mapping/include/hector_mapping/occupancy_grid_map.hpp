#ifndef HECTOR_MAPPING__OCCUPANCY_GRID_MAP_HPP_
#define HECTOR_MAPPING__OCCUPANCY_GRID_MAP_HPP_

#include <vector>
#include <cmath>
#include <Eigen/Core>

namespace hector_mapping
{

/**
 * Probability-based occupancy grid that stores log-odds values internally
 * and can expose both a [0,1] probability view and a gradient (for scan
 * matching optimisation).
 *
 * The map is square with side length `map_size` cells and a given resolution
 * (metres per cell).  The world-frame origin sits at the cell
 * (map_size * start_x, map_size * start_y).
 */
class OccupancyGridMap
{
public:
  OccupancyGridMap(int map_size, double resolution,
                   double start_x = 0.5, double start_y = 0.5,
                   double update_free = 0.4, double update_occ = 0.9);

  /* ── World ↔ Grid conversions ──────────────────────────────────── */
  Eigen::Vector2i worldToGrid(const Eigen::Vector2d & world) const;
  Eigen::Vector2d gridToWorld(const Eigen::Vector2i & grid) const;
  bool isInside(int gx, int gy) const;

  /* ── Map access ────────────────────────────────────────────────── */
  double getProbability(int gx, int gy) const;
  void   updateSetOccupied(int gx, int gy);
  void   updateSetFree(int gx, int gy);

  /* ── Bresenham ray trace (mark free along ray, occupied at end) ─ */
  void updateByScan(const Eigen::Vector2d & sensor_world,
                    const std::vector<Eigen::Vector2d> & endpoints_world);

  /* ── Bilinear-interpolated probability + gradient (for matcher) ─ */
  double getInterpolatedProbability(double wx, double wy) const;
  Eigen::Vector3d getInterpolatedProbabilityAndGradient(
      double wx, double wy) const;   // (val, dval/dwx, dval/dwy)

  /* ── Accessors ─────────────────────────────────────────────────── */
  int    size()       const { return map_size_; }
  double resolution() const { return resolution_; }
  double originX()    const { return origin_x_; }
  double originY()    const { return origin_y_; }
  const std::vector<double> & logOdds() const { return log_odds_; }

private:
  int    map_size_;
  double resolution_;
  double origin_x_, origin_y_;          // world coords of grid (0,0)
  double log_free_, log_occ_;           // log-odds update amounts
  std::vector<double> log_odds_;        // flat row-major

  int idx(int gx, int gy) const { return gy * map_size_ + gx; }

  static double logOddsFromProb(double p);
  static double probFromLogOdds(double l);
};

}  // namespace hector_mapping

#endif  // HECTOR_MAPPING__OCCUPANCY_GRID_MAP_HPP_
