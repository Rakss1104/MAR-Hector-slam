#include "hector_mapping/occupancy_grid_map.hpp"
#include <algorithm>
#include <cmath>

namespace hector_mapping
{

// ─── helpers ────────────────────────────────────────────────────────────
static constexpr double kLogOddsClamp = 50.0;   // avoid overflow

double OccupancyGridMap::logOddsFromProb(double p)
{
  p = std::clamp(p, 0.001, 0.999);
  return std::log(p / (1.0 - p));
}
double OccupancyGridMap::probFromLogOdds(double l)
{
  return 1.0 / (1.0 + std::exp(-l));
}

// ─── constructor ────────────────────────────────────────────────────────
OccupancyGridMap::OccupancyGridMap(
    int map_size, double resolution,
    double start_x, double start_y,
    double update_free, double update_occ)
: map_size_(map_size),
  resolution_(resolution),
  log_free_(logOddsFromProb(update_free)),
  log_occ_(logOddsFromProb(update_occ)),
  log_odds_(map_size * map_size, 0.0)
{
  // Center the map on world origin (0,0) with robot starting at center
  double half_map_size_meters = map_size * resolution * 0.5;
  origin_x_ = -half_map_size_meters;
  origin_y_ = -half_map_size_meters;
}

// ─── coordinate conversions ─────────────────────────────────────────────
Eigen::Vector2i OccupancyGridMap::worldToGrid(const Eigen::Vector2d & w) const
{
  return {static_cast<int>(std::floor((w.x() - origin_x_) / resolution_)),
          static_cast<int>(std::floor((w.y() - origin_y_) / resolution_))};
}

Eigen::Vector2d OccupancyGridMap::gridToWorld(const Eigen::Vector2i & g) const
{
  return {origin_x_ + (g.x() + 0.5) * resolution_,
          origin_y_ + (g.y() + 0.5) * resolution_};
}

bool OccupancyGridMap::isInside(int gx, int gy) const
{
  return gx >= 0 && gx < map_size_ && gy >= 0 && gy < map_size_;
}

// ─── probability access ────────────────────────────────────────────────
double OccupancyGridMap::getProbability(int gx, int gy) const
{
  if (!isInside(gx, gy)) return 0.5;
  return probFromLogOdds(log_odds_[idx(gx, gy)]);
}

void OccupancyGridMap::updateSetOccupied(int gx, int gy)
{
  if (!isInside(gx, gy)) return;
  double & v = log_odds_[idx(gx, gy)];
  v = std::clamp(v + log_occ_, -kLogOddsClamp, kLogOddsClamp);
}

void OccupancyGridMap::updateSetFree(int gx, int gy)
{
  if (!isInside(gx, gy)) return;
  double & v = log_odds_[idx(gx, gy)];
  v = std::clamp(v + log_free_, -kLogOddsClamp, kLogOddsClamp);
}

// ─── Bresenham-based ray update ─────────────────────────────────────────
void OccupancyGridMap::updateByScan(
    const Eigen::Vector2d & sensor_world,
    const std::vector<Eigen::Vector2d> & endpoints_world)
{
  Eigen::Vector2i s = worldToGrid(sensor_world);
  for (auto & ep : endpoints_world) {
    Eigen::Vector2i e = worldToGrid(ep);

    // Bresenham line from s to e — mark intermediate cells free
    int dx = std::abs(e.x() - s.x()), dy = std::abs(e.y() - s.y());
    int sx = (s.x() < e.x()) ? 1 : -1;
    int sy = (s.y() < e.y()) ? 1 : -1;
    int err = dx - dy;
    int cx = s.x(), cy = s.y();

    while (cx != e.x() || cy != e.y()) {
      updateSetFree(cx, cy);
      int e2 = 2 * err;
      if (e2 > -dy) { err -= dy; cx += sx; }
      if (e2 <  dx) { err += dx; cy += sy; }
    }
    // endpoint is occupied
    updateSetOccupied(e.x(), e.y());
  }
}

// ─── bilinear interpolation ─────────────────────────────────────────────
double OccupancyGridMap::getInterpolatedProbability(double wx, double wy) const
{
  double fx = (wx - origin_x_) / resolution_ - 0.5;
  double fy = (wy - origin_y_) / resolution_ - 0.5;
  int ix = static_cast<int>(std::floor(fx));
  int iy = static_cast<int>(std::floor(fy));
  double dx = fx - ix, dy = fy - iy;

  double p00 = getProbability(ix,     iy);
  double p10 = getProbability(ix + 1, iy);
  double p01 = getProbability(ix,     iy + 1);
  double p11 = getProbability(ix + 1, iy + 1);

  return (1 - dx) * (1 - dy) * p00 +
         dx       * (1 - dy) * p10 +
         (1 - dx) * dy       * p01 +
         dx       * dy       * p11;
}

Eigen::Vector3d OccupancyGridMap::getInterpolatedProbabilityAndGradient(
    double wx, double wy) const
{
  double fx = (wx - origin_x_) / resolution_ - 0.5;
  double fy = (wy - origin_y_) / resolution_ - 0.5;
  int ix = static_cast<int>(std::floor(fx));
  int iy = static_cast<int>(std::floor(fy));
  double dx = fx - ix, dy = fy - iy;

  double p00 = getProbability(ix,     iy);
  double p10 = getProbability(ix + 1, iy);
  double p01 = getProbability(ix,     iy + 1);
  double p11 = getProbability(ix + 1, iy + 1);

  double val = (1 - dx) * (1 - dy) * p00 +
               dx       * (1 - dy) * p10 +
               (1 - dx) * dy       * p01 +
               dx       * dy       * p11;

  // d/d(grid_x) → divide by resolution to get d/d(world_x)
  double dvdx = ((1 - dy) * (p10 - p00) + dy * (p11 - p01)) / resolution_;
  double dvdy = ((1 - dx) * (p01 - p00) + dx * (p11 - p10)) / resolution_;

  return {val, dvdx, dvdy};
}

}  // namespace hector_mapping
