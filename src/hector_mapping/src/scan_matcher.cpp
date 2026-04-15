#include "hector_mapping/scan_matcher.hpp"
#include <cmath>
#include <Eigen/Dense>

namespace hector_mapping
{

Eigen::Vector2d ScanMatcher::transformPoint(
    const Eigen::Vector3d & pose, const Eigen::Vector2d & pt)
{
  double ct = std::cos(pose.z()), st = std::sin(pose.z());
  return {pose.x() + ct * pt.x() - st * pt.y(),
          pose.y() + st * pt.x() + ct * pt.y()};
}

double ScanMatcher::match(
    Eigen::Vector3d & pose,
    const OccupancyGridMap & map,
    const std::vector<Eigen::Vector2d> & endpoints,
    int iterations,
    double translation_weight,
    double rotation_weight,
    double covariance_scale)
{
  if (endpoints.empty()) return 0.0;

  const int n = static_cast<int>(endpoints.size());

  for (int iter = 0; iter < iterations; ++iter) {
    double sin_th = std::sin(pose.z());
    double cos_th = std::cos(pose.z());

    // Hessian H (3×3) and gradient g (3×1) — Gauss-Newton
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d g = Eigen::Vector3d::Zero();

    for (int i = 0; i < n; ++i) {
      Eigen::Vector2d pw = transformPoint(pose, endpoints[i]);

      auto vg = map.getInterpolatedProbabilityAndGradient(pw.x(), pw.y());
      double M  = vg(0);   // interpolated occupancy probability
      double dMdx = vg(1);
      double dMdy = vg(2);

      // Jacobian of the world-frame endpoint w.r.t. pose (x, y, theta)
      //   dpw / d(x)     = (1, 0)
      //   dpw / d(y)     = (0, 1)
      //   dpw / d(theta) = (-sin*px - cos*py,  cos*px - sin*py)
      double dth_x = -sin_th * endpoints[i].x() - cos_th * endpoints[i].y();
      double dth_y =  cos_th * endpoints[i].x() - sin_th * endpoints[i].y();

      // dM / d(pose) via chain rule
      Eigen::Vector3d J;
      J(0) = dMdx;
      J(1) = dMdy;
      J(2) = dMdx * dth_x + dMdy * dth_y;

      double residual = 1.0 - M;   // we want M → 1 for occupied cells
      
      // Apply weights as Tikhonov regularization or scaling
      H += J * J.transpose();
      g += J * residual;
    }

    // Apply weights to the updates
    H(0,0) *= translation_weight;
    H(1,1) *= translation_weight;
    H(2,2) *= rotation_weight;

    // Apply covariance scaling for stability
    H *= covariance_scale;
    g *= covariance_scale;
    
    // Damping for numerical stability
    H.diagonal().array() += 1e-3;

    Eigen::Vector3d delta = H.ldlt().solve(g);
    
    // Limit pose updates to prevent large jumps within iterations
    // but keep them reasonable for motion tracking
    double max_translation = 0.5;  // 50cm max per iteration
    double max_rotation = 0.3;    // ~17 degrees max per iteration
    
    if (delta.head<2>().norm() > max_translation) {
      delta.head<2>() = delta.head<2>().normalized() * max_translation;
    }
    if (std::abs(delta.z()) > max_rotation) {
      delta.z() = (delta.z() > 0) ? max_rotation : -max_rotation;
    }

    // Apply update
    pose += delta;

    // Early termination
    if (delta.head<2>().squaredNorm() < 1e-8 &&
        std::abs(delta.z()) < 1e-6) {
      break;
    }
  }

  // Return total score
  double score = 0.0;
  for (auto & pt : endpoints) {
    Eigen::Vector2d pw = transformPoint(pose, pt);
    score += map.getInterpolatedProbability(pw.x(), pw.y());
  }
  return score;
}

}  // namespace hector_mapping
