// 这个 helper 怎么单独调用

// 最简单调用：

// #include "offboard_core_pkg/helpers/tsp_helper.hpp"

// std::vector<Eigen::Vector3d> targets;

// targets.push_back(Eigen::Vector3d(2.0, 1.0, -1.0));
// targets.push_back(Eigen::Vector3d(0.5, 3.0, -1.0));
// targets.push_back(Eigen::Vector3d(1.0, 0.5, -1.0));

// Eigen::Vector3d start(ctx.cx(), ctx.cy(), ctx.cz());

// std::vector<Eigen::Vector3d> sorted =
//   offboard_core_pkg::tsp_helper::sortPath(start, targets);

//2d排序





#pragma once

#include <vector>
#include <limits>
#include <algorithm>
#include <numeric>
#include <cmath>

#include <Eigen/Dense>

namespace offboard_core_pkg
{
namespace tsp_helper
{

using Vec3 = Eigen::Vector3d;

// ============================================================
// 2D 距离：默认忽略 z，只按平面路径排序
// ============================================================
inline double dist2D(const Vec3& a, const Vec3& b)
{
  const double dx = a.x() - b.x();
  const double dy = a.y() - b.y();
  return std::sqrt(dx * dx + dy * dy);
}

// ============================================================
// 计算一条开放路径长度：start -> targets[order[0]] -> ...
// 不回到起点
// ============================================================
inline double pathLength(
  const Vec3& start,
  const std::vector<size_t>& order,
  const std::vector<Vec3>& targets)
{
  if (order.empty()) {
    return 0.0;
  }

  double total = 0.0;
  Vec3 prev = start;

  for (const size_t idx : order) {
    total += dist2D(prev, targets[idx]);
    prev = targets[idx];
  }

  return total;
}

// ============================================================
// 最近邻初始解
// 输入：起点 + 目标点
// 输出：目标点访问顺序 index
// ============================================================
inline std::vector<size_t> nearestNeighborOrder(
  const Vec3& start,
  const std::vector<Vec3>& targets)
{
  const size_t n = targets.size();

  std::vector<size_t> order;
  order.reserve(n);

  if (n == 0) {
    return order;
  }

  std::vector<bool> visited(n, false);

  Vec3 current = start;

  for (size_t step = 0; step < n; ++step) {
    double best_dist = std::numeric_limits<double>::max();
    size_t best_idx = n;

    for (size_t i = 0; i < n; ++i) {
      if (visited[i]) {
        continue;
      }

      const double d = dist2D(current, targets[i]);

      if (d < best_dist) {
        best_dist = d;
        best_idx = i;
      }
    }

    if (best_idx >= n) {
      break;
    }

    order.push_back(best_idx);
    visited[best_idx] = true;
    current = targets[best_idx];
  }

  return order;
}

// ============================================================
// 2-opt 优化开放路径
// 路径形式：start -> p0 -> p1 -> ... -> pn
// 不强制回到 start
// ============================================================
inline void twoOptOpenPath(
  const Vec3& start,
  std::vector<size_t>& order,
  const std::vector<Vec3>& targets)
{
  const size_t n = order.size();

  if (n < 3) {
    return;
  }

  bool improved = true;
  const double eps = 1e-9;

  while (improved) {
    improved = false;

    for (size_t i = 0; i + 1 < n; ++i) {
      for (size_t k = i + 1; k < n; ++k) {
        const Vec3& prev = (i == 0)
          ? start
          : targets[order[i - 1]];

        const Vec3& a = targets[order[i]];
        const Vec3& b = targets[order[k]];

        double before = dist2D(prev, a);
        double after  = dist2D(prev, b);

        if (k + 1 < n) {
          const Vec3& next = targets[order[k + 1]];
          before += dist2D(b, next);
          after  += dist2D(a, next);
        }

        if (after + eps < before) {
          std::reverse(order.begin() + static_cast<long>(i),
                       order.begin() + static_cast<long>(k + 1));
          improved = true;
        }
      }
    }
  }
}

// ============================================================
// 主函数 1：输入起点 + 目标点，输出排序后的 index
// 这个适合用于重排 Waypoint，因为可以保留 hover/yaw 等信息
// ============================================================
inline std::vector<size_t> solveOrder(
  const Vec3& start,
  const std::vector<Vec3>& targets)
{
  if (targets.empty()) {
    return {};
  }

  std::vector<size_t> order = nearestNeighborOrder(start, targets);
  twoOptOpenPath(start, order, targets);

  return order;
}

// ============================================================
// 主函数 2：输入起点 + 目标点，直接输出排序后的目标点序列
// ============================================================
inline std::vector<Vec3> sortPath(
  const Vec3& start,
  const std::vector<Vec3>& targets)
{
  std::vector<Vec3> sorted;
  sorted.reserve(targets.size());

  const std::vector<size_t> order = solveOrder(start, targets);

  for (const size_t idx : order) {
    sorted.push_back(targets[idx]);
  }

  return sorted;
}

// ============================================================
// 主函数 3：不传起点时，默认从 0,0,0 开始
// 不推荐实机使用，但方便离线测试
// ============================================================
inline std::vector<Vec3> sortPath(
  const std::vector<Vec3>& targets)
{
  return sortPath(Vec3::Zero(), targets);
}

}  // namespace tsp_helper
}  // namespace offboard_core_pkg