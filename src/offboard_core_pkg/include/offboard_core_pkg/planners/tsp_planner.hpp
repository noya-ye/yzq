#pragma once

#include <vector>
#include <Eigen/Dense>
#include <limits>
#include <algorithm>

namespace offboard_core_pkg
{

class TspPlanner
{
public:
  using Vec3 = Eigen::Vector3d;

  // =========================
  // 主入口
  // =========================
  static std::vector<Vec3> solve(const std::vector<Vec3>& targets)
  {
    if (targets.empty()) return {};

    auto path = nearestNeighbor(targets);
    twoOpt(path, targets);

    std::vector<Vec3> ordered;
    for (int idx : path)
    {
      ordered.push_back(targets[idx]);
    }

    return ordered;
  }

private:

  // =========================
  // 2D距离（忽略Z）
  // =========================
  static double dist2D(const Vec3& a, const Vec3& b)
  {
    double dx = a.x() - b.x();
    double dy = a.y() - b.y();
    return std::sqrt(dx * dx + dy * dy);
  }

  // =========================
  // 最近邻
  // =========================
  static std::vector<int> nearestNeighbor(const std::vector<Vec3>& pts)
  {
    int n = pts.size();

    std::vector<bool> visited(n, false);
    std::vector<int> path;

    Vec3 current(0, 0, 0); // 起点

    for (int i = 0; i < n; ++i)
    {
      double best = std::numeric_limits<double>::max();
      int best_idx = -1;

      for (int j = 0; j < n; ++j)
      {
        if (!visited[j])
        {
          double d = dist2D(current, pts[j]);
          if (d < best)
          {
            best = d;
            best_idx = j;
          }
        }
      }

      path.push_back(best_idx);
      visited[best_idx] = true;
      current = pts[best_idx];
    }

    return path;
  }

  // =========================
  // 路径长度（2D）
  // =========================
  static double pathLength(const std::vector<int>& path,
                           const std::vector<Vec3>& pts)
  {
    double total = 0.0;
    Vec3 prev(0, 0, 0);

    for (int idx : path)
    {
      total += dist2D(prev, pts[idx]);
      prev = pts[idx];
    }

    return total;
  }

  // =========================
  // 2-opt优化（2D）
  // =========================
  static void twoOpt(std::vector<int>& path,
                     const std::vector<Vec3>& pts)
  {
    int n = path.size();
    if (n < 3) return;

    bool improved = true;

    while (improved)
    {
      improved = false;

      for (int i = 0; i < n - 1; ++i)
      {
        for (int j = i + 1; j < n; ++j)
        {
          double before = pathLength(path, pts);

          std::reverse(path.begin() + i, path.begin() + j + 1);

          double after = pathLength(path, pts);

          if (after < before)
          {
            improved = true;
          }
          else
          {
            std::reverse(path.begin() + i, path.begin() + j + 1);
          }
        }
      }
    }
  }

};

} // namespace