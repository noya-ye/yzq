#pragma once

#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"

#include "offboard_core_pkg/context.hpp"


class Grid {
public:
  using Waypoint = std::pair<float, float>;   // (x, y)

public:
  // 生成整张网格的遍历航点（蛇形）
  void build(Context& ctx);

  // 根据当前位置，更新当前所在格子编号
  void update_current_cell(Context& ctx) const;

  // 将格子坐标转换为世界坐标
  static Waypoint cell_center(const Context& ctx, int r, int c);

  // 清空网格
  void clear();
  void setPoints(const std::vector<Eigen::Vector3d>& pts);

  // 是否为空
  bool empty() const { return waypoints.empty(); }

  // 常用数学工具
  static float dist2d(float x1, float y1, float x2, float y2);
  static bool near(float a, float b, float tol);
  static float clamp_step(float current, float target, float max_step);

public:
  std::vector<Waypoint> waypoints;
  std::size_t wp_idx{0};
  std::vector<Eigen::Vector3d> points_;
};