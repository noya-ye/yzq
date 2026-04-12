#include "offboard_core_pkg/grid.hpp"

#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

void Grid::build(Context& ctx)
{
  waypoints.clear();
  wp_idx = 0;

  // 给数字存储区做初始化，避免后续越界
  const int n = ctx.rows * ctx.cols;
  ctx.grid_numbers.assign(n, -1);
  ctx.grid_num_best_score.assign(n, 0.0f);
  ctx.grid_num_stamp_us.assign(n, 0ULL);

  // 蛇形遍历
  for (int r = 0; r < ctx.rows; ++r) {
    if ((r % 2) == 0) {
      // 偶数行：左 -> 右
      for (int c = 0; c < ctx.cols; ++c) {
        waypoints.push_back(cell_center(ctx, r, c));
      }
    } else {
      // 奇数行：右 -> 左
      for (int c = ctx.cols - 1; c >= 0; --c) {
        waypoints.push_back(cell_center(ctx, r, c));
      }
    }
  }
}

void Grid::update_current_cell(Context& ctx) const
{
  if (!ctx.pos_valid()) {
    ctx.current_r = -1;
    ctx.current_c = -1;
    return;
  }

  const float x0 = ctx.home_x - static_cast<float>(ctx.origin_dx);
  const float y0 = ctx.home_y + static_cast<float>(ctx.origin_dy);
  const float cs = static_cast<float>(ctx.cell_size);

  // 由当前位置反推格子索引
  const float rf = (x0 - ctx.cx()) / cs;
  const float cf = (ctx.cy() - y0) / cs;

  int r = static_cast<int>(std::round(rf));
  int c = static_cast<int>(std::round(cf));

  // 边界裁剪
  r = std::clamp(r, 0, ctx.rows - 1);
  c = std::clamp(c, 0, ctx.cols - 1);

  ctx.current_r = r;
  ctx.current_c = c;
}

Grid::Waypoint Grid::cell_center(const Context& ctx, int r, int c)
{
  const float x =
    ctx.home_x - static_cast<float>(ctx.origin_dx)
               - static_cast<float>(r) * static_cast<float>(ctx.cell_size);

  const float y =
    ctx.home_y + static_cast<float>(ctx.origin_dy)
               + static_cast<float>(c) * static_cast<float>(ctx.cell_size);

  return {x, y};
}

void Grid::clear()
{
  waypoints.clear();
  wp_idx = 0;
}

float Grid::dist2d(float x1, float y1, float x2, float y2)
{
  const float dx = x1 - x2;
  const float dy = y1 - y2;
  return std::sqrt(dx * dx + dy * dy);
}

bool Grid::near(float a, float b, float tol)
{
  return std::fabs(a - b) <= tol;
}

float Grid::clamp_step(float current, float target, float max_step)
{
  const float delta = target - current;

  if (delta > max_step)  return current + max_step;
  if (delta < -max_step) return current - max_step;
  return target;
}
void Grid::setPoints(const std::vector<Eigen::Vector3d>& pts)
{
    points_ = pts;
}//用于写死位置作为调试