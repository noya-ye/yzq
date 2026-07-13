// 添加头文件：

// #include "../planners/a_star_planner.hpp"

// 调用示例：

// using AStarPlanner = offboard_core_pkg::AStarPlanner;

// std::vector<AStarPlanner::Cell> obstacles = {
//   {1, 1},
//   {2, 1},
//   {2, 2}
// };

// const auto result = AStarPlanner::plan(
//   cfg_.x_cells,
//   cfg_.y_cells,
//   cfg_.cell_size,
//   AStarPlanner::Cell{0, 0},
//   AStarPlanner::Cell{4, 4},
//   obstacles);

// if (!result.success) {
//   RCLCPP_ERROR(
//     logger_,
//     "[SNAKE] A* planning failed: %s",
//     result.message.c_str());

//   phase_ = Phase::FAILED;
//   return;
// }

// 把规划结果转换成你现在的 Waypoint：

// for (std::size_t i = 0; i < result.path.size(); ++i) {
//   // 第一个点通常就是当前所在的起点，可以跳过
//   if (i == 0) {
//     continue;
//   }

//   const auto& point = result.path[i];

//   Waypoint wp;
//   wp.ix = point.cell.x;
//   wp.iy = point.cell.y;

//   wp.x = origin_x_ +
//     static_cast<double>(cfg_.x_sign) * point.offset_x;

//   wp.y = origin_y_ +
//     static_cast<double>(cfg_.y_sign) * point.offset_y;

//   wp.z = target_z_;

//   // 转折点和最终目标点悬停
//   wp.hover_after = point.stop_after;

//   waypoints_.push_back(wp);
// }


#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace offboard_core_pkg
{

/*
 * Header-only 2D A* planner for grid-center waypoint generation.
 *
 * Coordinate convention:
 *   - cols: number of cells along x
 *   - rows: number of cells along y
 *   - Cell{0, 0} is the first cell
 *   - The center of Cell{0, 0} is treated as metric offset (0, 0)
 *   - metric_x = cell.x * cell_size
 *   - metric_y = cell.y * cell_size
 *
 * The returned route uses 4-connected motion only, so every segment is
 * parallel to a grid axis and passes through cell centers. A path point is
 * marked stop_after=true when it is a turning point or the final goal.
 */
class AStarPlanner
{
public:
  struct Cell
  {
    int x{0};
    int y{0};

    bool operator==(const Cell& other) const
    {
      return x == other.x && y == other.y;
    }

    bool operator!=(const Cell& other) const
    {
      return !(*this == other);
    }
  };

  struct PathPoint
  {
    Cell cell;

    // Relative metric position. Cell (0, 0) center is (0, 0).
    double offset_x{0.0};
    double offset_y{0.0};

    // True only when the direction changes at this point.
    bool is_turn{false};

    // SnakeGridTask can map this to Waypoint::hover_after.
    // The final goal is also marked true.
    bool stop_after{false};
  };

  struct Result
  {
    bool success{false};
    std::string message;
    std::vector<PathPoint> path;
  };

  static Result plan(
    int cols,
    int rows,
    double cell_size,
    const Cell& start,
    const Cell& goal,
    const std::vector<Cell>& obstacle_cells)
  {
    Result result;

    if (cols <= 0 || rows <= 0) {
      result.message = "grid size must be positive";
      return result;
    }

    if (!std::isfinite(cell_size) || cell_size <= 0.0) {
      result.message = "cell_size must be finite and positive";
      return result;
    }

    if (!inBounds(start, cols, rows)) {
      result.message = "start cell is outside the grid";
      return result;
    }

    if (!inBounds(goal, cols, rows)) {
      result.message = "goal cell is outside the grid";
      return result;
    }

    const int total = cols * rows;
    std::vector<std::uint8_t> blocked(static_cast<std::size_t>(total), 0U);

    for (const Cell& obstacle : obstacle_cells) {
      if (!inBounds(obstacle, cols, rows)) {
        result.message = "an obstacle cell is outside the grid";
        return result;
      }
      blocked[static_cast<std::size_t>(toIndex(obstacle, cols))] = 1U;
    }

    const int start_index = toIndex(start, cols);
    const int goal_index = toIndex(goal, cols);

    if (blocked[static_cast<std::size_t>(start_index)] != 0U) {
      result.message = "start cell is occupied";
      return result;
    }

    if (blocked[static_cast<std::size_t>(goal_index)] != 0U) {
      result.message = "goal cell is occupied";
      return result;
    }

    if (start == goal) {
      PathPoint point;
      point.cell = start;
      point.offset_x = static_cast<double>(start.x) * cell_size;
      point.offset_y = static_cast<double>(start.y) * cell_size;
      point.stop_after = true;

      result.success = true;
      result.message = "start equals goal";
      result.path.push_back(point);
      return result;
    }

    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> g_score(static_cast<std::size_t>(total), inf);
    std::vector<int> parent(static_cast<std::size_t>(total), -1);
    std::vector<std::uint8_t> closed(static_cast<std::size_t>(total), 0U);

    struct OpenNode
    {
      int index{-1};
      double f{0.0};
      double h{0.0};
    };

    struct CompareOpenNode
    {
      bool operator()(const OpenNode& lhs, const OpenNode& rhs) const
      {
        if (std::fabs(lhs.f - rhs.f) > 1e-12) {
          return lhs.f > rhs.f;
        }
        return lhs.h > rhs.h;
      }
    };

    std::priority_queue<OpenNode, std::vector<OpenNode>, CompareOpenNode> open;

    g_score[static_cast<std::size_t>(start_index)] = 0.0;
    const double start_h = heuristic(start, goal, cell_size);
    open.push(OpenNode{start_index, start_h, start_h});

    static constexpr int kNeighborDx[4] = {1, -1, 0, 0};
    static constexpr int kNeighborDy[4] = {0, 0, 1, -1};

    bool found = false;

    while (!open.empty()) {
      const OpenNode current_open = open.top();
      open.pop();

      const int current_index = current_open.index;
      if (closed[static_cast<std::size_t>(current_index)] != 0U) {
        continue;
      }

      if (current_index == goal_index) {
        found = true;
        break;
      }

      closed[static_cast<std::size_t>(current_index)] = 1U;
      const Cell current = fromIndex(current_index, cols);

      for (int i = 0; i < 4; ++i) {
        const Cell next{current.x + kNeighborDx[i], current.y + kNeighborDy[i]};

        if (!inBounds(next, cols, rows)) {
          continue;
        }

        const int next_index = toIndex(next, cols);
        const std::size_t next_pos = static_cast<std::size_t>(next_index);

        if (blocked[next_pos] != 0U || closed[next_pos] != 0U) {
          continue;
        }

        const double tentative_g =
          g_score[static_cast<std::size_t>(current_index)] + cell_size;

        if (tentative_g + 1e-12 >= g_score[next_pos]) {
          continue;
        }

        parent[next_pos] = current_index;
        g_score[next_pos] = tentative_g;

        const double h = heuristic(next, goal, cell_size);
        open.push(OpenNode{next_index, tentative_g + h, h});
      }
    }

    if (!found) {
      result.message = "no collision-free path found";
      return result;
    }

    std::vector<Cell> cells;
    for (int index = goal_index; index >= 0; index = parent[static_cast<std::size_t>(index)]) {
      cells.push_back(fromIndex(index, cols));
      if (index == start_index) {
        break;
      }
    }

    if (cells.empty() || cells.back() != start) {
      result.message = "path reconstruction failed";
      return result;
    }

    std::reverse(cells.begin(), cells.end());
    result.path.reserve(cells.size());

    for (std::size_t i = 0; i < cells.size(); ++i) {
      PathPoint point;
      point.cell = cells[i];
      point.offset_x = static_cast<double>(cells[i].x) * cell_size;
      point.offset_y = static_cast<double>(cells[i].y) * cell_size;

      if (i > 0 && i + 1 < cells.size()) {
        const int prev_dx = cells[i].x - cells[i - 1].x;
        const int prev_dy = cells[i].y - cells[i - 1].y;
        const int next_dx = cells[i + 1].x - cells[i].x;
        const int next_dy = cells[i + 1].y - cells[i].y;
        point.is_turn = prev_dx != next_dx || prev_dy != next_dy;
      }

      point.stop_after = point.is_turn || (i + 1 == cells.size());
      result.path.push_back(point);
    }

    result.success = true;
    result.message = "ok";
    return result;
  }

private:
  static bool inBounds(const Cell& cell, int cols, int rows)
  {
    return cell.x >= 0 && cell.x < cols && cell.y >= 0 && cell.y < rows;
  }

  static int toIndex(const Cell& cell, int cols)
  {
    return cell.y * cols + cell.x;
  }

  static Cell fromIndex(int index, int cols)
  {
    return Cell{index % cols, index / cols};
  }

  static double heuristic(const Cell& from, const Cell& to, double cell_size)
  {
    const int dx = std::abs(from.x - to.x);
    const int dy = std::abs(from.y - to.y);
    return static_cast<double>(dx + dy) * cell_size;
  }
};

}  // namespace offboard_core_pkg
