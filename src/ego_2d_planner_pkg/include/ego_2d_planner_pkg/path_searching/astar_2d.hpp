#pragma once

#include <vector>
#include <queue>
#include <limits>
#include <cmath>
#include <algorithm>
#include "ego_2d_planner_pkg/plan_env/grid_map_2d.hpp"

namespace ego_2d_planner_pkg
{

class AStar2D
{
public:
  bool search(const GridMap2D& map,
              const Vec2& start,
              const Vec2& goal,
              std::vector<Vec2>& path,
              int max_iter = 30000);

private:
  struct Node
  {
    int x{0};
    int y{0};
    double g{0.0};
    double f{0.0};
  };
};

}  // namespace ego_2d_planner_pkg
