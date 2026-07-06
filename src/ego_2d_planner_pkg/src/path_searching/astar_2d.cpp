#include "ego_2d_planner_pkg/path_searching/astar_2d.hpp"

namespace ego_2d_planner_pkg
{

bool AStar2D::search(const GridMap2D& map,
                     const Vec2& start,
                     const Vec2& goal,
                     std::vector<Vec2>& path,
                     int max_iter)
{
  path.clear();

  int sx = 0, sy = 0, gx = 0, gy = 0;
  if (!map.worldToGrid(start, sx, sy) || !map.worldToGrid(goal, gx, gy)) return false;
  if (map.isOccupied(sx, sy) || map.isOccupied(gx, gy)) return false;

  const int w = map.width();
  const int h = map.height();
  const int n = w * h;

  std::vector<double> g_score(n, std::numeric_limits<double>::infinity());
  std::vector<int> parent(n, -1);
  std::vector<uint8_t> closed(n, 0);

  auto idx = [w](int x, int y) { return y * w + x; };
  auto heuristic = [gx, gy](int x, int y) { return std::hypot(x - gx, y - gy); };

  struct Cmp
  {
    bool operator()(const Node& a, const Node& b) const { return a.f > b.f; }
  };

  std::priority_queue<Node, std::vector<Node>, Cmp> open;
  const int sidx = idx(sx, sy);
  g_score[sidx] = 0.0;
  open.push(Node{sx, sy, 0.0, heuristic(sx, sy)});

  const int dirs[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };

  int found_idx = -1;
  int iter = 0;

  while (!open.empty() && iter++ < max_iter) {
    const Node cur = open.top();
    open.pop();
    const int cidx = idx(cur.x, cur.y);
    if (closed[cidx]) continue;
    closed[cidx] = 1;

    if (cur.x == gx && cur.y == gy) {
      found_idx = cidx;
      break;
    }

    for (const auto& d : dirs) {
      const int nx = cur.x + d[0];
      const int ny = cur.y + d[1];
      if (!map.inMap(nx, ny) || map.isOccupied(nx, ny)) continue;

      // 防止斜向贴角穿障碍。
      if (d[0] != 0 && d[1] != 0) {
        if (map.isOccupied(cur.x + d[0], cur.y) || map.isOccupied(cur.x, cur.y + d[1])) continue;
      }

      const int nidx = idx(nx, ny);
      if (closed[nidx]) continue;

      const double step = (d[0] == 0 || d[1] == 0) ? 1.0 : 1.41421356237;
      const double ng = g_score[cidx] + step;
      if (ng < g_score[nidx]) {
        g_score[nidx] = ng;
        parent[nidx] = cidx;
        open.push(Node{nx, ny, ng, ng + heuristic(nx, ny)});
      }
    }
  }

  if (found_idx < 0) return false;

  std::vector<Vec2> rev;
  int cur = found_idx;
  while (cur >= 0) {
    const int x = cur % w;
    const int y = cur / w;
    rev.push_back(map.gridToWorld(x, y));
    if (cur == sidx) break;
    cur = parent[cur];
  }

  if (rev.empty()) return false;
  std::reverse(rev.begin(), rev.end());
  rev.front() = start;
  rev.back() = goal;
  path = std::move(rev);
  return true;
}

}  // namespace ego_2d_planner_pkg
