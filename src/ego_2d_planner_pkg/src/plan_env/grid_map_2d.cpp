#include "ego_2d_planner_pkg/plan_env/grid_map_2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ego_2d_planner_pkg
{

void GridMap2D::configure(const PlannerParams2D& p)
{
  resolution_ = std::max(0.02, p.resolution);
  size_x_ = std::max(1.0, p.map_size_x);
  size_y_ = std::max(1.0, p.map_size_y);
  inflate_radius_ = std::max(0.0, p.inflate_radius);
  occupied_threshold_ = p.occupied_threshold;

  persistent_cache_enable_ = p.persistent_cache_enable;
  persistent_confirm_s_ = std::max(0.0, p.persistent_confirm_s);
  persistent_miss_tolerance_s_ = std::max(0.0, p.persistent_miss_tolerance_s);
  persistent_min_observations_ = std::max(1, p.persistent_min_observations);

  width_ = static_cast<int>(std::ceil(size_x_ / resolution_));
  height_ = static_cast<int>(std::ceil(size_y_ / resolution_));
  inflate_cells_ = static_cast<int>(std::ceil(inflate_radius_ / resolution_));
  data_.assign(width_ * height_, 0);
  raw_data_.assign(width_ * height_, 0);
  dist_data_.assign(width_ * height_, 1e6);

  persistent_candidates_.clear();
  persistent_occupied_.clear();
  observed_keys_this_frame_.clear();
}

void GridMap2D::resetAround(const Vec2& center)
{
  origin_x_ = center.x - size_x_ * 0.5;
  origin_y_ = center.y - size_y_ * 0.5;
  std::fill(data_.begin(), data_.end(), 0);
  std::fill(raw_data_.begin(), raw_data_.end(), 0);
  std::fill(dist_data_.begin(), dist_data_.end(), 1e6);
  observed_keys_this_frame_.clear();
}

bool GridMap2D::worldToGrid(const Vec2& p, int& ix, int& iy) const
{
  ix = static_cast<int>(std::floor((p.x - origin_x_) / resolution_));
  iy = static_cast<int>(std::floor((p.y - origin_y_) / resolution_));
  return inMap(ix, iy);
}

Vec2 GridMap2D::gridToWorld(int ix, int iy) const
{
  return Vec2{origin_x_ + (static_cast<double>(ix) + 0.5) * resolution_,
              origin_y_ + (static_cast<double>(iy) + 0.5) * resolution_};
}

bool GridMap2D::inMap(int ix, int iy) const
{
  return ix >= 0 && iy >= 0 && ix < width_ && iy < height_;
}

int GridMap2D::index(int ix, int iy) const
{
  return iy * width_ + ix;
}

int GridMap2D::worldCellX(double x) const
{
  return static_cast<int>(std::floor(x / resolution_));
}

int GridMap2D::worldCellY(double y) const
{
  return static_cast<int>(std::floor(y / resolution_));
}

std::int64_t GridMap2D::worldKey(int wx, int wy) const
{
  const std::uint64_t ux = static_cast<std::uint32_t>(wx);
  const std::uint64_t uy = static_cast<std::uint32_t>(wy);
  return static_cast<std::int64_t>((ux << 32) | uy);
}

std::pair<int, int> GridMap2D::decodeWorldKey(std::int64_t key) const
{
  const std::uint64_t ukey = static_cast<std::uint64_t>(key);
  const int wx = static_cast<int>(static_cast<std::int32_t>(static_cast<std::uint32_t>(ukey >> 32)));
  const int wy = static_cast<int>(static_cast<std::int32_t>(static_cast<std::uint32_t>(ukey & 0xffffffffULL)));
  return {wx, wy};
}

Vec2 GridMap2D::worldCellCenter(int wx, int wy) const
{
  return Vec2{(static_cast<double>(wx) + 0.5) * resolution_,
              (static_cast<double>(wy) + 0.5) * resolution_};
}

void GridMap2D::beginUpdate(double stamp_s)
{
  update_stamp_s_ = stamp_s;
  observed_keys_this_frame_.clear();
}

void GridMap2D::setOccupiedWorld(const Vec2& p)
{
  int ix = 0, iy = 0;
  if (!worldToGrid(p, ix, iy)) return;

  raw_data_[index(ix, iy)] = 100;

  if (persistent_cache_enable_) {
    const int wx = worldCellX(p.x);
    const int wy = worldCellY(p.y);
    observed_keys_this_frame_.insert(worldKey(wx, wy));
  }
}

void GridMap2D::updatePersistentCandidate(std::int64_t key)
{
  if (persistent_occupied_.find(key) != persistent_occupied_.end()) {
    return;
  }

  auto it = persistent_candidates_.find(key);
  if (it == persistent_candidates_.end() ||
      update_stamp_s_ - it->second.last_seen_s > persistent_miss_tolerance_s_) {
    PersistentCandidate cand;
    cand.first_seen_s = update_stamp_s_;
    cand.last_seen_s = update_stamp_s_;
    cand.observations = 1;
    persistent_candidates_[key] = cand;
    return;
  }

  it->second.last_seen_s = update_stamp_s_;
  it->second.observations += 1;

  const double continuous_time = it->second.last_seen_s - it->second.first_seen_s;
  if (continuous_time >= persistent_confirm_s_ &&
      it->second.observations >= persistent_min_observations_) {
    persistent_occupied_.insert(key);
    persistent_candidates_.erase(it);
  }
}

void GridMap2D::applyPersistentOccupiedToRaw()
{
  if (!persistent_cache_enable_) {
    return;
  }

  for (const auto key : persistent_occupied_) {
    const auto [wx, wy] = decodeWorldKey(key);
    const Vec2 p = worldCellCenter(wx, wy);

    int ix = 0, iy = 0;
    if (worldToGrid(p, ix, iy)) {
      raw_data_[index(ix, iy)] = 100;
    }
  }
}

void GridMap2D::finishUpdate()
{
  if (!persistent_cache_enable_) {
    return;
  }

  for (const auto key : observed_keys_this_frame_) {
    updatePersistentCandidate(key);
  }

  // Remove stale unconfirmed candidates. Confirmed occupied cells are permanent
  // during this node's lifetime and are intentionally not removed here.
  for (auto it = persistent_candidates_.begin(); it != persistent_candidates_.end();) {
    if (update_stamp_s_ - it->second.last_seen_s > persistent_miss_tolerance_s_) {
      it = persistent_candidates_.erase(it);
    } else {
      ++it;
    }
  }

  applyPersistentOccupiedToRaw();
}

void GridMap2D::inflateObstacles()
{
  data_ = raw_data_;
  if (inflate_cells_ <= 0) return;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (raw_data_[index(x, y)] < occupied_threshold_) continue;
      for (int dy = -inflate_cells_; dy <= inflate_cells_; ++dy) {
        for (int dx = -inflate_cells_; dx <= inflate_cells_; ++dx) {
          const int nx = x + dx;
          const int ny = y + dy;
          if (!inMap(nx, ny)) continue;
          const double d = std::sqrt(static_cast<double>(dx * dx + dy * dy)) * resolution_;
          if (d <= inflate_radius_) {
            data_[index(nx, ny)] = 100;
          }
        }
      }
    }
  }
}

void GridMap2D::computeDistanceField()
{
  std::fill(dist_data_.begin(), dist_data_.end(), 1e6);

  struct Item
  {
    double d;
    int x;
    int y;
    bool operator>(const Item& o) const { return d > o.d; }
  };

  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (data_[index(x, y)] >= occupied_threshold_) {
        dist_data_[index(x, y)] = 0.0;
        q.push(Item{0.0, x, y});
      }
    }
  }

  const int dirs[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };

  while (!q.empty()) {
    const Item cur = q.top();
    q.pop();
    const int cidx = index(cur.x, cur.y);
    if (cur.d > dist_data_[cidx] + 1e-9) continue;

    for (const auto& dir : dirs) {
      const int nx = cur.x + dir[0];
      const int ny = cur.y + dir[1];
      if (!inMap(nx, ny)) continue;
      const double step = (dir[0] == 0 || dir[1] == 0) ? resolution_ : resolution_ * 1.41421356237;
      const double nd = cur.d + step;
      const int nidx = index(nx, ny);
      if (nd < dist_data_[nidx]) {
        dist_data_[nidx] = nd;
        q.push(Item{nd, nx, ny});
      }
    }
  }
}

bool GridMap2D::isOccupied(int ix, int iy) const
{
  if (!inMap(ix, iy)) return true;
  return data_[index(ix, iy)] >= occupied_threshold_;
}

bool GridMap2D::isOccupiedWorld(const Vec2& p) const
{
  int ix = 0, iy = 0;
  if (!worldToGrid(p, ix, iy)) return true;
  return isOccupied(ix, iy);
}

bool GridMap2D::isSegmentSafe(const Vec2& a, const Vec2& b, double step) const
{
  const double len = dist(a, b);
  const int n = std::max(1, static_cast<int>(std::ceil(len / std::max(0.02, step))));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    const Vec2 p = a + (b - a) * t;
    if (isOccupiedWorld(p)) return false;
  }
  return true;
}

bool GridMap2D::isPathSafe(const std::vector<Vec2>& path, double step) const
{
  if (path.empty()) return false;
  if (path.size() == 1) return !isOccupiedWorld(path.front());
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (!isSegmentSafe(path[i - 1], path[i], step)) return false;
  }
  return true;
}

double GridMap2D::getDistanceGrid(int ix, int iy) const
{
  if (!inMap(ix, iy)) return 0.0;
  return dist_data_[index(ix, iy)];
}

double GridMap2D::getDistanceWorld(const Vec2& p) const
{
  int ix = 0, iy = 0;
  if (!worldToGrid(p, ix, iy)) return 0.0;
  return getDistanceGrid(ix, iy);
}

Vec2 GridMap2D::getDistanceGradientWorld(const Vec2& p) const
{
  int ix = 0, iy = 0;
  if (!worldToGrid(p, ix, iy)) return {0.0, 0.0};

  const double dx = (getDistanceGrid(ix + 1, iy) - getDistanceGrid(ix - 1, iy)) / (2.0 * resolution_);
  const double dy = (getDistanceGrid(ix, iy + 1) - getDistanceGrid(ix, iy - 1)) / (2.0 * resolution_);
  Vec2 g{dx, dy};
  const double n = norm(g);
  if (n < 1e-6 || !std::isfinite(n)) return {0.0, 0.0};
  return g / n;
}

}  // namespace ego_2d_planner_pkg
