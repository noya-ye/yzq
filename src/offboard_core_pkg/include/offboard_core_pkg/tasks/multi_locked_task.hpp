#pragma once

#include "offboard_core_pkg/itask.hpp"
#include "offboard_core_pkg/context.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace offboard_core_pkg
{

class MultiLockedTask : public ITask
{
public:
  struct Config
  {
    // 整个多目标队列总超时
    double timeout_s = 120.0;

    // 单个目标对准总超时
    double align_timeout_s = 18.0;

    // 单个目标丢失超时
    double align_lost_timeout_s = 1.0;

    // 回原点距离阈值
    double return_origin_tol_m = 0.10;

    // 图像误差换算到米的比例
    double k_img_to_meter = 0.45;

    // 认为对准的误差，单位 m
    double align_tol_m = 0.04;

    // 连续多少帧对准才算完成
    int stable_required = 12;

    // 每次最大修正步长，单位 m
    double max_step_m = 0.10;

    // 目标最低面积/置信度
    double score_thresh = 500.0;

    // MOT 匹配最大距离，单位是归一化图像 dx/dy
    double max_match_dist = 0.22;

    // 最优和次优候选代价差小于这个值，认为 ambiguous
    double ambiguous_cost_gap = 0.04;

    // 连续丢失多少帧后放弃当前目标
    int max_lost_count = 8;

    // 连续 ambiguous 多少帧后进入 lost
    int max_ambiguous_count = 6;

    // 位置滤波系数，越大越跟手，越小越平滑
    double pos_alpha = 0.65;

    // 速度滤波系数
    double vel_alpha = 0.25;

    // 误差连续变大保护
    int max_error_grow_count = 6;

    // 误差增长容忍量，避免因为微小抖动误判
    double error_grow_eps_m = 0.015;

    // ambiguous 时步长缩放
    double ambiguous_step_scale = 0.30;

    // lost 时是否悬停
    bool hold_when_lost = true;

    // 是否每个目标对准后回到进入任务时的原点
    bool return_origin_between_targets = true;
  };

  explicit MultiLockedTask(rclcpp::Logger logger);

  MultiLockedTask(
      rclcpp::Logger logger,
      const Config& cfg);

  std::string name() const override;

  void onEnter(Context& ctx) override;

  Status tick(Context& ctx, double dt) override;

  void onExit(Context& ctx) override;

private:
  struct Detection
  {
    int type = 0;
    float dx = 0.0f;
    float dy = 0.0f;
    float score = 0.0f;
    float cost = 0.0f;
  };

  struct Track
  {
    bool valid = false;

    int id = 0;
    int type = 0;

    float dx = 0.0f;
    float dy = 0.0f;
    float score = 0.0f;
    float cost = 0.0f;

    float vx = 0.0f;
    float vy = 0.0f;

    int age = 0;
    int hit_count = 0;
    int lost_count = 0;
    int ambiguous_count = 0;
  };

  enum class MotState
  {
    IDLE,
    LOCKED,
    AMBIGUOUS,
    LOST,
    FAILED
  };

  enum class MultiState
  {
    WAIT_FRAME,
    PICK_NEXT,
    ALIGN_TARGET,
    RETURN_ORIGIN,
    FINISH
  };

private:
  rclcpp::Logger logger_;
  Config cfg_;

  MultiState multi_state_ = MultiState::WAIT_FRAME;

  MotState mot_state_ = MotState::IDLE;
  Track track_;
  int next_track_id_ = 1;

  double elapsed_s_ = 0.0;
  double print_elapsed_s_ = 0.0;
  double align_elapsed_s_ = 0.0;
  double align_lost_elapsed_s_ = 0.0;

  int stable_count_ = 0;
  int error_grow_count_ = 0;

  bool has_last_err_ = false;
  float last_err_m_ = 0.0f;

  float origin_x_ = 0.0f;
  float origin_y_ = 0.0f;
  float hold_z_ = 0.0f;

  std::vector<Detection> target_queue_;
  int target_index_ = 0;

  bool has_current_target_ = false;
  int current_target_type_ = 0;
  Detection current_seed_target_;

private:
  Status tick_wait_frame(Context& ctx);
  Status tick_pick_next(Context& ctx);
  Status tick_align_target(Context& ctx, double dt);
  Status tick_return_origin(Context& ctx);
  Status tick_finish(Context& ctx);

  void reset_mot();
  void reset_align_state();

  std::vector<Detection> build_detections(const Context& ctx) const;

  bool mot_update(const std::vector<Detection>& detections, double dt);

  bool mot_init_lock(const std::vector<Detection>& detections);

  bool mot_update_locked(const std::vector<Detection>& detections, double dt);

  float calc_match_cost(
      const Detection& det,
      float pred_dx,
      float pred_dy) const;

  void accept_detection(const Detection& det, float dt);

  bool mark_lost();

  bool mark_ambiguous();

  bool has_valid_lock() const;

  const char* mot_state_name() const;

  const char* multi_state_name() const;

  void image_offset_to_world_delta(
      const Context& ctx,
      float dx_img,
      float dy_img,
      float& dwx,
      float& dwy) const;

  float calc_dynamic_step(float err_m) const;

  bool check_error_growing(float err_m);

  Status finish_status() const;
};

}  // namespace offboard_core_pkg

