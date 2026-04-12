#pragma once

#include <array>
#include <cmath>

namespace fastlio_to_px4
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quat
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// ---------------------------
// 基础四元数工具
// ---------------------------
inline Quat quat_multiply(const Quat &a, const Quat &b)
{
  Quat q;
  q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
  q.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
  q.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
  q.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
  return q;
}

inline Quat quat_conjugate(const Quat &q)
{
  return Quat{q.w, -q.x, -q.y, -q.z};
}

inline Quat quat_normalize(const Quat &q)
{
  const double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n < 1e-12) {
    return Quat{};
  }
  return Quat{q.w/n, q.x/n, q.y/n, q.z/n};
}

inline Vec3 quat_rotate(const Quat &q_in, const Vec3 &v)
{
  const Quat q = quat_normalize(q_in);
  Quat vq{0.0, v.x, v.y, v.z};
  Quat rq = quat_multiply(quat_multiply(q, vq), quat_conjugate(q));
  return Vec3{rq.x, rq.y, rq.z};
}

// ---------------------------
// 坐标系转换
// ROS world: ENU
// PX4 world: NED
//
// ENU -> NED:
// x_ned = y_enu
// y_ned = x_enu
// z_ned = -z_enu
// ---------------------------
inline Vec3 enu_to_ned(const Vec3 &v_enu)
{
  return Vec3{
    v_enu.y,
    v_enu.x,
    -v_enu.z
  };
}

// ---------------------------
// ROS body: FLU
// PX4 body: FRD
//
// FLU -> FRD:
// x_frd = x_flu
// y_frd = -y_flu
// z_frd = -z_flu
// ---------------------------
inline Vec3 flu_to_frd(const Vec3 &v_flu)
{
  return Vec3{
    v_flu.x,
    -v_flu.y,
    -v_flu.z
  };
}

// ---------------------------
// 四元数转旋转矩阵（body -> world）
// q = [w x y z]
// ---------------------------
inline std::array<std::array<double, 3>, 3> quat_to_rotmat(const Quat &q_in)
{
  const Quat q = quat_normalize(q_in);

  const double w = q.w;
  const double x = q.x;
  const double y = q.y;
  const double z = q.z;

  return {{
    {{1.0 - 2.0*(y*y + z*z), 2.0*(x*y - z*w),       2.0*(x*z + y*w)}},
    {{2.0*(x*y + z*w),       1.0 - 2.0*(x*x + z*z), 2.0*(y*z - x*w)}},
    {{2.0*(x*z - y*w),       2.0*(y*z + x*w),       1.0 - 2.0*(x*x + y*y)}}
  }};
}

// ---------------------------
// 旋转矩阵转四元数
// ---------------------------
inline Quat rotmat_to_quat(const std::array<std::array<double, 3>, 3> &R)
{
  Quat q{};
  const double trace = R[0][0] + R[1][1] + R[2][2];

  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q.w = 0.25 * s;
    q.x = (R[2][1] - R[1][2]) / s;
    q.y = (R[0][2] - R[2][0]) / s;
    q.z = (R[1][0] - R[0][1]) / s;
  } else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
    const double s = std::sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0;
    q.w = (R[2][1] - R[1][2]) / s;
    q.x = 0.25 * s;
    q.y = (R[0][1] + R[1][0]) / s;
    q.z = (R[0][2] + R[2][0]) / s;
  } else if (R[1][1] > R[2][2]) {
    const double s = std::sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0;
    q.w = (R[0][2] - R[2][0]) / s;
    q.x = (R[0][1] + R[1][0]) / s;
    q.y = 0.25 * s;
    q.z = (R[1][2] + R[2][1]) / s;
  } else {
    const double s = std::sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0;
    q.w = (R[1][0] - R[0][1]) / s;
    q.x = (R[0][2] + R[2][0]) / s;
    q.y = (R[1][2] + R[2][1]) / s;
    q.z = 0.25 * s;
  }

  return quat_normalize(q);
}

// ---------------------------
// 姿态转换：ROS body->ENU  转 PX4 FRD->NED
//
// 若 R_enu_flu 表示 FLU body 到 ENU world
// 则 R_ned_frd = T_enu2ned * R_enu_flu * T_frd2flu
//
// 其中：
// T_enu2ned = [ [0,1,0], [1,0,0], [0,0,-1] ]
// T_frd2flu = [ [1,0,0], [0,-1,0], [0,0,-1] ]
// ---------------------------
inline Quat attitude_enu_flu_to_ned_frd(const Quat &q_enu_flu)
{
  const auto R = quat_to_rotmat(q_enu_flu);

  const std::array<std::array<double, 3>, 3> T_enu2ned = {{
    {{0.0, 1.0, 0.0}},
    {{1.0, 0.0, 0.0}},
    {{0.0, 0.0, -1.0}}
  }};

  const std::array<std::array<double, 3>, 3> T_frd2flu = {{
    {{1.0, 0.0, 0.0}},
    {{0.0, -1.0, 0.0}},
    {{0.0, 0.0, -1.0}}
  }};

  std::array<std::array<double, 3>, 3> tmp{};
  std::array<std::array<double, 3>, 3> out{};

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      tmp[i][j] = 0.0;
      for (int k = 0; k < 3; ++k) {
        tmp[i][j] += T_enu2ned[i][k] * R[k][j];
      }
    }
  }

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out[i][j] = 0.0;
      for (int k = 0; k < 3; ++k) {
        out[i][j] += tmp[i][k] * T_frd2flu[k][j];
      }
    }
  }

  return rotmat_to_quat(out);
}

}  // namespace fastlio_to_px4