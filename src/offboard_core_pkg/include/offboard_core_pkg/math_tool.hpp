#pragma once

#include <cmath>
#include <array>
#include <algorithm>

/*
============================================================
                 数学工具函数检索表 (math_tool)
============================================================

【角度 / 航向处理】
------------------------------------------------------------
函数名                     功能
------------------------------------------------------------
wrap_pi(a)                 将角度归一化到 [-π, π]
wrap_2pi(a)                将角度归一化到 [0, 2π]
yaw_from_quat(q)           四元数 → yaw角 (PX4格式 [w x y z])
yaw_error(target, cur)     计算最短航向误差


【距离 / 位置计算】
------------------------------------------------------------
函数名                     功能
------------------------------------------------------------
dist2d(x1,y1,x2,y2)        计算二维距离
dist3d(x1,y1,z1,x2,y2,z2)  计算三维距离
dist2d_sq(x1,y1,x2,y2)     二维距离平方 (更快)
reached_xy(x,y,tx,ty,tol)  判断是否到达二维目标点


【向量处理】
------------------------------------------------------------
函数名                     功能
------------------------------------------------------------
normalize2d(x,y)           二维向量单位化


【数值限制 / 常用数学】
------------------------------------------------------------
函数名                     功能
------------------------------------------------------------
clamp(x,lo,hi)             限制数值范围
sign(x)                    返回符号 (-1,0,1)
deadband(v,th)             死区函数
lerp(a,b,t)                线性插值


【常量】
------------------------------------------------------------
PI                         π
TWO_PI                     2π

============================================================
使用示例：

float yaw = math_tool::yaw_from_quat(ctx.q);
float err = math_tool::wrap_pi(target_yaw - yaw);

float d = math_tool::dist2d(x,y,tx,ty);

float vx = math_tool::clamp(cmd_vx,-1.0f,1.0f);

============================================================
*/

namespace math_tool
{

// ===============================
// 常量
// ===============================

constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;


// ===============================
// clamp
// ===============================

template<typename T>
static inline T clamp(T x, T lo, T hi)
{
    return std::max(lo, std::min(hi, x));
}


// ===============================
// wrap angle to [-pi, pi]
// ===============================

static inline float wrap_pi(float a)
{
    while (a >  PI) a -= TWO_PI;
    while (a < -PI) a += TWO_PI;
    return a;
}


// ===============================
// wrap angle to [0, 2pi]
// ===============================

static inline float wrap_2pi(float a)
{
    while (a >= TWO_PI) a -= TWO_PI;
    while (a < 0.0f) a += TWO_PI;
    return a;
}


// ===============================
// quaternion → yaw
// PX4 quaternion format: [w x y z]
// ===============================

static inline float yaw_from_quat(const std::array<float,4>& q)
{
    const float w = q[0];
    const float x = q[1];
    const float y = q[2];
    const float z = q[3];

    const float siny_cosp = 2.0f * (w*z + x*y);
    const float cosy_cosp = 1.0f - 2.0f * (y*y + z*z);

    return std::atan2(siny_cosp, cosy_cosp);
}


// ===============================
// 2D distance
// ===============================

static inline float dist2d(float x1, float y1, float x2, float y2)
{
    const float dx = x1 - x2;
    const float dy = y1 - y2;
    return std::sqrt(dx*dx + dy*dy);
}


// ===============================
// 3D distance
// ===============================

static inline float dist3d(float x1, float y1, float z1,
                           float x2, float y2, float z2)
{
    const float dx = x1 - x2;
    const float dy = y1 - y2;
    const float dz = z1 - z2;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}


// ===============================
// squared distance (更快)
// ===============================

static inline float dist2d_sq(float x1, float y1, float x2, float y2)
{
    const float dx = x1 - x2;
    const float dy = y1 - y2;
    return dx*dx + dy*dy;
}
// 判断两个值是否接近（带容差）
static inline bool near(float a, float b, float tol)
{
    return std::fabs(a - b) <= tol;
}

// ===============================
// normalize vector 2D
// ===============================

static inline void normalize2d(float& x, float& y)
{
    const float n = std::sqrt(x*x + y*y);
    if (n > 1e-6f)
    {
        x /= n;
        y /= n;
    }
}


// ===============================
// deadband
// ===============================

static inline float deadband(float v, float th)
{
    if (std::fabs(v) < th)
        return 0.0f;
    return v;
}


// ===============================
// sign
// ===============================

template<typename T>
static inline int sign(T x)
{
    return (x > 0) - (x < 0);
}


// ===============================
// linear interpolation
// ===============================

static inline float lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}



static inline bool reached_xy(float x, float y,
                              float tx, float ty,
                              float tol)
{
    return dist2d_sq(x,y,tx,ty) < tol*tol;
}



static inline float yaw_error(float target, float current)
{
    return wrap_pi(target - current);
}

} // namespace math_tool