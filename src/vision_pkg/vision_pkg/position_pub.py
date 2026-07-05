# import rclpy
# from rclpy.node import Node
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image, CameraInfo
# from geometry_msgs.msg import PointStamped
# from nav_msgs.msg import Odometry
# from collections import deque
# import numpy as np
# import tf_transformations
# import signal
# import sys
# from std_msgs.msg import Bool
# # 订阅：
# #   /vision/front/enable      std_msgs/msg/Bool
# #   /target_point             geometry_msgs/msg/PointStamped
# #   /d435/depth               sensor_msgs/msg/Image
# #   /d435/camera_info         sensor_msgs/msg/CameraInfo
# #   /fastlio2/lio_odom        nav_msgs/msg/Odometry

# # 发布：
# #   /vision/front/goal_position   geometry_msgs/msg/PointStamped
# #   /calib/camera_point           geometry_msgs/msg/PointStamped
# #   /calib/body_true_point        geometry_msgs/msg/PointStamped
# #   /calib/lio_point              geometry_msgs/msg/PointStamped

# # 作用：
# #   把 D435 图像中的目标像素点转换成 FAST-LIO2 世界坐标下的目标点，
# #   同时收集 Pc 与 Pb_true 样本，用于在线优化相机到雷达/body 的外参。


# class Positionpub_Node(Node):
#     def __init__(self):
#         super().__init__('Positionpub_Node')

#         self.bridge = CvBridge()

#         # ============================================================
#         # 已知真实目标点 FAST-LIO2 坐标
#         # 目标点真实坐标：P_lio_true = (0m, 1.0m, 0m)
#         # ============================================================
#         self.P_lio_true = np.array([0.0, 1.0, 0.0], dtype=np.float64)

#         # ============================================================
#         # 相机内参
#         # ============================================================
#         self.fx = None
#         self.fy = None
#         self.cx = None
#         self.cy = None

#         # ============================================================
#         # 外参：Pc(camera) -> Pb(body/lidar)
#         #
#         # 你的相机安装在雷达坐标系 y 轴正方向，所以初始 t_cb 给 y 正方向
#         # 如果实际相机距离雷达中心 5cm，就改成 [0.0, 0.05, 0.0]
#         # 如果实际 10cm，就改成 [0.0, 0.10, 0.0]
#         # ============================================================
#         self.R_cb = np.eye(3, dtype=np.float64)
#         self.t_cb = np.array([0.0, 0.08, 0.0], dtype=np.float64)

#         # 是否使用在线优化后的外参发布最终 FAST-LIO2 坐标
#         self.use_optimized_extrinsic = True

#         # ============================================================
#         # 缓存
#         # ============================================================
#         self.depth_buffer = deque(maxlen=30)      # (stamp_sec, depth_image)
#         self.odom_buffer = deque(maxlen=200)      # (stamp_sec, R_lio, T_lio)

#         # Pc 低通缓存，降低单帧深度抖动
#         self.pc_buffer = deque(maxlen=5)

#         # 标定样本缓存
#         # 每个样本是 (Pc, Pb_true)
#         self.samples = []

#         # 最近一次有效计算结果，用于低频打印
#         self.last_status = None

#         # ============================================================
#         # 参数
#         # ============================================================
#         self.max_depth_dt = 0.40      # depth 约 3Hz，允许 400ms 匹配窗口
#         self.max_odom_dt = 0.20       # FAST-LIO odom 一般更快，先给 200ms

#         self.min_depth = 0.05          # D435 最小有效深度
#         self.max_depth = 8.0           # 根据场地可调整

#         self.min_samples_to_opt = 6    # 至少多少个样本后开始优化外参
#         self.max_samples = 300         # 最多保留多少个好样本

#         # 坏值过滤阈值
#         self.raw_pair_max_dist = 5.0   # Pc 和 Pb_true 差太离谱，直接丢
#         self.residual_gate_start = 0.35  # 优化后残差大于 35cm，认为是坏点
#         self.mad_k = 3.5               # MAD 鲁棒过滤系数
#         self.min_keep_after_filter = 6

#         # 打印频率控制
#         self.print_period = 1.0        # 1秒打印一次
#         self.warn_period = 2.0         # 同类 warn 最快 2秒一次
#         self.last_warn_time = {}

#         # 统计
#         self.total_points = 0
#         self.accepted_points = 0
#         self.rejected_points = 0
#         self.optimized_count = 0
#         self.enabled = True

#         self.create_subscription(
#             Bool,
#             '/vision/front/enable',
#             self.enable_callback,
#             10
#         )

#         # ============================================================
#         # ROS 订阅
#         # ============================================================
#         self.create_subscription(PointStamped, '/target_point', self.point_callback, 10)
#         self.create_subscription(Image, '/d435/depth', self.depth_callback, 10)
#         self.create_subscription(CameraInfo, '/d435/camera_info', self.info_callback, 10)
#         self.create_subscription(Odometry, '/fastlio2/lio_odom', self.odom_callback, 50)

#         # ============================================================
#         # ROS 发布
#         # ============================================================
#         self.lio_pub = self.create_publisher(PointStamped, '/vision/front/goal_position', 10)

#         # 标定调试用
#         self.cam_pub = self.create_publisher(PointStamped, '/calib/camera_point', 10)
#         self.body_true_pub = self.create_publisher(PointStamped, '/calib/body_true_point', 10)
#         self.lio_calib_pub = self.create_publisher(PointStamped, '/calib/lio_point', 10)

#         # 低频打印 timer
#         self.create_timer(self.print_period, self.print_status_timer)

#         self.get_logger().info("============================================================")
#         self.get_logger().info("Extrinsic Online Calibration Node started")
#         self.get_logger().info("Known target FAST-LIO2 point P_lio_true = [0.0, 1.0, 0.0]")
#         self.get_logger().info("Estimate extrinsic: Pc(camera) -> Pb(body/lidar)")
#         self.get_logger().info("Initial t_cb assumes camera is on +Y of lidar/body frame")
#         self.get_logger().info("Press Ctrl+C to print final best extrinsic parameters")
#         self.get_logger().info("============================================================")

#     # ============================================================
#     # 工具函数
#     # ============================================================
    
#     def enable_callback(self, msg):
#         self.enabled = bool(msg.data)
#     @staticmethod
#     def stamp_to_sec(stamp_msg):
#         return float(stamp_msg.sec) + float(stamp_msg.nanosec) * 1e-9

#     def now_sec(self):
#         return self.get_clock().now().nanoseconds * 1e-9

#     def throttled_warn(self, key, text):
#         now = self.now_sec()
#         last = self.last_warn_time.get(key, -1e9)
#         if now - last >= self.warn_period:
#             self.get_logger().warn(text)
#             self.last_warn_time[key] = now

#     @staticmethod
#     def is_finite_vec(v):
#         return np.all(np.isfinite(v))

#     @staticmethod
#     def rot_to_rpy_deg(R):
#         # R = Rz(yaw) Ry(pitch) Rx(roll)
#         sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
#         singular = sy < 1e-6

#         if not singular:
#             roll = np.arctan2(R[2, 1], R[2, 2])
#             pitch = np.arctan2(-R[2, 0], sy)
#             yaw = np.arctan2(R[1, 0], R[0, 0])
#         else:
#             roll = np.arctan2(-R[1, 2], R[1, 1])
#             pitch = np.arctan2(-R[2, 0], sy)
#             yaw = 0.0

#         return np.rad2deg(np.array([roll, pitch, yaw], dtype=np.float64))

#     # ============================================================
#     # 时间同步查找
#     # ============================================================
#     def find_closest_depth(self, target_stamp):
#         best_depth = None
#         best_dt = float('inf')

#         for stamp_sec, depth_img in self.depth_buffer:
#             dt = abs(stamp_sec - target_stamp)
#             if dt < best_dt:
#                 best_dt = dt
#                 best_depth = depth_img

#         if best_depth is None or best_dt > self.max_depth_dt:
#             return None, None

#         return best_depth, best_dt

#     def find_closest_odom(self, target_stamp):
#         best_R = None
#         best_T = None
#         best_dt = float('inf')

#         for stamp_sec, R_lio, T_lio in self.odom_buffer:
#             dt = abs(stamp_sec - target_stamp)
#             if dt < best_dt:
#                 best_dt = dt
#                 best_R = R_lio
#                 best_T = T_lio

#         if best_R is None or best_dt > self.max_odom_dt:
#             return None, None, None

#         return best_R, best_T, best_dt

#     # ============================================================
#     # FAST-LIO odom
#     # ============================================================
#     def odom_callback(self, msg):
#         q = msg.pose.pose.orientation
#         t = msg.pose.pose.position

#         quat = [q.x, q.y, q.z, q.w]
#         R_lio = tf_transformations.quaternion_matrix(quat)[:3, :3]
#         T_lio = np.array([t.x, t.y, t.z], dtype=np.float64)

#         if not self.is_finite_vec(T_lio):
#             return

#         stamp_sec = self.stamp_to_sec(msg.header.stamp)
#         self.odom_buffer.append((stamp_sec, R_lio, T_lio))

#     # ============================================================
#     # 深度图
#     # ============================================================
#     def depth_callback(self, msg):
#         try:
#             depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
#         except Exception as e:
#             self.throttled_warn("depth_convert", f"Depth convert error: {e}")
#             return

#         stamp_sec = self.stamp_to_sec(msg.header.stamp)
#         self.depth_buffer.append((stamp_sec, depth_image))

#     # ============================================================
#     # 相机内参
#     # ============================================================
#     def info_callback(self, msg):
#         self.fx = float(msg.k[0])
#         self.fy = float(msg.k[4])
#         self.cx = float(msg.k[2])
#         self.cy = float(msg.k[5])

#     # ============================================================
#     # 从深度图反投影出 Pc
#     # ============================================================
#     def pixel_to_pc(self, u, v, depth_image):
#         h, w = depth_image.shape

#         if u < 3 or v < 3 or u >= w - 3 or v >= h - 3:
#             return None

#         # 7x7 深度窗口，过滤 NaN / inf / 非法深度
#         kernel = depth_image[v - 3:v + 4, u - 3:u + 4].astype(np.float64)
#         valid = kernel[np.isfinite(kernel)]
#         valid = valid[(valid > self.min_depth) & (valid < self.max_depth)]

#         if valid.size < 8:
#             return None

#         # 用 median 抗坏深度点
#         Z = float(np.median(valid))

#         if not np.isfinite(Z) or Z <= 0:
#             return None

#         X = (float(u) - self.cx) * Z / self.fx
#         Y = (float(v) - self.cy) * Z / self.fy

#         Pc = np.array([X, Y, Z], dtype=np.float64)

#         if not self.is_finite_vec(Pc):
#             return None

#         # Pc 小窗口平均，降低噪声
#         self.pc_buffer.append(Pc)
#         Pc_smooth = np.mean(np.array(self.pc_buffer), axis=0)

#         return Pc_smooth

#     # ============================================================
#     # 根据当前 FAST-LIO 位姿，把 FAST-LIO2 系真实点反算到 body/lidar 坐标系
#     #
#     # P_lio_true = R_lio @ Pb_true + T_lio
#     # Pb_true = R_lio.T @ (P_lio_true - T_lio)
#     # ============================================================
#     def compute_pb_true(self, R_lio, T_lio):
#         Pb_true = R_lio.T @ (self.P_lio_true - T_lio)
#         return Pb_true

#     # ============================================================
#     # 初步坏值过滤
#     # ============================================================
#     def accept_raw_sample(self, Pc, Pb_true):
#         if not self.is_finite_vec(Pc) or not self.is_finite_vec(Pb_true):
#             return False, "non_finite"

#         if np.linalg.norm(Pc) < 0.03:
#             return False, "pc_too_small"

#         if np.linalg.norm(Pb_true) > 20.0:
#             return False, "pb_true_too_far"

#         # 在没有好外参之前，Pc 和 Pb_true 的距离不应该离谱到好几米
#         # 这里是粗筛，主要干掉深度跳变或 odom 对错时间的点
#         raw_dist = np.linalg.norm(Pc - Pb_true)
#         if raw_dist > self.raw_pair_max_dist:
#             return False, f"raw_dist_too_large={raw_dist:.3f}"

#         return True, "ok"

#     # ============================================================
#     # SVD 刚体配准：求 Pb ≈ R_cb @ Pc + t_cb
#     # ============================================================
#     def solve_rigid_transform(self, Pc_list, Pb_list):
#         A = np.asarray(Pc_list, dtype=np.float64)
#         B = np.asarray(Pb_list, dtype=np.float64)

#         if A.shape[0] < 3:
#             return None, None

#         centroid_A = np.mean(A, axis=0)
#         centroid_B = np.mean(B, axis=0)

#         AA = A - centroid_A
#         BB = B - centroid_B

#         H = AA.T @ BB

#         try:
#             U, S, Vt = np.linalg.svd(H)
#         except np.linalg.LinAlgError:
#             return None, None

#         R = Vt.T @ U.T

#         # 防止反射
#         if np.linalg.det(R) < 0:
#             Vt[-1, :] *= -1.0
#             R = Vt.T @ U.T

#         t = centroid_B - R @ centroid_A

#         return R, t

#     # ============================================================
#     # 鲁棒优化外参：先全量求，再按残差/MAD 过滤，再重求
#     # ============================================================
#     def optimize_extrinsic(self):
#         if len(self.samples) < self.min_samples_to_opt:
#             return False

#         Pc_all = [s[0] for s in self.samples]
#         Pb_all = [s[1] for s in self.samples]

#         R0, t0 = self.solve_rigid_transform(Pc_all, Pb_all)
#         if R0 is None:
#             return False

#         residuals = []
#         for Pc, Pb_true in self.samples:
#             pred = R0 @ Pc + t0
#             residuals.append(np.linalg.norm(pred - Pb_true))

#         residuals = np.asarray(residuals, dtype=np.float64)

#         med = float(np.median(residuals))
#         mad = float(np.median(np.abs(residuals - med))) + 1e-9

#         # MAD 门限 + 固定最大门限，越优化越严格
#         mad_gate = med + self.mad_k * 1.4826 * mad
#         gate = min(max(mad_gate, 0.08), self.residual_gate_start)

#         keep_indices = np.where(residuals <= gate)[0]

#         if keep_indices.size < self.min_keep_after_filter:
#             # 如果过滤后太少，就暂时接受初始解
#             self.R_cb = R0
#             self.t_cb = t0
#             self.optimized_count += 1
#             return True

#         Pc_keep = [Pc_all[i] for i in keep_indices]
#         Pb_keep = [Pb_all[i] for i in keep_indices]

#         R1, t1 = self.solve_rigid_transform(Pc_keep, Pb_keep)
#         if R1 is None:
#             return False

#         # 更新外参
#         self.R_cb = R1
#         self.t_cb = t1
#         self.optimized_count += 1

#         # 把明显坏样本从样本池里删掉，防止污染后续优化
#         new_samples = [self.samples[i] for i in keep_indices]

#         # 限制样本数量，只保留最近的好样本
#         if len(new_samples) > self.max_samples:
#             new_samples = new_samples[-self.max_samples:]

#         self.samples = new_samples

#         return True

#     # ============================================================
#     # 目标点回调
#     # ============================================================
#     def point_callback(self, msg):
#         if not self.enabled:
#             return
#         self.total_points += 1

#         if self.fx is None:
#             self.throttled_warn("no_camera_info", "No camera_info yet")
#             return

#         target_stamp = self.stamp_to_sec(msg.header.stamp)

#         # 1) 找最近深度
#         depth_image, depth_dt = self.find_closest_depth(target_stamp)
#         if depth_image is None:
#             self.rejected_points += 1
#             self.throttled_warn("no_depth", "No matched depth frame for target_point")
#             return

#         # 2) 找最近 odom
#         R_lio, T_lio, odom_dt = self.find_closest_odom(target_stamp)
#         if R_lio is None:
#             self.rejected_points += 1
#             self.throttled_warn("no_odom", "No matched odom for target_point")
#             return

#         u = int(msg.point.x)
#         v = int(msg.point.y)

#         Pc = self.pixel_to_pc(u, v, depth_image)
#         if Pc is None:
#             self.rejected_points += 1
#             return

#         # FAST-LIO2真实目标点反算到 body/lidar 系
#         Pb_true = self.compute_pb_true(R_lio, T_lio)

#         ok, reason = self.accept_raw_sample(Pc, Pb_true)
#         if not ok:
#             self.rejected_points += 1
#             self.throttled_warn("reject_raw", f"Reject raw sample: {reason}")
#             return

#         # 如果已经有外参了，再用当前外参残差过滤一次
#         Pb_pred_now = self.R_cb @ Pc + self.t_cb
#         residual_body_now = float(np.linalg.norm(Pb_pred_now - Pb_true))

#         if len(self.samples) >= self.min_samples_to_opt and residual_body_now > self.residual_gate_start:
#             self.rejected_points += 1
#             self.throttled_warn(
#                 "reject_residual",
#                 f"Reject sample by residual: {residual_body_now:.3f} m"
#             )
#             return

#         # 接受样本
#         self.samples.append((Pc.copy(), Pb_true.copy()))
#         if len(self.samples) > self.max_samples:
#             self.samples = self.samples[-self.max_samples:]

#         self.accepted_points += 1

#         # 每输入新样本就优化一次
#         self.optimize_extrinsic()

#         # 当前外参下的结果
#         Pb_est = self.R_cb @ Pc + self.t_cb
#         P_lio_est = R_lio @ Pb_est + T_lio
#         err_lio = P_lio_est - self.P_lio_true
#         err_norm = float(np.linalg.norm(err_lio))

#         # 发布
#         out_stamp = msg.header.stamp

#         p_lio = PointStamped()
#         p_lio.header.stamp = out_stamp
#         p_lio.header.frame_id = "fastlio2"
#         p_lio.point.x = float(P_lio_est[0])
#         p_lio.point.y = float(P_lio_est[1])
#         p_lio.point.z = float(P_lio_est[2])
#         self.lio_pub.publish(p_lio)

#         p_cam = PointStamped()
#         p_cam.header.stamp = out_stamp
#         p_cam.header.frame_id = "camera"
#         p_cam.point.x = float(Pc[0])
#         p_cam.point.y = float(Pc[1])
#         p_cam.point.z = float(Pc[2])
#         self.cam_pub.publish(p_cam)

#         p_body_true = PointStamped()
#         p_body_true.header.stamp = out_stamp
#         p_body_true.header.frame_id = "body_lidar"
#         p_body_true.point.x = float(Pb_true[0])
#         p_body_true.point.y = float(Pb_true[1])
#         p_body_true.point.z = float(Pb_true[2])
#         self.body_true_pub.publish(p_body_true)

#         self.lio_calib_pub.publish(p_lio)

#         # 保存给低频 timer 打印
#         self.last_status = {
#             "depth_dt_ms": depth_dt * 1000.0,
#             "odom_dt_ms": odom_dt * 1000.0,
#             "Pc": Pc.copy(),
#             "Pb_true": Pb_true.copy(),
#             "Pb_est": Pb_est.copy(),
#             "P_lio_true": self.P_lio_true.copy(),
#             "P_lio_est": P_lio_est.copy(),
#             "err_lio": err_lio.copy(),
#             "err_norm": err_norm,
#             "residual_body": residual_body_now,
#             "samples": len(self.samples),
#             "accepted": self.accepted_points,
#             "rejected": self.rejected_points,
#         }

#     # ============================================================
#     # 低频打印，不刷屏
#     # ============================================================
#     def print_status_timer(self):
#         if self.last_status is None:
#             return

#         s = self.last_status

#         rpy_deg = self.rot_to_rpy_deg(self.R_cb)

#         self.get_logger().info(
#             "\n"
#             "---------------- Online Extrinsic Calib ----------------\n"
#             f"samples={s['samples']} | accepted={s['accepted']} | rejected={s['rejected']} | "
#             f"optimized={self.optimized_count}\n"
#             f"sync: depth_dt={s['depth_dt_ms']:.1f} ms, odom_dt={s['odom_dt_ms']:.1f} ms\n"
#             f"Pc(camera)      = {np.round(s['Pc'], 4)}\n"
#             f"Pb_true(body)   = {np.round(s['Pb_true'], 4)}\n"
#             f"Pb_est(body)    = {np.round(s['Pb_est'], 4)}\n"
#             f"P_lio_true(world)  = {np.round(s['P_lio_true'], 4)}\n"
#             f"P_lio_est(world)   = {np.round(s['P_lio_est'], 4)}\n"
#             f"err_lio       = {np.round(s['err_lio'], 4)} | norm={s['err_norm']:.4f} m\n"
#             f"R_cb rpy(deg)   = roll={rpy_deg[0]:.3f}, pitch={rpy_deg[1]:.3f}, yaw={rpy_deg[2]:.3f}\n"
#             f"t_cb(m)         = {np.round(self.t_cb, 6)}\n"
#             "--------------------------------------------------------"
#         )

#     # ============================================================
#     # 程序结束时打印最终最优参数
#     # ============================================================
#     def print_final_result(self):
#         self.get_logger().info("\n\n")
#         self.get_logger().info("============================================================")
#         self.get_logger().info("FINAL ONLINE EXTRINSIC CALIBRATION RESULT")
#         self.get_logger().info("Pc(camera) -> Pb(body/lidar)")
#         self.get_logger().info("============================================================")

#         if len(self.samples) < self.min_samples_to_opt:
#             self.get_logger().warn(
#                 f"Not enough valid samples: {len(self.samples)} / {self.min_samples_to_opt}"
#             )

#         # 结束前再优化一次
#         self.optimize_extrinsic()

#         rpy_deg = self.rot_to_rpy_deg(self.R_cb)

#         self.get_logger().info("R_cb = np.array([")
#         self.get_logger().info(f"    [{self.R_cb[0,0]: .9f}, {self.R_cb[0,1]: .9f}, {self.R_cb[0,2]: .9f}],")
#         self.get_logger().info(f"    [{self.R_cb[1,0]: .9f}, {self.R_cb[1,1]: .9f}, {self.R_cb[1,2]: .9f}],")
#         self.get_logger().info(f"    [{self.R_cb[2,0]: .9f}, {self.R_cb[2,1]: .9f}, {self.R_cb[2,2]: .9f}],")
#         self.get_logger().info("], dtype=np.float64)")

#         self.get_logger().info("")
#         self.get_logger().info(
#             f"t_cb = np.array([{self.t_cb[0]: .9f}, {self.t_cb[1]: .9f}, {self.t_cb[2]: .9f}], dtype=np.float64)"
#         )

#         self.get_logger().info("")
#         self.get_logger().info(
#             f"RPY degree = roll={rpy_deg[0]:.6f}, pitch={rpy_deg[1]:.6f}, yaw={rpy_deg[2]:.6f}"
#         )

#         if len(self.samples) > 0:
#             residuals = []
#             for Pc, Pb_true in self.samples:
#                 Pb_est = self.R_cb @ Pc + self.t_cb
#                 residuals.append(np.linalg.norm(Pb_est - Pb_true))

#             residuals = np.asarray(residuals, dtype=np.float64)

#             self.get_logger().info("")
#             self.get_logger().info(
#                 f"Residual body error: mean={np.mean(residuals):.6f} m, "
#                 f"median={np.median(residuals):.6f} m, "
#                 f"max={np.max(residuals):.6f} m"
#             )

#         self.get_logger().info("")
#         self.get_logger().info(f"total_points    = {self.total_points}")
#         self.get_logger().info(f"accepted_points = {self.accepted_points}")
#         self.get_logger().info(f"rejected_points = {self.rejected_points}")
#         self.get_logger().info(f"final_samples   = {len(self.samples)}")
#         self.get_logger().info("============================================================")
#         self.get_logger().info("Copy R_cb and t_cb back into your normal vision node.")
#         self.get_logger().info("============================================================")


# def main(args=None):
#     rclpy.init(args=args)
#     node = Positionpub_Node()

#     def handle_sigint(sig, frame):
#         node.print_final_result()
#         node.destroy_node()
#         rclpy.shutdown()
#         sys.exit(0)

#     signal.signal(signal.SIGINT, handle_sigint)

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.print_final_result()
#     finally:
#         try:
#             node.destroy_node()
#         except Exception:
#             pass
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == '__main__':
#     main()




















import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from collections import deque
import numpy as np
import tf_transformations


# 订阅：
#   /vision/front/enable      std_msgs/msg/Bool
#   /target_point             geometry_msgs/msg/PointStamped
#   /d435/depth               sensor_msgs/msg/Image
#   /d435/camera_info         sensor_msgs/msg/CameraInfo
#   /fastlio2/lio_odom        nav_msgs/msg/Odometry
#
# 发布：
#   /vision/front/goal_position   geometry_msgs/msg/PointStamped
#
# 作用：
#   使用已经标定好的外参，把 D435 图像目标点转换到 FAST-LIO2 世界坐标系。
#
# 坐标链路：
#   像素点 + 深度图 -> Pc(camera)
#   Pb(body/lidar)   = R_cb @ Pc + t_cb
#   P_world(fastlio) = R_lio @ Pb + T_lio


class Positionpub_Node(Node):
    def __init__(self):
        super().__init__('Positionpub_Node')

        self.bridge = CvBridge()

        # ============================================================
        # 相机内参
        # ============================================================
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # ============================================================
        # 已标定外参：Pc(camera) -> Pb(body/lidar)
        #
        # Pb = R_cb @ Pc + t_cb
        # ============================================================
        self.R_cb = np.array([
            [ 0.992015407,  0.095560730, -0.082301761],
            [ 0.124280162, -0.629755927,  0.766786746],
            [ 0.021444680, -0.770892741, -0.636603886],
        ], dtype=np.float64)

        self.t_cb = np.array([0.029626960, 0.243183442, 0.127970307], dtype=np.float64)

        # ============================================================
        # 缓存
        # ============================================================
        self.depth_buffer = deque(maxlen=30)      # (stamp_sec, depth_image)
        self.odom_buffer = deque(maxlen=200)      # (stamp_sec, R_lio, T_lio)

        # Pc 低通缓存，降低单帧深度抖动
        self.pc_buffer = deque(maxlen=5)

        # ============================================================
        # 参数
        # ============================================================
        self.max_depth_dt = 0.40      # depth 约 3Hz，允许 400ms 匹配窗口
        self.max_odom_dt = 0.20       # FAST-LIO odom 一般更快，先给 200ms

        self.min_depth = 0.05
        self.max_depth = 8.0

        self.print_period = 1.0
        self.warn_period = 2.0
        self.last_warn_time = {}

        self.enabled = True
        self.last_world = None

        # ============================================================
        # ROS 订阅
        # ============================================================
        self.create_subscription(Bool, '/vision/front/enable', self.enable_callback, 10)
        self.create_subscription(PointStamped, '/target_point', self.point_callback, 10)
        self.create_subscription(Image, '/d435/depth', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/d435/camera_info', self.info_callback, 10)
        self.create_subscription(Odometry, '/fastlio2/lio_odom', self.odom_callback, 50)

        # ============================================================
        # ROS 发布
        # ============================================================
        self.lio_pub = self.create_publisher(PointStamped, '/vision/front/goal_position', 10)

        # 只打印当前外参算出的世界坐标
        self.create_timer(self.print_period, self.print_world_timer)

        self.get_logger().info("Positionpub world-only node started")
        self.get_logger().info("Publish target world position to /vision/front/goal_position")

    # ============================================================
    # 工具函数
    # ============================================================
    def enable_callback(self, msg):
        self.enabled = bool(msg.data)

    @staticmethod
    def stamp_to_sec(stamp_msg):
        return float(stamp_msg.sec) + float(stamp_msg.nanosec) * 1e-9

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def throttled_warn(self, key, text):
        now = self.now_sec()
        last = self.last_warn_time.get(key, -1e9)
        if now - last >= self.warn_period:
            self.get_logger().warn(text)
            self.last_warn_time[key] = now

    @staticmethod
    def is_finite_vec(v):
        return np.all(np.isfinite(v))

    # ============================================================
    # 时间同步查找
    # ============================================================
    def find_closest_depth(self, target_stamp):
        best_depth = None
        best_dt = float('inf')

        for stamp_sec, depth_img in self.depth_buffer:
            dt = abs(stamp_sec - target_stamp)
            if dt < best_dt:
                best_dt = dt
                best_depth = depth_img

        if best_depth is None or best_dt > self.max_depth_dt:
            return None

        return best_depth

    def find_closest_odom(self, target_stamp):
        best_R = None
        best_T = None
        best_dt = float('inf')

        for stamp_sec, R_lio, T_lio in self.odom_buffer:
            dt = abs(stamp_sec - target_stamp)
            if dt < best_dt:
                best_dt = dt
                best_R = R_lio
                best_T = T_lio

        if best_R is None or best_dt > self.max_odom_dt:
            return None, None

        return best_R, best_T

    # ============================================================
    # FAST-LIO odom
    # ============================================================
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        t = msg.pose.pose.position

        quat = [q.x, q.y, q.z, q.w]
        R_lio = tf_transformations.quaternion_matrix(quat)[:3, :3]
        T_lio = np.array([t.x, t.y, t.z], dtype=np.float64)

        if not self.is_finite_vec(T_lio):
            return

        stamp_sec = self.stamp_to_sec(msg.header.stamp)
        self.odom_buffer.append((stamp_sec, R_lio, T_lio))

    # ============================================================
    # 深度图
    # ============================================================
    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.throttled_warn("depth_convert", f"Depth convert error: {e}")
            return

        stamp_sec = self.stamp_to_sec(msg.header.stamp)
        self.depth_buffer.append((stamp_sec, depth_image))

    # ============================================================
    # 相机内参
    # ============================================================
    def info_callback(self, msg):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    # ============================================================
    # 从深度图反投影出 Pc
    # ============================================================
    def pixel_to_pc(self, u, v, depth_image):
        h, w = depth_image.shape

        if u < 3 or v < 3 or u >= w - 3 or v >= h - 3:
            return None

        kernel = depth_image[v - 3:v + 4, u - 3:u + 4].astype(np.float64)
        valid = kernel[np.isfinite(kernel)]
        valid = valid[(valid > self.min_depth) & (valid < self.max_depth)]

        if valid.size < 8:
            return None

        Z = float(np.median(valid))

        if not np.isfinite(Z) or Z <= 0:
            return None

        X = (float(u) - self.cx) * Z / self.fx
        Y = (float(v) - self.cy) * Z / self.fy

        Pc = np.array([X, Y, Z], dtype=np.float64)

        if not self.is_finite_vec(Pc):
            return None

        self.pc_buffer.append(Pc)
        Pc_smooth = np.mean(np.array(self.pc_buffer), axis=0)

        return Pc_smooth

    # ============================================================
    # 目标点回调
    # ============================================================
    def point_callback(self, msg):
        if not self.enabled:
            return

        if self.fx is None:
            self.throttled_warn("no_camera_info", "No camera_info yet")
            return

        target_stamp = self.stamp_to_sec(msg.header.stamp)

        depth_image = self.find_closest_depth(target_stamp)
        if depth_image is None:
            self.throttled_warn("no_depth", "No matched depth frame for target_point")
            return

        R_lio, T_lio = self.find_closest_odom(target_stamp)
        if R_lio is None:
            self.throttled_warn("no_odom", "No matched odom for target_point")
            return

        u = int(msg.point.x)
        v = int(msg.point.y)

        Pc = self.pixel_to_pc(u, v, depth_image)
        if Pc is None:
            return

        # 当前外参下的目标世界坐标
        Pb_est = self.R_cb @ Pc + self.t_cb
        P_lio_est = R_lio @ Pb_est + T_lio

        if not self.is_finite_vec(P_lio_est):
            return

        p_lio = PointStamped()
        p_lio.header.stamp = msg.header.stamp
        p_lio.header.frame_id = "fastlio2"
        p_lio.point.x = float(P_lio_est[0])
        p_lio.point.y = float(P_lio_est[1])
        p_lio.point.z = float(P_lio_est[2])

        self.lio_pub.publish(p_lio)

        self.last_world = P_lio_est.copy()

    # ============================================================
    # 只打印世界坐标
    # ============================================================
    def print_world_timer(self):
        if self.last_world is None:
            return

        p = self.last_world
        self.get_logger().info(
            f"[TARGET_WORLD] x={p[0]:.4f}, y={p[1]:.4f}, z={p[2]:.4f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Positionpub_Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()