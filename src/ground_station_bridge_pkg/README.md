# ground_station_bridge_pkg

`ground_station_bridge_pkg` 是一个运行在 Jetson 上的 **ROS2 地面站通信桥接包**，用于在 **PX4/ROS2 系统** 与 **地面站无线串口链路** 之间建立中间桥梁。

当前版本的目标是先完成 **ROS 端基础遥测采集与协议编码**，为后续接入：

- 无线串口模块
- 单片机地面站
- 串口屏显示
- 地面站控制命令回传

打下基础。

---

## 1. 当前功能

当前版本已经实现：

- 订阅 PX4 基础状态 topic
- 缓存飞行器当前遥测信息
- 周期性编码为地面站可用的文本协议字符串
- 可选通过 Linux 串口发送
- 可选从串口接收原始数据并转发为 ROS topic

---

## 2. 当前已完成的订阅

当前节点已经订阅以下 PX4 话题：

### `/fmu/out/vehicle_local_position`
用于获取本地位置与速度：

- `x`
- `y`
- `z`
- `vx`
- `vy`
- `vz`
- `xy_valid`
- `z_valid`

当前会将这些字段写入内部遥测缓存。

---

### `/fmu/out/vehicle_attitude`
用于获取飞行器姿态四元数，并计算：

- `yaw`

当前只提取偏航角用于地面站显示。

---

### `/fmu/out/vehicle_status_v1`
用于获取飞行器状态：

- `arming_state`
- `nav_state`

当前可用于显示：

- 是否解锁
- 当前飞行模式状态编号

---

## 3. 当前遥测输出内容

当前节点会将已缓存数据编码为如下文本协议格式：

```text
$STAT,lp=1,att=1,st=1,arm=2,nav=14,x=0.00,y=0.00,z=-1.02,vx=0.00,vy=0.00,vz=0.00,yaw=1.57,task=IDLE,wp=0/0*CS


字段说明：

lp：本地位置是否有效
att：姿态是否有效
st：状态是否有效
arm：解锁状态
nav：飞行状态编号
x y z：当前位置
vx vy vz：当前速度
yaw：当前偏航角
task：当前任务名（当前版本为占位）
wp：当前航点编号 / 总航点数（当前版本为占位）

协议校验当前使用：

XOR 异或校验



telemetry_data.hpp

定义遥测缓存结构体 TelemetryData，用于统一保存：

位置
速度
偏航角
飞行状态
任务状态占位信息
protocol.hpp / protocol.cpp

实现地面站文本协议编码逻辑：

遥测字符串拼接
XOR 校验生成

当前主要接口：

Protocol::encode_status(const TelemetryData&)
serial_port.hpp / serial_port.cpp

实现 Linux 串口基础封装：

打开串口
配置波特率
写数据
读数据
关闭串口

当前支持常见波特率：

9600
19200
38400
57600
115200
ground_station_bridge_node.hpp / .cpp

主桥接节点，负责：

订阅 PX4 状态
更新内部遥测缓存
定时编码并输出遥测帧
可选通过串口发送
可选从串口读取原始数据并发布到 ROS
7. 节点说明

节点名称：

ground_station_bridge_node
8. 订阅与发布关系
订阅
/fmu/out/vehicle_local_position
/fmu/out/vehicle_attitude
/fmu/out/vehicle_status_v1
发布
/ground_station/cmd

说明：

当前 /ground_station/cmd 主要用于发布串口收到的原始命令字符串
该话题目前只是“上行入口”，尚未接入任务控制器
9. 参数说明

当前节点支持以下参数：

serial_device

串口设备路径。

默认值：

/dev/ttyUSB0
baudrate

串口波特率。

默认值：

115200
enable_serial

是否启用串口收发。

默认值：

false

说明：

false：仅运行 ROS 端逻辑，不打开串口
true：尝试打开串口并进行发送/接收
print_tx

是否在终端打印发送帧。

默认值：

true
10. 编译方法

在工作空间根目录执行：

cd ~/px4_ros2_ws
colcon build --packages-select ground_station_bridge_pkg
source install/setup.bash
11. 运行方法
仅运行 ROS 端调试模式

不打开串口，仅查看遥测帧是否正常生成：

ros2 run ground_station_bridge_pkg ground_station_bridge_node --ros-args -p enable_serial:=false -p print_tx:=true
启用串口发送

例如使用 /dev/ttyUSB0：

ros2 run ground_station_bridge_pkg ground_station_bridge_node --ros-args \
  -p enable_serial:=true \
  -p serial_device:=/dev/ttyUSB0 \
  -p baudrate:=115200 \
  -p print_tx:=true
12. 调试建议

建议按以下顺序调试：

第一步：确认 PX4 topic 正常存在
ros2 topic echo /fmu/out/vehicle_local_position
ros2 topic echo /fmu/out/vehicle_attitude
ros2 topic echo /fmu/out/vehicle_status_v1
第二步：运行 bridge 节点

查看终端是否周期性打印类似内容：

TX: $STAT,lp=1,att=1,st=1,arm=2,nav=14,x=0.00,y=0.00,z=-1.02,vx=0.00,vy=0.00,vz=0.00,yaw=1.57,task=IDLE,wp=0/0*XX

重点检查：

飞机移动时 x y z 是否变化
飞机转向时 yaw 是否变化
解锁时 arm 是否变化
模式切换时 nav 是否变化
第三步：串口联调

打开串口后，使用：

串口调试助手
单片机 UART
无线串口透传链路

查看是否能收到完整字符串帧。