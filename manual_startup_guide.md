# 多机仿真手动启动指南

## 基于你的单机流程的多机扩展版本

### 🔥 重要提醒
- **双层控制架构**：轨迹控制器 → 姿态控制器 → MAVROS
- **上下飞行配置**：UAV0地面层，UAV1上层（5米）
- **启动顺序很重要**：必须按顺序执行！

---

## 📋 启动步骤

### 第1步：启动PX4和Gazebo
**终端1**（UAV0-PX4-Gazebo）：
```bash
cd /path/to/PX4-Autopilot
make px4_sitl gz_x500
```
⏱️ 等待Gazebo完全启动（约10秒）

### 第2步：启动第二架飞机
**终端2**（UAV1-PX4）：
```bash
cd /path/to/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,0,5" ./build/px4_sitl_default/bin/px4 -i 1
```
⏱️ 等待第二架飞机启动（约5秒）

### 第3步：启动MAVROS通信
**终端3**（UAV0-MAVROS）：
```bash
export ROS_DOMAIN_ID=0
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14555 namespace:=uav0
```

**终端4**（UAV1-MAVROS）：
```bash
export ROS_DOMAIN_ID=0
ros2 launch mavros px4.launch fcu_url:=udp://:14541@127.0.0.1:14565 namespace:=uav1
```

### 第4步：启动地面站
**终端5**（QGroundControl）：
```bash
qgroundcontrol
```

---

## 🎮 控制系统启动

### 第5步：进入工作空间并启动控制器

**终端6**（UAV0控制）：
```bash
cd /path/to/your/workspace
export ROS_DOMAIN_ID=0
source install/setup.bash

# 1. 启动姿态控制器（后台）
ros2 run control attitude_controller --ros-args -r __ns:=/uav0 &

# 2. 启动轨迹控制器（后台）
ros2 run control traj_controller_NL --ros-args -r __ns:=/uav0 -p height_offset:=0.0 &

# 3. 设置OFFBOARD模式
ros2 service call /uav0/mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'OFFBOARD'}"

# 4. 启用轨迹跟踪
ros2 param set /uav0/traj_controller_NL traj_mode true

# 5. 解锁飞机
ros2 service call /uav0/mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

**终端7**（UAV1控制）：
```bash
cd /path/to/your/workspace
export ROS_DOMAIN_ID=0
source install/setup.bash

# 1. 启动姿态控制器（后台）
ros2 run control attitude_controller --ros-args -r __ns:=/uav1 &

# 2. 启动轨迹控制器（5米高度偏移）
ros2 run control traj_controller_NL --ros-args -r __ns:=/uav1 -p height_offset:=5.0 &

# 3. 设置OFFBOARD模式
ros2 service call /uav1/mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'OFFBOARD'}"

# 4. 启用轨迹跟踪
ros2 param set /uav1/traj_controller_NL traj_mode true

# 5. 解锁飞机
ros2 service call /uav1/mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

---

## 🔍 监控和调试

**终端8**（监控）：
```bash
export ROS_DOMAIN_ID=0

# 检查话题连接
ros2 topic list | grep -E "(uav0|uav1)"

# 监控飞机位置
ros2 topic echo /uav0/mavros/local_position/pose --field pose.position &
ros2 topic echo /uav1/mavros/local_position/pose --field pose.position &

# 监控控制指令
ros2 topic echo /uav0/control/attitude --field thrust &
ros2 topic echo /uav1/control/attitude --field thrust &

# 查看参数
ros2 param list /uav0/attitude_controller
ros2 param list /uav1/traj_controller_NL
```

---

## ⚠️ 关键注意事项

### 启动顺序（不能乱！）
1. ✅ PX4 + Gazebo
2. ✅ 第二架PX4
3. ✅ 两个MAVROS
4. ✅ 地面站
5. ✅ 姿态控制器（两个）
6. ✅ 轨迹控制器（两个）
7. ✅ OFFBOARD模式
8. ✅ 轨迹跟踪参数
9. ✅ 解锁

### 常见问题解决
- **飞机连接失败**：检查端口是否冲突
- **控制器无响应**：确认命名空间设置正确
- **轨迹不执行**：确认`traj_mode`参数设置为true
- **高度不对**：检查`height_offset`参数

### 数据记录
日志文件自动保存在：`/home/swarm/wz/Log/`
- `sliding_control_log_uav0_*.csv`
- `sliding_control_log_uav1_*.csv`  
- `attitude_control_log_uav0_*.csv`
- `attitude_control_log_uav1_*.csv`

---

## 🎯 预期效果

- **UAV0**：在地面高度执行轨迹
- **UAV1**：在5米高度执行相同轨迹路径
- **垂直对齐**：两机保持垂直位置关系
- **气动干扰**：可通过日志数据分析相互影响