#!/bin/bash

# 完整多机仿真启动脚本
# 基于用户的单机流程扩展为双机上下飞行仿真
# 用途：研究两架无人机上下飞行时的气动干扰

echo "=========================================="
echo "        多机仿真启动脚本 v2.0"
echo "   双层控制架构 + 上下飞行配置"
echo "=========================================="

# 配置参数 - 请根据你的环境修改这些路径
PX4_PATH="/home/$(whoami)/PX4-Autopilot"  # 修改为你的PX4路径
WORKSPACE_PATH="/home/$(whoami)/your_workspace"  # 修改为你的工作空间路径
QGC_PATH="qgroundcontrol"  # QGroundControl路径，如果在PATH中则直接写命令名

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 检查路径
check_paths() {
    echo -e "${YELLOW}检查环境路径...${NC}"
    
    if [ ! -d "$PX4_PATH" ]; then
        echo -e "${RED}错误：PX4路径不存在: $PX4_PATH${NC}"
        echo "请修改脚本中的PX4_PATH变量"
        exit 1
    fi
    
    if [ ! -d "$WORKSPACE_PATH" ]; then
        echo -e "${RED}错误：工作空间路径不存在: $WORKSPACE_PATH${NC}"
        echo "请修改脚本中的WORKSPACE_PATH变量"
        exit 1
    fi
    
    echo -e "${GREEN}路径检查通过${NC}"
}

# 启动函数
start_simulation() {
    echo -e "${GREEN}=== 第1步：启动Gazebo和UAV0（下层飞机） ===${NC}"
    gnome-terminal --title="UAV0-PX4-Gazebo" --geometry=80x24+0+0 -- bash -c "
        cd $PX4_PATH
        echo -e '${CYAN}启动下层无人机(UAV0)和Gazebo环境...${NC}'
        make px4_sitl gz_x500
        exec bash
    "
    
    echo "等待Gazebo启动完成..."
    sleep 10
    
    echo -e "${GREEN}=== 第2步：启动UAV1（上层飞机） ===${NC}"
    gnome-terminal --title="UAV1-PX4" --geometry=80x24+800+0 -- bash -c "
        cd $PX4_PATH
        echo -e '${CYAN}启动上层无人机(UAV1)...${NC}'
        echo '注意：初始高度为5米'
        PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE='0,0,5' ./build/px4_sitl_default/bin/px4 -i 1
        exec bash
    "
    
    echo "等待第二架飞机启动..."
    sleep 8
    
    echo -e "${GREEN}=== 第3步：启动MAVROS for UAV0 ===${NC}"
    gnome-terminal --title="UAV0-MAVROS" --geometry=80x24+0+300 -- bash -c "
        export ROS_DOMAIN_ID=0
        echo -e '${CYAN}启动UAV0的MAVROS通信...${NC}'
        echo 'UAV0 MAVROS: udp://:14540@127.0.0.1:14555'
        ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14555 namespace:=uav0
        exec bash
    "
    
    sleep 3
    
    echo -e "${GREEN}=== 第4步：启动MAVROS for UAV1 ===${NC}"
    gnome-terminal --title="UAV1-MAVROS" --geometry=80x24+800+300 -- bash -c "
        export ROS_DOMAIN_ID=0
        echo -e '${CYAN}启动UAV1的MAVROS通信...${NC}'
        echo 'UAV1 MAVROS: udp://:14541@127.0.0.1:14565'
        ros2 launch mavros px4.launch fcu_url:=udp://:14541@127.0.0.1:14565 namespace:=uav1
        exec bash
    "
    
    sleep 3
    
    echo -e "${GREEN}=== 第5步：启动地面站 ===${NC}"
    gnome-terminal --title="QGroundControl" --geometry=80x24+1600+0 -- bash -c "
        echo -e '${CYAN}启动QGroundControl地面站...${NC}'
        echo '连接信息：'
        echo 'UAV0: TCP连接 127.0.0.1:5760'
        echo 'UAV1: TCP连接 127.0.0.1:5761'
        $QGC_PATH
        exec bash
    "
    
    sleep 2
    
    echo -e "${GREEN}=== 第6步：准备控制系统 ===${NC}"
    gnome-terminal --title="UAV0-Controllers" --geometry=120x30+0+600 -- bash -c "
        cd $WORKSPACE_PATH
        export ROS_DOMAIN_ID=0
        source install/setup.bash
        
        echo -e '${PURPLE}=========================================='
        echo '           UAV0 控制系统准备就绪'
        echo '==========================================${NC}'
        echo ''
        echo -e '${YELLOW}第一步：启动姿态控制器${NC}'
        echo -e '${BLUE}ros2 run control attitude_controller --ros-args -r __ns:=/uav0${NC}'
        echo ''
        echo -e '${YELLOW}第二步：启动轨迹控制器${NC}'
        echo -e '${BLUE}ros2 run control traj_controller_NL --ros-args -r __ns:=/uav0 -p height_offset:=0.0${NC}'
        echo ''
        echo -e '${YELLOW}第三步：设置OFFBOARD模式${NC}'
        echo -e '${BLUE}ros2 service call /uav0/mavros/set_mode mavros_msgs/srv/SetMode \"{base_mode: 0, custom_mode: OFFBOARD}\"${NC}'
        echo ''
        echo -e '${YELLOW}第四步：设置轨迹跟踪模式${NC}'
        echo -e '${BLUE}ros2 param set /uav0/traj_controller_NL traj_mode true${NC}'
        echo ''
        echo -e '${YELLOW}第五步：解锁飞机${NC}'
        echo -e '${BLUE}ros2 service call /uav0/mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: true}\"${NC}'
        echo ''
        echo -e '${RED}请按顺序复制粘贴上述命令！${NC}'
        echo ''
        exec bash
    "
    
    gnome-terminal --title="UAV1-Controllers" --geometry=120x30+1200+600 -- bash -c "
        cd $WORKSPACE_PATH
        export ROS_DOMAIN_ID=0
        source install/setup.bash
        
        echo -e '${PURPLE}=========================================='
        echo '           UAV1 控制系统准备就绪'
        echo '==========================================${NC}'
        echo ''
        echo -e '${YELLOW}第一步：启动姿态控制器${NC}'
        echo -e '${BLUE}ros2 run control attitude_controller --ros-args -r __ns:=/uav1${NC}'
        echo ''
        echo -e '${YELLOW}第二步：启动轨迹控制器${NC}'
        echo -e '${BLUE}ros2 run control traj_controller_NL --ros-args -r __ns:=/uav1 -p height_offset:=5.0${NC}'
        echo ''
        echo -e '${YELLOW}第三步：设置OFFBOARD模式${NC}'
        echo -e '${BLUE}ros2 service call /uav1/mavros/set_mode mavros_msgs/srv/SetMode \"{base_mode: 0, custom_mode: OFFBOARD}\"${NC}'
        echo ''
        echo -e '${YELLOW}第四步：设置轨迹跟踪模式${NC}'
        echo -e '${BLUE}ros2 param set /uav1/traj_controller_NL traj_mode true${NC}'
        echo ''
        echo -e '${YELLOW}第五步：解锁飞机${NC}'
        echo -e '${BLUE}ros2 service call /uav1/mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: true}\"${NC}'
        echo ''
        echo -e '${RED}请按顺序复制粘贴上述命令！${NC}'
        echo ''
        exec bash
    "
    
    sleep 2
    
    echo -e "${GREEN}=== 第7步：监控和调试终端 ===${NC}"
    gnome-terminal --title="Monitor-Debug" --geometry=100x25+400+900 -- bash -c "
        export ROS_DOMAIN_ID=0
        echo -e '${PURPLE}=========================================='
        echo '           监控和调试终端'
        echo '==========================================${NC}'
        echo ''
        echo -e '${YELLOW}常用监控命令：${NC}'
        echo -e '${CYAN}# 查看话题列表${NC}'
        echo 'ros2 topic list | grep -E \"(uav0|uav1)\"'
        echo ''
        echo -e '${CYAN}# 监控飞机位置${NC}'
        echo 'ros2 topic echo /uav0/mavros/local_position/pose --field pose.position'
        echo 'ros2 topic echo /uav1/mavros/local_position/pose --field pose.position'
        echo ''
        echo -e '${CYAN}# 监控控制指令${NC}'
        echo 'ros2 topic echo /uav0/control/attitude --field thrust'
        echo 'ros2 topic echo /uav1/control/attitude --field thrust'
        echo ''
        echo -e '${CYAN}# 参数调整示例${NC}'
        echo 'ros2 param set /uav0/attitude_controller P_gain \"[8.0, 8.0, 2.0]\"'
        echo 'ros2 param set /uav1/attitude_controller P_gain \"[10.0, 10.0, 3.0]\"'
        echo ''
        echo -e '${CYAN}# 查看日志文件${NC}'
        echo 'ls /home/swarm/wz/Log/'
        echo ''
        exec bash
    "
}

# 显示启动后操作指南
show_operation_guide() {
    echo ""
    echo -e "${PURPLE}=========================================="
    echo "            启动完成 - 操作指南"
    echo "==========================================${NC}"
    echo ""
    echo -e "${YELLOW}启动顺序（重要！）：${NC}"
    echo "1. 等待所有组件启动完成（约20-30秒）"
    echo "2. 在地面站检查两架飞机连接状态"
    echo "3. 先启动姿态控制器（两个终端）"
    echo "4. 再启动轨迹控制器（两个终端）"
    echo "5. 设置OFFBOARD模式"
    echo "6. 启用轨迹跟踪"
    echo "7. 解锁飞机"
    echo ""
    echo -e "${GREEN}预期效果：${NC}"
    echo "• UAV0：在地面高度执行轨迹"
    echo "• UAV1：在5米高度执行相同轨迹"
    echo "• 两机垂直对齐，可观察上下气动干扰"
    echo ""
    echo -e "${RED}注意事项：${NC}"
    echo "• 先确保单机能正常飞行"
    echo "• 观察日志文件中的控制性能"
    echo "• 必要时调整控制增益参数"
    echo "• 紧急情况下手动在地面站切换模式"
    echo ""
    echo -e "${CYAN}日志位置：/home/swarm/wz/Log/${NC}"
    echo ""
}

# 主函数
main() {
    check_paths
    echo ""
    echo -e "${YELLOW}即将启动多机仿真，请确保：${NC}"
    echo "1. 已编译PX4和你的控制代码"
    echo "2. 所有路径配置正确"
    echo "3. 没有其他Gazebo实例在运行"
    echo ""
    read -p "按Enter继续，或Ctrl+C取消..."
    echo ""
    
    start_simulation
    show_operation_guide
    
    echo -e "${GREEN}多机仿真启动脚本执行完成！${NC}"
    echo "请按照上述指南操作各个终端。"
}

# 执行主函数
main