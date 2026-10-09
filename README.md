# Volleyball Tracker (排球轨迹追踪与动力学预测系统)

本项目是排球机器人感知与运动规划技术栈的核心算法库，运行在 **Ubuntu (ROS 2 Jazzy)** 环境与 **AMD Ryzen AI 9 365** 工控主机平台。

功能包基于**排球二次方空气动力学阻力模型**与**扩展卡尔曼滤波 (Aero-EKF)**，实现对高速运动排球（80~120 km/h）的三维轨迹滤波平滑、落点位置预测、到点剩余时间倒计时计算以及出界/落界判断。向下无缝桥接拦截规划器（`volleyball_planner`）与机器人底盘/并联 Stewart 击打中枢（`volleyball_controller`）。

---

## 目录
- [一、 系统架构与数据流图](#一-系统架构与数据流图)
- [二、 部署准备：Linux 系统与硬件设置 (SOP)](#二-部署准备linux-系统与硬件设置-sop)
- [三、 标定与模型配置](#三-标定与模型配置)
- [四、 编译与环境构建](#四-编译与环境构建)
- [五、 分级启动与实机调试流程](#五-分级启动与实机调试流程)
- [六、 ROS 2 话题接口字典](#六-ros-2-话题接口字典)
- [七、 常见问题排查 (Troubleshooting)](#七-常见问题排查-troubleshooting)

---

## 一、 系统架构与数据流图

```
[迈德威视长基线双目 (150 FPS, Line 0 硬触发)]
                   │
                   ▼ (camera_optical_frame)
          volleyball_vision
   (极线矫正 LUT + YOLO 识别 + 稀疏三角化)
                   │
                   ▼ /volleyball/position
         volleyball_tracker (本项目)
   (坐标转换到 base_link + Aero-EKF + 弹道预测)
                   │
                   ▼ /volleyball/trajectory_pred (带速度、落点与到点时间)
         volleyball_planner
   (底盘拦截速度解算 + Stewart 法向对准 + 击打倒计时)
                   │
                   ├──> /chassis_command (chassis_controllers)
                   ├──> /stewart_command (volleyball_controller)
                   └──> /volleyball/strike_trigger (volleyball_hub 状态机)
```

---

## 二、 部署准备：Linux 系统与硬件设置 (SOP)

在将工业相机接入上位机前，必须完成以下 Linux 底层系统调优，避免出现高通量数据流丢包或权限闪退。

### 1. 扩充 USB 内存缓冲区（极度重要）
Linux 默认 `usbfs_memory_mb` 仅为 16MB，双目相机在 150 FPS 高通量传输时数秒内就会触发 `ENOMEM` / `LIBUSB_ERROR_NO_MEM`。
* **临时生效（即刻调试）**：
  ```bash
  sudo sh -c 'echo 1024 > /sys/module/usbcore/parameters/usbfs_memory_mb'
  ```
* **永久生效（写入内核引导项）**：
  编辑 `/etc/default/grub`，在 `GRUB_CMDLINE_LINUX_DEFAULT` 追加参数：
  ```bash
  GRUB_CMDLINE_LINUX_DEFAULT="... usbcore.usbfs_memory_mb=1024"
  ```
  执行更新引导并重启：
  ```bash
  sudo update-grub
  ```

### 2. 配置 USB 免 Sudo 权限规则
将迈德威视设备规则安装至 udev：
```bash
sudo cp /home/aerialstewart/Downloads/linuxSDK_V2.1.0.49202602041120/88-mvusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 3. USB 独立根集线器插线要求
两台相机必须连接在工控主机不同的物理 **USB Root Hub** 接口上。连接后执行：
```bash
lsusb -t
```
确认左相机与右相机分别挂载在不同的 `Bus` 节点下，避免争抢单个 USB 3.0 控制器的 5Gbps 带宽。

### 4. 外部 GPIO 硬件硬触发连接
* 使用信号发生器（如单片机/主相机 Strobe 输出）产生 **150 Hz 的方波脉冲**；
* 将脉冲并联接入两台迈德威视相机的 GPIO **Line In (Line 0)** 接口；
* **必须确保信号源地线（GND）与两台相机的外部地线共地**，实现微秒级曝光时刻物理对齐。

### 5. 镜头对焦与光圈锁死
* 在排球场中场（距离相机约 8~12m）放置排球；
* 手动调焦至球体纹理边缘最清晰，适当收小光圈以获得足够的大景深（确保 5m~25m 均不虚化）；
* **拧紧镜头上的对焦与光圈物理锁紧螺丝**，防止机器人激烈移动或击球震动导致散焦。

---

## 三、 标定与模型配置

### 1. 长基线双目立体标定
由于基线跨度达 1.5m~2.0m，常规 A4 棋盘格在远距离像素极小，建议定制 **$1.0\text{m} \times 0.8\text{m}$ 的 Charuco 标定板**。
1. 采集 25~30 组双目同步图像（涵盖近距 3m 至远距 15m，各种倾角）；
2. 运行双目标定程序，解算出内参矩阵 $K_1, K_2$、畸变系数 $D_1, D_2$ 以及相对外参旋转平移 $[R \mid T]$；
3. 将解算出的矩阵覆盖写入：
   ```text
   volleyball_vision/config/stereo_calibration.yaml
   ```

### 2. YOLO 排球检测模型配置
1. 准备训练好的排球轻量级 ONNX 模型（推荐 YOLOv8n / YOLOv11n，单类别检测，输入 640x640）；
2. 放置到项目目录，并在 `volleyball_vision/config/vision_params.yaml` 中配置路径：
   ```yaml
   /volleyball_vision_node:
     ros__parameters:
       calib_file: "/home/aerialstewart/Volleyball_Tracker/src/volleyball_vision/config/stereo_calibration.yaml"
       model_path: "/home/aerialstewart/Volleyball_Tracker/src/volleyball_vision/models/volleyball_yolov8n.onnx"
       conf_thresh: 0.35
       exposure_us: 1500.0
       use_hw_trigger: true
   ```

---

## 四、 编译与环境构建

在工作空间根目录下，联合下游已有控制器环境进行编译：

```bash
# 1. 载入下游控制器包（提供 ChassisControl 和 StewartControl 消息）
source /home/aerialstewart/26_RC_Volleyball_2/install/setup.bash

# 2. 编译本项目及相关功能包
cd /home/aerialstewart/Volleyball_Tracker
colcon build

# 3. 激活当前工作空间
source install/setup.bash
```

---

## 五、 分级启动与实机调试流程

为确保安全，强烈建议按以下阶段循序渐进开展测试：

### 阶段 1：视觉双目采集与三角测距校验
```bash
ros2 launch volleyball_vision vision.launch.py
```
* **检查项 1**：查看输出帧率是否稳定在 120~150 Hz：
  ```bash
  ros2 topic hz /volleyball/position
  ```
* **检查项 2**：在 5m、10m、15m 处固定排球，打印坐标：
  ```bash
  ros2 topic echo /volleyball/position
  ```
  核对 $Z$ 坐标值与激光测距仪测量的真实物理距离，误差应在 $\pm 2\sim 5\text{ cm}$ 以内。

### 阶段 2：Aero-EKF 轨迹滤波与落点弹道预测
保持视觉节点运行，新开终端启动追踪节点：
```bash
ros2 launch volleyball_tracker tracker.launch.py
```
* **检查项 1**：人工向场内抛掷排球，在终端查看高频平滑位姿与完整预测：
  ```bash
  ros2 topic echo /volleyball/trajectory_pred
  ```
* **检查项 2**：打开 RViz2，添加 `MarkerArray` 监听 `/volleyball/trajectory_markers`：
  - 观察三维黄色抛物线是否平滑、无锯齿跳变；
  - 观察击球平面上的预测落点球是否在排球飞行的初段（起跳点刚过）就已经稳定收敛。

### 阶段 3：Planner 拦截闭环与击打时序验证
保持前两项运行，启动规划中枢：
```bash
ros2 launch volleyball_planner planner.launch.py
```
* **底盘速度响应**：抛球时监听 `ros2 topic echo /chassis_command`，观察 `x_speed` 与 `y_speed` 是否根据落点位置与剩余时间平滑加速与减速对齐；
* **Stewart 姿态响应**：监听 `ros2 topic echo /stewart_command`，观察并联靶面的 `roll` 和 `pitch` 是否朝向来球的反弹法向倾斜；
* **击打触发验证**：监听 `ros2 topic echo /volleyball/strike_trigger`，确认系统在排球落至拦截面前 **0.41s** 时精准触发单次击打脉冲。

---

## 六、 ROS 2 话题接口字典

| 话题名称 | 消息类型 (Message Type) | 产生节点 | 说明 |
| :--- | :--- | :--- | :--- |
| `/volleyball/position` | `geometry_msgs/msg/PointStamped` | `volleyball_vision` | 视觉三角化原始空间点 (camera_optical_frame) |
| `/volleyball/position_filtered` | `geometry_msgs/msg/PointStamped` | `volleyball_tracker` | 经 EKF 滤波平滑的排球实时点 (base_link 系) |
| `/volleyball/trajectory_pred` | `volleyball_tracker/msg/TrajectoryPrediction` | `volleyball_tracker` | 包含实时速度、拦截面落点、剩余飞行时间的完整消息 |
| `/volleyball/landing_prediction` | `geometry_msgs/msg/PointStamped` | `volleyball_tracker` | 拦截面三维落点简易格式 |
| `/volleyball/trajectory_markers` | `visualization_msgs/msg/MarkerArray` | `volleyball_tracker` | RViz2 3D 抛物线与落点标记 |
| `/chassis_command` | `chassis_controllers/msg/ChassisControl` | `volleyball_planner` | 驱动全向/舵轮底盘执行拦截对位 |
| `/stewart_command` | `volleyball_controller/msg/StewartControl` | `volleyball_planner` | 驱动 6-DOF Stewart 并联机构姿态对齐 |
| `/volleyball/strike_trigger` | `std_msgs/msg/Bool` | `volleyball_planner` | 机械击打动作触发信号 (匹配 0.41s 机构延时) |

---

## 七、 常见问题排查 (Troubleshooting)

1. **相机报错 `LIBUSB_ERROR_NO_MEM` 或直接闪退**：
   - 检查 `cat /sys/module/usbcore/parameters/usbfs_memory_mb` 是否为 `1024`。若仍为 `16`，必须按第二节重新配置。
2. **两台相机开机后视差为负数或测距混乱**：
   - 这是因为 USB 重新插拔后左右相机设备号（0 和 1）互换所致。请在调优后使用相机唯一序列号（Serial Number）绑定左右目。
3. **轨迹预测曲线发散或重力方向错误**：
   - 检查 `tracker_params.yaml` 中的外参配置（`cam_mount_z`, `cam_pitch_deg`）或确认 TF 树是否正确广播了 `camera_optical_frame` 至 `base_link` 的旋转变换。
4. **击打动作打空（击打偏早或偏晚）**：
   - 用高速摄像机测量您当前击打机构从触发到最高点的真实物理响应时间，在 `planner_params.yaml` 中微调 `strike_delay_time`（默认 0.41s）。
