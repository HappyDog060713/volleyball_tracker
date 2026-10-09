# volleyball_tracker

基于空气动力学扩展卡尔曼滤波 (Aero-EKF) 的排球 3D 轨迹跟踪与落点预测 ROS 2 功能包。

## 功能特性
- **非线性空气阻力弹道模型**：融入二次方空气阻力 $\vec{a}_{drag} = -\frac{\rho C_d A}{2m} \|\vec{v}\| \vec{v}$，使用 4 阶龙格库塔 (RK4) 进行数值积分。
- **自适应滤波与异常值剔除**：基于马氏距离（Mahalanobis Gate）过滤由于场地反光、球员干扰造成的野值误检。
- **落点与落界预测**：向前数值外推计算在设定拦截面 $Z_{strike}$ 或地面 $Z=0$ 的落点坐标、到达时间倒计时，并自动校验排球是否落在场地界内（In/Out）。
- **RViz2 3D 轨迹可视化**：发布 `visualization_msgs/msg/MarkerArray` 呈现彩色三维抛物线与落点球。

## 通信接口
- **订阅话题**:
  - `/volleyball/position` (`geometry_msgs/msg/PointStamped`): 视觉三角测距输出的原始三维点。
- **发布话题**:
  - `/volleyball/position_filtered` (`geometry_msgs/msg/PointStamped`): 滤波后高频平滑排球三维坐标（天然对接 `volleyball_hub`）。
  - `/volleyball/landing_prediction` (`geometry_msgs/msg/PointStamped`): 预测拦截点/落点。
  - `/volleyball/trajectory_markers` (`visualization_msgs/msg/MarkerArray`): 弹道轨迹可视化标记。

## 编译与启动
```bash
colcon build --packages-select volleyball_tracker
source install/setup.bash
ros2 launch volleyball_tracker tracker.launch.py
```
