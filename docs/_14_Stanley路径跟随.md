下面记录了基于仓库中 `src/pure_persuit` 的结构，新建 **Stanley** 路径跟随包时用到的核心原理与公式，供对照代码理解。

## 控制思路与关键公式

- 选择**参考航向**：在车辆最近的路径点 $P\_r(x\_r, y\_r)$ 与其后一点评估切线方向，得到参考航向 $\psi\_p = \operatorname{atan2}(y\_{r+1}-y\_r,\, x\_{r+1}-x\_r)$。
- **航向误差**：车辆航向为 $\psi$，采用标准角度归一化 $\theta\_e = \operatorname{wrap}(\psi\_p - \psi)$（将差值限制在 $[-\pi, \pi]$）。
- **横向误差**：使用路径切线的法向量计算有符号距离
  $$e\_{ct} = -\sin(\psi\_p)\,(x - x\_r) + \cos(\psi\_p)\,(y - y\_r)$$
  其中 $(x, y)$ 为车辆当前位置；符号表示车辆位于路径左/右侧。
- **Stanley 控制律**：在车辆当前速度 $v$ 下，前轮转角命令
  $$\delta = \theta\_e + \arctan\!\left(\frac{k\_e\, e\_{ct}}{v + k\_s}\right)$$
  其中 $k\_e$ 为横向误差增益，$k\_s$ 为软化系数（避免 $v \to 0$ 时的高增益发散）。
- **停止判据**：若车辆到终点的距离 $d\_{goal} < \text{stop\_distance}$，则线速度和角速度均置零。

## 代码结构与节点接口

- 包名：`stanley_controller`，启动文件：`launch/stanley_controller.launch`。
- 节点订阅：
  - `/smart/rear_pose` (`geometry_msgs/PoseStamped`)
  - `/smart/velocity` (`geometry_msgs/TwistStamped`)
  - `/final_waypoints` (`styx_msgs/Lane`)
- 节点发布：
  - `/smart/cmd_vel` (`geometry_msgs/Twist`)：`linear.x` 取路径点的期望速度，`angular.z` 按上式求得。

上述公式与接口与 `scripts/stanley_controller.py` 中的实现保持一致，可直接对照查看参数和默认增益。
