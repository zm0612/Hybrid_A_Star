# launch/run_hybrid_a_star.launch 参数说明

本文档说明 `launch/run_hybrid_a_star.launch` 中各个参数的作用与调参方法，供快速定位问题使用。

## 1. 文件整体结构

launch 文件主要做了四件事：

| 内容 | 说明 |
| --- | --- |
| `robot_description` | 加载 `model/car_model/default.urdf` 车体模型 |
| `joint_state_publisher` / `robot_state_publisher` | 根据 URDF 发布 TF，用于 RViz 显示车模 |
| `rviz` | 打开 `rviz/display_hybrid_astar.rviz` 可视化配置 |
| `static_transform_publisher` | 发布 `world → map` 的静态 TF（平移旋转均为 0） |
| `map_server` | 加载 `maps/map.yaml` 栅格地图，发布到 `/map` 话题 |
| `run_hybrid_astar` 节点 | 规划器本体，所有 `planner/` 参数都配置在这个节点下 |

**使用方式**：launch 后在 RViz 中用 **2D Pose Estimate**（发布 `/initialpose`，起点）和 **2D Nav Goal**（发布 `/move_base_simple/goal`，终点）给出起终点，规划器会自动搜索并发布路径。

**关于 `fixed_frame`**：`<arg name="fixed_frame" default="world"/>` 在本 launch 文件中声明后并未被引用，代码里坐标系是硬编码的 `world`（见 `hybrid_a_star_flow.cpp`），因此修改它不会产生实际效果。

---

## 2. 前端 Hybrid A* 搜索参数

这些参数决定搜索图的离散化程度和路径代价，影响**搜索速度、路径粗细与换挡/转向行为**。

### 2.1 车辆运动学

| 参数 | 默认值 | 作用 | 调整方法 |
| --- | --- | --- | --- |
| `planner/steering_angle` | 30.0 | 前轮最大转角（**度**，内部会转成弧度）。决定最小转弯半径 `R = wheel_base / tan(steering_angle)`；同时传给后端优化器作为曲率上限（`max_curvature = tan(转角)/wheel_base`） | 必须与真实车辆一致。改大 → 允许更急的弯，狭窄场景更容易搜到解，但转弯半径过小实际车辆无法执行；改小 → 路径更"拐得开"、更保守，但狭窄空间可能搜不到解 |
| `planner/steering_angle_discrete_num` | 3 | 转角档位离散数。每个节点向左右各生成 `N` 档转角（加上 0 档，共 `2N+1` 条前向弧 + `2N+1` 条后向弧） | 增大 → 转向选择更细，路径更平滑、成功率高，但邻域变大、搜索显著变慢；减小 → 搜索快，但转角粒度粗，可能绕路或失败。一般 1~5 之间 |
| `planner/wheel_base` | 2.0 | 前后轴距（米）。用于运动学模型 `Δθ = Δs·tan(φ)/wheel_base`，即转角一定时轴距越长转弯越缓 | 与真实车辆一致即可。它和 `steering_angle` 共同决定后端优化的曲率上限 |

### 2.2 运动基元（图离散化）

| 参数 | 默认值 | 作用 | 调整方法 |
| --- | --- | --- | --- |
| `planner/segment_length` | 1.6 | 每条运动基元（邻域弧）的长度（米）。值越大图越稀疏 | 增大 → 节点数少、搜索快，但路径粗糙（折线感强）；减小 → 路径细但节点爆炸、搜索变慢。**注意**：必须能被 `segment_length_discrete_num` 整除（代码中有断言 `segment_length = num × 步长`） |
| `planner/segment_length_discrete_num` | 8 | 每条基元细分的步数，步长 `move_step_size = segment_length / num`（默认 0.2 m）。每个细分步都会做一次碰撞检测 | 增大 → 弧线内部碰撞检测更密、更安全，但每步都检测所以更慢；减小 → 更快，但长弧可能"穿过"障碍漏检。同时**步长也决定了最终输出轨迹的采样间距**（`resample_spacing_ = move_step_size`），不要设置得过大 |

### 2.3 代价惩罚项

`g` 代价 = `segment_length × steering_penalty(转向时) × reversing_penalty(倒车时) × steering_change_penalty(换挡角时)`（见 `hybrid_a_star.cpp` 的 `ComputeG`）。

| 参数 | 默认值 | 作用 | 调整方法 |
| --- | --- | --- | --- |
| `planner/steering_penalty` | 1.5 | 任何非零转向档位的惩罚系数 | 增大 → 更偏好走直线，路径少"蛇形"，但可能为了保持直线而绕远；减小 → 转向更自由，路径更贴障碍 |
| `planner/reversing_penalty` | 3.0 | 倒车动作的惩罚系数 | 增大 → 减少不必要的倒车/换挡，路径更"利落"，但在窄巷掉头类场景可能搜不到解或绕远；减小 → 允许更多倒车，适合泊车、死胡同场景 |
| `planner/steering_change_penalty` | 2.0 | 相邻节点转向档位发生变化的惩罚系数 | 增大 → 转向变化更少、方向盘动作更平滑，但灵活性下降；减小 → 转角切换频繁，路径可能抖 |

### 2.4 Reeds-Shepp 解析扩展

| 参数 | 默认值 | 作用 | 调整方法 |
| --- | --- | --- | --- |
| `planner/shot_distance` | 5.0 | 两个用途（米）：① 当前节点到终点距离 ≤ 该值时尝试 RS 曲线直达终点，成功即结束搜索；② 启发式函数在距离 < 3×该值时改用 RS 距离（更准的启发式） | 增大 → 更早用 RS 曲线收尾，终点附近搜索明显加速，但每次尝试本身有开销、失败时是浪费，且过早放弃搜索可能得不到贴障碍的路径；减小 → 纯搜索部分更长，路径更"可控"但更慢。一般 3~10 之间 |

---

## 3. 后处理 / 后端优化参数

`planner/enable_post_process`（默认 `true`）是总开关。后处理流程（`hybrid_a_star_flow.cpp` 的 `PostProcess`）：

1. 推断每个点的前进/倒车方向（齿轮）；
2. **冗余换挡剔除**：删除"磨来磨去"的无效前后摆动；
3. **Ceres 平滑**：`opt_num_passes` 轮「优化 → 等距重采样」，代价 = 平滑 + 参考 + 障碍 + 曲率 + 长度。

设成 `false` 则直接输出前端原始路径。

### 3.1 冗余换挡剔除（`path_processor`）

| 参数 | 默认值 | 作用 | 调整方法 |
| --- | --- | --- | --- |
| `planner/opt_min_segment_length` | 3.0 | 长度（米）小于该值的换挡段视为可折叠的冗余段 | 增大 → 删掉更多短摆动，换挡更少更干净，但可能把真正需要的微调动作删掉；减小 → 保留更多原样换挡。若看到路径里反复"前-后-前"小幅摆动，调大它 |
| `planner/opt_heading_tol` | 0.6 | 一个摆动段"净航向变化"（弧度）小于该值才判定为冗余 | 增大 → 判定更宽松，删除更激进；减小 → 更保守，只有几乎没改变朝向的摆动才会被删。注意剔除时会做碰撞检测（用车身覆盖圆对 ESDF 查询），不安全的缝合不会发生 |

### 3.2 Ceres 平滑器权重（`trajectory_optimizer`）

| 参数 | 默认值 | 作用 | 调整方法 |
| --- | --- | --- | --- |
| `planner/opt_w_smooth` | 2.0 | 平滑项权重（相邻点二阶差分，越小越直） | 增大 → 路径更光滑、曲率更小；过大 → 会把贴墙的细节也拉直，可能顶到障碍（由障碍项拦着）或把掉头点附近抹平 |
| `planner/opt_w_reference` | 0.05 | 参考项权重（偏离前端原始路径的惩罚） | 该项保证问题良定、稳定。增大 → 优化结果更贴近前端路径、更"守规矩"；减小 → 优化自由度更大。一般保持小值即可 |
| `planner/opt_w_obstacle` | 1.0 | 障碍项权重（基于 ESDF 的距离惩罚） | 增大 → 路径更远离障碍、更安全，但在窄通道里会把路径往中间挤；减小 → 允许贴墙。若优化后轨迹蹭到障碍，优先调大它或调小 `opt_obstacle_eps` |
| `planner/opt_w_curvature` | 2.0 | 曲率软上限惩罚，上限由 `steering_angle` 和 `wheel_base` 自动算出 | 该项保证优化后的路径仍满足车辆运动学。增大 → 更强调曲率不超限；若车辆实际无法跟随优化结果（转不过弯），增大它 |
| `planner/opt_w_length` | 0.3 | 总弧长惩罚（抑制长波长蛇形绕行） | 平滑项对低频的大 S 形绕行几乎无效，靠这项把路径"拉直"。**必须保持远小于 `opt_w_smooth`**（其残差是 ~ds 量级而非 ds²，太大容易把路径拉变形）。看到路径整体绕远/大波浪，可小幅增大 |
| `planner/opt_num_passes` | 3 | 「优化 + 重采样」的轮数 | 每轮把路径再拉直一点并重新离散化。增大 → 低频晃动压得更平，但耗时线性增加；减小 → 快但路径可能还残留大波浪。1~5 较合理 |

### 3.3 间隙 / 运动学 / 求解器

| 参数 | 默认值 | 作用 | 调整方法 |
| --- | --- | --- | --- |
| `planner/opt_safety_margin` | 0.1 | 在车身覆盖圆之外额外要求的间隙（米） | 增大 → 离障碍更远、更安全；减小 → 允许贴得更近。想进窄车位/窄通道时减小它 |
| `planner/opt_num_circles` | 5 | 用 N 个圆近似车身轮廓（沿车长均匀分布） | 圆越多半径越小，间隙要求越接近真实半车宽，障碍项表现为"碰撞硬约束"而不是把路径过度推开（过度推开在换挡点固定的情况下反而会在窄处产生折角）。**该值还决定冗余换挡剔除时碰撞检测用的圆半径**。3~7 均可，一般不动 |
| `planner/opt_obstacle_eps` | 0.1 | 倒障碍函数的"地板带宽"（米） | 越小障碍屏障越硬（平滑项更难把路径推进障碍里），数值上更刚但求解可能更慢/更难收敛；越大越软。若优化结果穿障碍 → 调小；若优化卡住不收敛 → 适当调大 |
| `planner/opt_max_iterations` | 150 | 每次 Ceres 求解的最大迭代次数 | 增大 → 收敛更充分但更慢；减小 → 快但可能未收敛。轨迹出现局部"没优化完"的毛刺时增大它 |

### 3.4 换挡点附近的平滑度渐弱（cusp taper）

强平滑会把换挡点（cusp）附近自然的切入几何抹平，因此离换挡点越近，平滑权重越小。

| 参数 | 默认值 | 作用 | 调整方法 |
| --- | --- | --- | --- |
| `planner/opt_cusp_taper_dist` | 5.0 | 渐弱作用的弧长范围（米）。超过该距离恢复满权重 | 增大 → 换挡点前后更长的范围平滑被抑制，掉头几何保持得更完整；减小 → 渐弱只影响换挡点近处一小段 |
| `planner/opt_cusp_taper_min_ratio` | 0.1 | 正好在换挡点处平滑权重的比例（0 = 完全不平滑） | 减小 → 换挡点附近保留更原汁原味的前端几何（参考项和曲率项仍生效，路径不会乱跑）；增大 → 换挡点附近也被拉直一些。若掉头点被抹平导致车辆转不过去，调小它 |

### 3.5 硬锁定区（最强的保护手段）

与渐弱不同，**硬锁定区内的点完全不参与优化**，保持前端几何原样。

| 参数 | 默认值 | 作用 | 调整方法 |
| --- | --- | --- | --- |
| `planner/opt_min_opt_segment_length` | 2.0 | 两个换挡点之间（或换挡点与端点之间）弧长小于该值的段整体冻结不优化（0 = 关闭） | 防止两个相邻换挡点之间的紧凑动作被拉直。若相邻换挡点之间的精细动作被优化"吃掉"，增大它 |
| `planner/opt_cusp_fix_dist` | 2.0 | 每个换挡点前后该弧长（米）内的点全部冻结（0 = 关闭） | 最强旋钮：掉头点前后的轨迹被原样保留，保证换挡前后的切入/切出几何可执行。换挡点被抹平、车转不过去时增大它；路径整体不够平滑时减小它 |
| `planner/opt_endpoint_fix_dist` | 0.0 | 起点/终点附近该弧长（米）内的点全部冻结（0 = 关闭）。注意：**起点/终点的位置和朝向始终被锁定**，不受此参数影响 | 若优化把刚出发/进终点那一小段改得偏离了用户指定的位姿，设一个小值（如 0.5~1.0）保护端点邻域 |

---

## 4. 常见问题速查表

| 现象 | 优先调整 |
| --- | --- |
| 搜索太慢 / 卡死 | 增大 `segment_length`；减小 `steering_angle_discrete_num`；增大 `shot_distance`；减小 `segment_length_discrete_num`（注意碰撞检测变粗） |
| 路径像折线、不光滑 | 减小 `segment_length`；增大 `steering_angle_discrete_num`、`opt_w_smooth`、`opt_num_passes` |
| 路径大波浪、绕远 | 增大 `opt_w_length`（保持远小于 `opt_w_smooth`）；增大 `opt_num_passes` |
| 换挡太多、来回磨 | 增大 `reversing_penalty`；增大 `opt_min_segment_length`、`opt_heading_tol` |
| 优化后路径蹭障碍 / 穿障碍 | 增大 `opt_w_obstacle`；减小 `opt_obstacle_eps`；增大 `opt_max_iterations` |
| 窄通道 / 窄车位进不去 | 减小 `opt_safety_margin`；减小 `steering_angle` 允许更急的弯 |
| 掉头点被优化抹平、车转不过去 | 增大 `opt_cusp_fix_dist`；减小 `opt_cusp_taper_min_ratio`；增大 `opt_w_curvature` |
| 路径偏离前端原始结果太多 | 增大 `opt_w_reference`；增大 `opt_cusp_taper_dist` |
| 后端优化太慢 | 减小 `opt_max_iterations`、`opt_num_passes` |
| 想完全关掉后处理对比效果 | `enable_post_process` 设为 `false` |

---

## 5. 注意事项

1. **车身尺寸是硬编码的**：后端优化器和换挡剔除用的车辆尺寸（长 4.7 m、宽 2.0 m、后轴到车尾 1.3 m）写死在 `hybrid_a_star_flow.h` 的成员变量里，**不是**从 URDF 读取，也没有对应的 launch 参数。RViz 中 `vehicle_path` 车模显示的尺寸（4.0 × 2.0）也是硬编码在 `hybrid_a_star_flow.cpp` 的 `PublishVehiclePath` 调用里。换车时需要同步修改这两处源码。
2. **参数联动**：
   - `segment_length / segment_length_discrete_num` 同时决定了最终输出轨迹的采样间距（默认 0.2 m）；
   - `steering_angle` 和 `wheel_base` 同时决定了后端优化的曲率上限，改前端参数会连带影响优化结果；
   - `opt_num_circles` 同时影响优化器的障碍项和冗余换挡剔除的碰撞检测半径。
3. **数值约束**：`segment_length` 应能被 `segment_length_discrete_num` 整除（1.6 / 8 = 0.2），否则启动时断言失败。
4. **障碍判定**：地图中所有非 0 的栅格（包括"未知"区域）都会被视为障碍（前端搜索与 ESDF 构建一致），如果地图四周有未探索区域，车辆会被限制在已知区域内。
5. 修改参数后需要重新 `roslaunch`，参数在节点启动时一次性读取。
