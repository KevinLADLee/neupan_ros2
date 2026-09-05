# NeuPAN 原始算法与 C++ 运行时的等价性说明

本文以原始仓库 `NeuPAN/neupan` 中的 Python/CVXPY 实现为基准，不以任何第三方
NeuPAN C++ 移植为正确性依据。当前结论适用于本仓库已经声明支持的范围：差速机器人、
凸多边形（当前配置为矩形）外形、常速度动态点云和 line 初始路径。

## 1. DUNE

对第 `t` 个预测时刻，机器人状态为
`s_t = [x_t, y_t, theta_t]`，全局障碍点为 `p_t`。原始实现先计算

```text
R_t = [[cos(theta_t), -sin(theta_t)],
       [sin(theta_t),  cos(theta_t)]]
p0_t = R_t^T (p_t - [x_t, y_t])
mu_t = MLP(p0_t)
lambda_t = -R_t G^T mu_t
distance_t = mu_t^T (G p0_t - h)
```

动态点按原始 PAN 的常速度假设先传播：

```text
p_t = p_observed + t * dt * velocity.
```

C++ 对点和速度使用完全相同的下采样索引，再执行上述每阶段坐标变换。

C++ 的 `PAN::generatePointFlow` 与 `DUNE::forward` 使用相同表达式。MLP 层次、
LayerNorm 的 biased variance、激活次序和 float32 推理也与 PyTorch 网络一致。

C++ 只对距离最小的 `nrmp_max_num` 个点做 partial sort。它与原始全排序在规划结果上
等价，因为：

1. NRMP 只读取排序后的前 `nrmp_max_num` 个点；
2. PAN 停止条件也只读取前 `effect_num <= nrmp_max_num` 个 `mu/lambda`；
3. `min_distance` 在截断前对全点云计算。

因此这是减少排序开销的等价优化，不是模型近似。

## 2. 差速运动学线性化

原始离散模型为

```text
f(s,u) = s + dt [v cos(theta), v sin(theta), omega]^T.
```

在名义点 `(s_bar, u_bar)` 的一阶仿射展开写成

```text
s_(t+1) = A_t s_t + B_t u_t + C_t,
```

其中

```text
A = [[1, 0, -v_bar dt sin(theta_bar)],
     [0, 1,  v_bar dt cos(theta_bar)],
     [0, 0, 1]]

B = [[cos(theta_bar) dt, 0],
     [sin(theta_bar) dt, 0],
     [0, dt]]

C = [ theta_bar v_bar dt sin(theta_bar),
     -theta_bar v_bar dt cos(theta_bar),
       0 ].
```

代入名义点可得 `A s_bar + B u_bar + C = f(s_bar,u_bar)`，其局部误差为二阶量。
C++ 实现逐项相同，并由解析回归测试验证。

## 3. NRMP 原问题

变量为状态 `s in R^(3 x (T+1))`、控制 `u in R^(2 x T)`，有障碍时另有
`d in R^T`。对每个预测时刻和入选障碍点，原始代码构造

```text
fa_(t,k) = lambda_(t+1,k)^T
fb_(t,k) = lambda_(t+1,k)^T p_(t+1,k) + mu_(t+1,k)^T h
I_(t,k)  = fa_(t,k) s_xy_(t+1) - fb_(t,k) - d_t.
```

少于 `nrmp_max_num` 个点时，原始实现重复距离最近点的系数；C++ 保持相同规则。

原始目标函数为

```text
J = ||q_s .* (s-ref_s)||_F^2
  + ||p_u (u_v-ref_us)||_2^2
  + 0.5 bk ||s-nom_s||_F^2
  - eta sum(d)
  + 0.5 ro_obs sum(max(-I, 0)^2).
```

约束为

```text
s_0 = nom_s_0
s_(t+1) = A_t s_t + B_t u_t + C_t
|u_t| <= max_speed
|u_(t+1)-u_t| <= max_acce * dt
max(0, d_min) <= d_t <= d_max.
```

最后一个下界中的 `max(0,d_min)` 来自原始 CVXPY 同时声明
`Variable(nonneg=True)` 和 `d >= d_min`。此前 C++ 仅使用 `d_min`；现已修正。

## 4. 从 CVXPY 到 OSQP 的等价变换

C++ 引入 `e_(t,k)` 并写成

```text
e_(t,k) >= -I_(t,k),  e_(t,k) >= 0,
cost_e = 0.5 ro_obs e_(t,k)^2.
```

当 `ro_obs > 0` 时，对固定的 `(s,d)`，唯一最优值是
`e = max(-I,0)`，代回后正好得到原始 `cp.sum_squares(cp.neg(I))`。当
`ro_obs = 0` 时，两种形式对 `(s,u,d)` 的最优集合仍相同。因此该 epigraph 变换是精确
重写，不是软约束近似。

OSQP 使用 `0.5 z^T P z + q^T z`。按 Eigen 的列主序展开 `s,u,d,e` 后：

```text
P_s(t,r) = 2 q_s(r)^2 + bk
q_s(t,r) = -2 q_s(r)^2 ref_s(r,t) - bk nom_s(r,t)
P_uv(t)  = 2 p_u^2
q_uv(t)  = -2 p_u^2 ref_us(t)
q_d(t)   = -eta
P_e(t,k) = ro_obs.
```

与变量无关的常数项不进入 OSQP，不影响最优解。动力学在约束矩阵中写为
`A_t s_t + B_t u_t - s_(t+1) = -C_t`，其余 box/avoidance 约束也是原不等式的直接
改写。

结论：当前 OSQP 建模在上述支持范围内是原始凸问题的等价 QP。ECOS 与 OSQP 的停止
准则、缩放和落在最优集合中的具体点可以不同，所以应比较可行性、目标值与控制输出，
而不能要求所有浮点位完全一致。

## 5. PAN 外循环和无障碍降维

原始 PAN 在相邻迭代间使用

```text
no DUNE: ||s-s_prev||^2 + ||u-u_prev||^2
with DUNE: (||mu-mu_prev|| / effect_num)^2
         + (||lambda-lambda_prev|| / effect_num)^2
```

并让前一迭代值跨控制帧保留。C++ 的范数展开和状态生命周期与此一致。

当 `nrmp_max_num == 0` 时，原始 NRMP 完全没有 `d`。此前 C++ 仍创建 `T` 个无用
`d`，现已移除。当 `dune_max_num == 0`、但 `nrmp_max_num > 0` 时，原始 PAN 会跳过
DUNE，却在 NRMP 中留下 `fa=fb=0` 的 `d/e` 子问题。该子问题与 `(s,u)` 完全可分离，
所以 C++ 也将它消去；导航状态和控制最优解不变，仅不再返回无意义的 `d`。无障碍部署
也因此不再加载 DUNE 模型文件。

## 6. 已修正的非等价点

- 外部路径：原始 `set_initial_path` 保留输入采样和 gear，并把平均相邻距离设为
  `interval`；此前 C++ 会重新插值且把 gear 全改成 `+1`，现已按原始语义修正。
- 距离非负性：C++ 的实际下界现为 `max(0,d_min)`。
- 无障碍变量：`nrmp_max_num == 0` 时不再创建原始问题中不存在的 `d/e`。
- 无 DUNE 模式：不再要求 checkpoint，并消去与导航变量可分离的障碍子问题。
- OSQP 多帧更新：固定稀疏结构仍保留，但每帧完整更新 `A` 的数值。原因是
  OsqpEigen 0.11.2 的高层更新接口以初始 Data 快照做比较，系数回到初值时可能漏更新。
  当前实现针对固定的 OSQP 1.0.0 API，并有多帧回归覆盖。

## 7. 尚未宣称等价的范围

- 动态障碍已支持原始实现的逐点常速度模型；加速度、转弯意图和概率轨迹不属于当前
  NeuPAN 模型。
- Ackermann、omni 运动学和非 line 曲线尚未移植。
- loop 路径模式尚未移植。
- C++ core 的外部路径 API 已保留 gear；但标准 `nav_msgs/Path` 没有 gear 字段，当前
  ROS 2 适配层仍将该输入解释为全程前进。反向路径需要单独的带 gear 消息接口。
- 求解失败时 C++ 保留上一次可接受计划；原始 Python 通常让求解异常向上传播。
- C++ 对成功结果的首控制量再做一次速度边界钳制。正常可行解上它是恒等操作。
- 端到端回放目前会逐帧同步 Python 的隐藏名义控制序列，用于隔离求解器误差；它不是
  “两套闭环长期轨迹完全相同”的证明。

因此，当前可以确认的是“已支持常速度动态点云差速子集中的数学问题等价”，而不是“完整 NeuPAN
所有功能等价”。后续扩展每一种运动学或更丰富的动态障碍模型时，都应先补同级别的方程与数值
回归，再开放 ROS 2 接口。
