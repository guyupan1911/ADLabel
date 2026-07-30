# 基于 Textured Surfel Map 的道路 Top-down 彩色地图生成方案

## 1. 目标

### 1.1 输入

* 每帧去运动畸变后的 LiDAR 点云
* LIO 输出的连续车辆轨迹
* 多相机图像
* Camera 内参和畸变参数
* LiDAR–Camera 外参
* LiDAR 和 Camera 时间戳

### 1.2 输出

生成指定地图区域、固定分辨率的 Top-down RGB 道路图，并输出与 RGB 图严格对齐的辅助图层：

```text
rgb_bev.png
height_bev.exr
normal_bev.exr
intensity_bev.png
confidence_bev.exr
source_frame_bev.png
```

### 1.3 设计目标

1. 道路几何位置准确
2. 车道线、停止线、箭头等纹理尽量清晰
3. 最终图像分辨率不直接受 LiDAR 点密度限制
4. 支持坡道、道路横坡和局部起伏
5. 减少车辆、行人和遮挡物对道路纹理的污染
6. 支持按 Tile 分块生成和更新

---

## 2. 核心设计

整体流程为：

```text
LIO 轨迹 + 单帧 LiDAR
          │
          ▼
逐点运动补偿并拼接
          │
          ▼
静态全局点云
          │
          ▼
地面/道路候选点提取
          │
          ▼
自适应 Octree 构建
          │
          ▼
每个叶节点拟合局部平面 Surfel
          │
          ▼
计算 Plane ∩ Voxel 的有效多边形
          │
          ▼
在 Surfel 平面建立固定分辨率纹理栅格
          │
          ▼
纹理 Texel 投影到候选相机
          │
          ▼
可见性、视角和图像质量检查
          │
          ▼
选择最佳观测或融合 Top-K 观测
          │
          ▼
得到 Textured Surfel Map
          │
          ▼
正交 BEV Rasterization
          │
          ▼
Top-down RGB 道路地图
```

本方案将两种分辨率解耦：

* **几何分辨率**：由局部几何复杂度决定，例如 5～40 cm
* **纹理分辨率**：固定为 2～3 cm/texel，用于保存车道线等精细纹理

因此，大面积平坦路面可以由一个较大的 Surfel 表达，但其内部仍然可以保存高分辨率颜色纹理。

---

## 3. 坐标系定义

建议统一使用以下坐标系：

* (W)：LIO 世界坐标系
* (L)：LiDAR 坐标系
* (R)：车辆 Rig 坐标系
* (C_k)：第 (k) 个 Camera 坐标系
* (B)：BEV 地图坐标系，一般与世界坐标系的水平 XY 平面对齐

LIO 输出的 LiDAR 世界位姿为：

[
T_{W\leftarrow L}(t)
]

相机在曝光时刻 (t_c) 的世界位姿为：

[
T_{W\leftarrow C_k}(t_c)
========================

T_{W\leftarrow L}(t_c)
T_{L\leftarrow C_k}
]

其中：

* (T_{L\leftarrow C_k}) 是 Camera 到 LiDAR 的外参
* (T_{W\leftarrow L}(t_c)) 必须通过 LIO 轨迹插值得到
* 不应直接使用距离相机时间戳最近的 LiDAR 帧位姿

将世界点投影到相机时，需要使用逆变换：

[
T_{C_k\leftarrow W}(t_c)
========================

T_{W\leftarrow C_k}^{-1}(t_c)
]

---

## 4. 第一阶段：生成最终静态全局点云

### 4.1 LiDAR 点运动补偿

旋转式 LiDAR 一帧扫描期间车辆持续运动，因此不同点具有不同采集时刻。

对于点 (\mathbf p_i^L)，应使用该点自己的采集时间 (t_i)：

[
\mathbf p_i^W
=============

T_{W\leftarrow L}(t_i)
\mathbf p_i^L
]

不能让整帧点云共享同一个位姿，否则可能导致：

* 地面变厚
* 路沿出现双边
* 墙面变宽
* 后续图像投影产生系统性错位

### 4.2 使用最终优化轨迹

建议在完成以下优化后，再重新拼接最终点云：

* 回环检测和 Pose Graph 优化
* GNSS 融合
* 全局轨迹优化
* 局部或全局 Bundle Adjustment

不要直接给在线 LIO 过程中产生的临时地图上色。

轨迹发生调整后，应重新执行：

```text
原始单帧点云
    ↓
基于最终轨迹重新运动补偿
    ↓
重新拼接全局点云
    ↓
重新建立 Surfel Map
    ↓
重新融合 Camera 颜色
```

### 4.3 动态点过滤

尽量移除：

* 车辆
* 行人
* 骑行者
* 临时障碍物
* 明显孤立点
* 短时间出现的移动物体

可以使用：

* 已有 3D 检测框
* 多帧占据一致性
* 点云高度和连通性规则
* 同一地图位置的时间一致性判断

如果最终只需要道路图，可以只保留地面和道路附近高度范围内的点。

---

## 5. 第二阶段：构建自适应 Surfel Octree

### 5.1 Octree 分辨率

第一版建议使用以下层级：

```text
40 cm
20 cm
10 cm
5 cm
```

也可以使用：

```text
根节点尺度：80 cm

允许生成叶节点的尺度：
40 cm
20 cm
10 cm
5 cm
```

构建策略为从粗到细：

```text
尝试用大 Voxel 中的一个平面表达局部点云
                │
        ┌───────┴────────┐
        │                │
   拟合足够准确       拟合不准确
        │                │
        ▼                ▼
  生成叶节点 Surfel    继续拆分八个子节点
```

### 5.2 节点统计

设节点内点集为：

[
\mathcal P
==========

{\mathbf p_1,\mathbf p_2,\ldots,\mathbf p_N}
]

点集均值为：

[
\boldsymbol\mu
==============

\frac{1}{N}
\sum_{i=1}^{N}\mathbf p_i
]

协方差矩阵为：

[
\Sigma
======

\frac{1}{N}
\sum_{i=1}^{N}
(\mathbf p_i-\boldsymbol\mu)
(\mathbf p_i-\boldsymbol\mu)^\top
]

对协方差矩阵做特征分解：

[
\Sigma
======

V
\operatorname{diag}
(\lambda_0,\lambda_1,\lambda_2)
V^\top
]

约定：

[
\lambda_0\leq\lambda_1\leq\lambda_2
]

最小特征值对应的特征向量作为局部平面法向：

[
\mathbf n=\mathbf v_0
]

对于道路 Surfel，统一令法向朝上：

[
n_z>0
]

如果 (n_z<0)，则执行：

[
\mathbf n\leftarrow-\mathbf n
]

### 5.3 平面模型

局部平面可以表示为：

[
\mathbf n^\top
(\mathbf x-\boldsymbol\mu)=0
]

每个点到平面的有符号距离为：

[
d_i
===

\mathbf n^\top
(\mathbf p_i-\boldsymbol\mu)
]

平面拟合 RMS 为：

[
E_{\mathrm{rms}}
================

\sqrt{
\frac{1}{N}
\sum_{i=1}^{N}d_i^2
}
]

最大残差为：

[
E_{\max}
========

\max_i|d_i|
]

平面性指标可以定义为：

[
E_{\mathrm{plane}}
==================

\frac{\lambda_0}
{\lambda_0+\lambda_1+\lambda_2}
]

### 5.4 停止细分条件

节点要成为叶节点 Surfel，建议同时满足：

```text
point_count ≥ N_min
plane_rms ≤ threshold_rms
max_residual ≤ threshold_max
planarity ≤ threshold_planarity
support_coverage ≥ threshold_coverage
```

参考初始参数：

| Voxel 尺度 | 最少点数 |     RMS 阈值 | 最大残差 |
| -------- | ---: | ---------: | ---: |
| 40 cm    |   30 | 1.5～2.0 cm | 4 cm |
| 20 cm    |   15 | 1.0～1.5 cm | 3 cm |
| 10 cm    |    8 | 0.8～1.0 cm | 2 cm |
| 5 cm     |  3～5 |  最后一层可强制输出 |    — |

这些阈值需要根据实际 LIO 拼接后地面厚度调整。

### 5.5 防止错误的大平面

仅依赖 RMS 可能将路面和路沿错误拟合成一个大平面。

建议增加以下检查：

* 点到平面距离分布是否多峰
* 局部法向是否存在多个明显方向
* 支持点在局部平面内是否覆盖充分
* Voxel 内是否存在明显高度跳变
* 与相邻节点的高度和法向是否连续

如果支持点只分布在 Voxel 的一个小角落，不应让 Surfel 覆盖整个 Voxel。

---

## 6. Surfel 几何表示

每个叶节点可以保存：

```cpp
struct SurfelGeometry {
    Eigen::Vector3d center;
    Eigen::Vector3d normal;

    Eigen::Vector3d tangent_u;
    Eigen::Vector3d tangent_v;

    Eigen::Matrix3d covariance;

    Eigen::Vector3d voxel_min;
    Eigen::Vector3d voxel_max;

    std::vector<Eigen::Vector3d> boundary_polygon;

    float plane_rms;
    float max_residual;
    float confidence;

    uint32_t point_count;
};
```

### 6.1 建立 Surfel 局部坐标系

对于道路表面，建议让纹理方向尽量稳定地与世界 XY 方向对齐。

首先将世界 X 轴投影到 Surfel 平面：

[
\tilde{\mathbf e}_u
===================

## \mathbf e_x

(\mathbf e_x^\top\mathbf n)\mathbf n
]

然后归一化：

[
\mathbf e_u
===========

\frac{\tilde{\mathbf e}_u}
{|\tilde{\mathbf e}_u|}
]

另一个切向方向为：

[
\mathbf e_v
===========

\mathbf n\times\mathbf e_u
]

如果法向接近世界 X 轴，使得投影长度过小，则改用世界 Y 轴构造 (\mathbf e_u)。

不建议直接使用 PCA 的另外两个特征向量作为纹理坐标轴，因为：

* PCA 切向量可能因噪声发生翻转
* 相邻 Surfel 的纹理方向可能不连续
* 不利于纹理拼接和 BEV 输出

### 6.2 Surfel 有效边界

采用 TeSO 的 Cube-bounded Surfel 思路：

[
\text{Surfel Boundary}
======================

\text{Plane}
\cap
\text{Voxel Cube}
]

计算过程：

1. 枚举 Voxel 的 12 条边
2. 计算局部平面与每条边的交点
3. 仅保留位于线段内部的交点
4. 去除重复交点
5. 将交点投影到 Surfel 局部 ((u,v)) 坐标系
6. 按极角排序，形成凸多边形

相较于固定圆盘，这种表示具有以下优点：

* Surfel 的空间范围与 Octree 节点一致
* 相邻节点边界天然对齐
* 更容易做正交 BEV 栅格化
* 减少不同尺度 Surfel 之间的无意义重叠

对于支持点覆盖不足的边界节点，可以将支持点投影到局部二维平面，并使用二维 Convex Hull 进一步限制有效区域。

---

## 7. 第三阶段：建立 Surfel 固定分辨率纹理栅格

### 7.1 纹理分辨率

建议：

```text
第一版：3 cm/texel
高质量版本：2 cm/texel
```

不建议一开始直接使用 1 cm/texel，因为整体精度通常受以下因素限制：

* LiDAR–Camera 外参
* 时间同步误差
* LIO 轨迹误差
* Rolling shutter
* 相机运动模糊
* 路面到相机的观察角度

### 7.2 纹理覆盖范围

设 Surfel 有效多边形顶点为 (\mathbf q_k)。

将每个顶点投影到局部二维坐标：

[
u_k
===

\mathbf e_u^\top
(\mathbf q_k-\boldsymbol\mu)
]

[
v_k
===

\mathbf e_v^\top
(\mathbf q_k-\boldsymbol\mu)
]

计算：

[
u_{\min},u_{\max},
v_{\min},v_{\max}
]

纹理宽度为：

[
W_s
===

\left\lceil
\frac{u_{\max}-u_{\min}}
{r_t}
\right\rceil
]

纹理高度为：

[
H_s
===

\left\lceil
\frac{v_{\max}-v_{\min}}
{r_t}
\right\rceil
]

其中 (r_t) 为纹理物理分辨率。

### 7.3 Texel 对应的三维位置

纹理中第 ((i,j)) 个 Texel 的局部坐标为：

[
u_i
===

u_{\min}
+
\left(i+\frac{1}{2}\right)r_t
]

[
v_j
===

v_{\min}
+
\left(j+\frac{1}{2}\right)r_t
]

对应世界坐标点为：

[
\mathbf P_{ij}^{W}
==================

\boldsymbol\mu
+
u_i\mathbf e_u
+
v_j\mathbf e_v
]

只保留位于 Surfel 有效多边形内部的 Texel。

### 7.4 数据结构

```cpp
struct TextureTexel {
    Eigen::Vector3f rgb;

    float best_score;
    float confidence;
    float best_gsd;
    float color_variance;

    uint32_t source_frame_id;
    uint8_t source_camera_id;

    bool valid;
};

struct TexturedSurfel {
    SurfelGeometry geometry;

    float texel_resolution;
    float u_min;
    float v_min;

    int texture_width;
    int texture_height;

    std::vector<TextureTexel> texels;
    std::vector<uint8_t> valid_polygon_mask;
};
```

---

## 8. 第四阶段：建立候选相机观测

不能让每个 Surfel 遍历所有 Camera 图像，否则计算量过大。

### 8.1 Camera Frame 数据结构

```cpp
struct CameraFrame {
    uint32_t frame_id;
    uint8_t camera_id;
    uint64_t timestamp_ns;

    SE3 T_world_camera;
    CameraModel model;

    Eigen::Vector3d position_world;
    Frustum frustum;
};
```

### 8.2 相机帧空间索引

可以使用：

* 基于相机位置的 KD-tree
* 基于轨迹距离的有序索引
* 基于地图 Tile 的 Camera Frame 倒排索引
* 基于相机 Frustum 和 Surfel AABB 的相交查询

对每个 Surfel，只查询：

* 距离在合理范围内的相机
* 相机视锥可能覆盖 Surfel 的帧
* 相机朝向能够看到该表面的帧

参考候选距离：

| 相机类型 |   候选距离 |
| ---- | -----: |
| 前视窄角 | 5～40 m |
| 前视广角 | 3～25 m |
| 侧视相机 | 2～15 m |

最终应根据实际 GSD 再筛选，而不是只根据相机距离。

---

## 9. 第五阶段：将 Texel 投影到 Camera

对于一个 Texel 世界点：

[
\mathbf P_W
]

变换到相机坐标系：

[
\mathbf P_C
===========

T_{C\leftarrow W}(t_c)
\mathbf P_W
]

要求：

[
P_{C,z}>0
]

再通过相机投影模型得到像素位置：

[
(u,v)
=====

\pi(\mathbf P_C)
]

投影模型必须与实际相机一致，例如：

* Pinhole
* Fisheye
* Kannala–Brandt
* Unified Camera Model

建议提前将原始图像去畸变到统一针孔模型，后续使用标准针孔投影，工程实现更简单。

颜色从原始或去畸变图像中进行双线性采样：

[
C_k
===

I_k(u,v)
]

---

## 10. 第六阶段：可见性判断

仅仅投影在图像范围内，并不代表 Texel 在该相机中真实可见。

### 10.1 静态 Surfel Map Z-buffer

对每张相机图像，将附近 Surfel Patch 渲染到相机平面，输出：

```text
depth_buffer
surfel_id_buffer
normal_buffer
```

对于某个 Texel：

* 预测相机深度为 (z_t)
* Depth Buffer 中的最近深度为 (z_b)

只有满足：

[
|z_t-z_b|<\tau_z
]

才认为该 Texel 在静态地图层面可见。

深度阈值可以随距离增加：

[
\tau_z(d)
=========

\tau_0+kd
]

例如：

[
\tau_0=0.05\text{ m}
]

### 10.2 当前帧动态遮挡

静态 Surfel Map 中通常不包含当前帧的车辆和行人，因此还需要检查当前时刻的动态遮挡。

可以将相机邻近时刻的单帧 LiDAR 投影到相机，生成：

```text
current_scan_depth_buffer
```

如果：

[
z_{\mathrm{scan}}
<
z_{\mathrm{texel}}-\tau
]

说明相机到道路 Texel 之间存在更近的物体，该观测不能用于采色。

还可以结合：

* 已有的 3D 动态目标框
* 图像动态区域 Mask
* 车辆和行人 Tracking 结果

### 10.3 入射角检查

从 Texel 指向相机的单位向量为：

[
\mathbf v_k
===========

\frac{\mathbf C_k-\mathbf P}
{|\mathbf C_k-\mathbf P|}
]

观察角余弦为：

[
c_k
===

\mathbf n^\top\mathbf v_k
]

要求：

[
c_k>\cos\theta_{\max}
]

初始可以设置：

[
\theta_{\max}=70^\circ
]

观察角过大时，相机像素在路面上的物理覆盖面积很大，纹理会被明显拉伸。

### 10.4 图像有效区域检查

过滤：

* 图像最外侧边缘
* 固定车身遮挡区域
* 镜头污渍区域
* 严重过曝和欠曝区域
* 明显运动模糊的图像
* 强畸变区域

---

## 11. 第七阶段：观测质量评分

对于 Texel 在第 (k) 张相机图像中的观测，定义质量评分：

[
S_k
===

S_{\mathrm{angle}}
S_{\mathrm{gsd}}
S_{\mathrm{center}}
S_{\mathrm{sharpness}}
S_{\mathrm{exposure}}
S_{\mathrm{visibility}}
]

### 11.1 视角评分

[
S_{\mathrm{angle}}
==================

\max
(0,\mathbf n^\top\mathbf v_k)^\gamma
]

建议：

[
\gamma=2\sim4
]

相机观察方向越接近表面法向，权重越高。

### 11.2 GSD 评分

GSD 表示一个相机像素投影到道路表面后覆盖的真实尺寸。

可以通过相邻像素射线与 Surfel 平面求交估计：

[
\mathrm{GSD}_x
==============

|
\mathbf P(u+1,v)
----------------

\mathbf P(u,v)
|
]

[
\mathrm{GSD}_y
==============

|
\mathbf P(u,v+1)
----------------

\mathbf P(u,v)
|
]

定义：

[
\mathrm{GSD}
============

\max
(\mathrm{GSD}_x,\mathrm{GSD}_y)
]

纹理分辨率为 (r_t) 时，优先选择：

[
\mathrm{GSD}\leq r_t
]

评分可以定义为：

[
S_{\mathrm{gsd}}
================

\min
\left(
1,
\frac{r_t}
{\mathrm{GSD}+\epsilon}
\right)
]

相比仅根据相机距离选择图像，GSD 更能准确反映当前图像对路面的实际纹理分辨率。

### 11.3 图像中心评分

[
S_{\mathrm{center}}
===================

\exp
\left(
-\frac{
|(u,v)-(u_0,v_0)|^2
}{
2\sigma_c^2
}
\right)
]

用于降低图像边缘和畸变较大区域的权重。

### 11.4 清晰度评分

可以为整张图像或局部 Patch 计算：

* Laplacian 方差
* 梯度能量
* 高频能量
* 运动模糊指标

清晰度低于阈值的观测直接过滤。

---

## 12. 第八阶段：颜色融合

### 12.1 第一版：Top-1 最佳观测

每个 Texel 只保留评分最高的 Camera 观测：

```cpp
if (score > texel.best_score) {
    texel.rgb = sampled_rgb;
    texel.best_score = score;
    texel.confidence = score;
    texel.best_gsd = gsd;
    texel.source_frame_id = frame_id;
    texel.source_camera_id = camera_id;
    texel.valid = true;
}
```

Top-1 的优点：

* 能保持车道线边缘清晰
* 不会因为轻微位姿误差导致多帧平均模糊
* 每个 Texel 的来源明确
* 便于排查标定、同步和轨迹问题

### 12.2 第二版：Top-K 鲁棒融合

每个 Texel 保存评分最高的 3～5 个观测。

可以使用：

* 加权中值
* Trimmed Mean
* Huber 鲁棒均值
* 对离群颜色进行剔除后再加权平均

不建议将所有可见帧直接平均。

### 12.3 跨相机光度校正

不同 Camera 可能存在：

* 曝光不同
* 白平衡不同
* 色彩响应不同
* 暗角不同

可以为每个 Camera 建立颜色校正模型：

[
C_{\mathrm{canonical}}
======================

A_kC_k+\mathbf b_k
]

第一版可以简化为每个通道独立的增益和偏置：

[
C'_c
====

a_cC_c+b_c
]

参数可以通过不同 Camera 重叠区域中的静态道路颜色估计。

---

## 13. 第九阶段：生成 Top-down RGB Image

最终不应把 Texel 当作离散点直接撒到 XY 栅格，而应该将每个 Textured Surfel 作为带纹理的平面多边形做正交栅格化。

### 13.1 定义 BEV 范围

设地图范围为：

[
[x_{\min},x_{\max}]
\times
[y_{\min},y_{\max}]
]

BEV 分辨率为：

[
r_{\mathrm{bev}}
================

0.02\sim0.03\text{ m/pixel}
]

图像宽度为：

[
W
=

\left\lceil
\frac{x_{\max}-x_{\min}}
{r_{\mathrm{bev}}}
\right\rceil
]

图像高度为：

[
H
=

\left\lceil
\frac{y_{\max}-y_{\min}}
{r_{\mathrm{bev}}}
\right\rceil
]

### 13.2 世界坐标映射到 BEV 像素

[
u_{\mathrm{bev}}
================

\frac{x-x_{\min}}
{r_{\mathrm{bev}}}
]

[
v_{\mathrm{bev}}
================

\frac{y_{\max}-y}
{r_{\mathrm{bev}}}
]

这里对 Y 轴进行翻转，使地图北向或世界 Y 正方向位于图像上方。

### 13.3 Surfel Polygon 栅格化

对每个 Textured Surfel：

1. 获取 Plane–Voxel Intersection Polygon
2. 将凸多边形三角化
3. 每个三角形顶点保存：

   * 世界坐标 XYZ
   * Surfel 局部纹理坐标 UV
4. 使用正交投影栅格化到 BEV
5. 对每个 BEV Pixel 计算重心坐标
6. 插值得到 Surfel Local UV
7. 从 Surfel Texture 中双线性采样 RGB
8. 写入 RGB BEV

伪代码：

```cpp
for (const auto& surfel : road_surfels) {
    const auto triangles =
        Triangulate(surfel.geometry.boundary_polygon);

    for (const auto& triangle : triangles) {
        RasterizeTriangleToBev(
            triangle,
            [&](int pixel_x,
                int pixel_y,
                const Barycentric& barycentric) {
                const Eigen::Vector2f local_uv =
                    InterpolateLocalUv(
                        triangle,
                        barycentric);

                const TextureSample sample =
                    BilinearSample(
                        surfel.texture,
                        local_uv);

                if (!sample.valid) {
                    return;
                }

                UpdateBevPixel(
                    pixel_x,
                    pixel_y,
                    sample,
                    surfel);
            });
    }
}
```

### 13.4 为什么不能只把 Texel 落到 BEV 点上

简单点投影容易产生：

* 像素空洞
* 锯齿
* Surfel 之间的细缝
* 斜坡区域采样不均匀
* 重复覆盖和局部缺失

使用带 UV 的三角形栅格化，可以连续地将 Surfel Texture 映射到 BEV。

### 13.5 多个 Surfel 落到同一 BEV Pixel

道路地图不应简单采用最高 Z 值，否则：

* 车辆顶可能覆盖道路
* 桥面可能覆盖桥下道路
* 植被可能覆盖路边地面

第一步应只选择 Road-like Surfel。

Road-like Surfel 可以通过以下传统规则识别：

* 法向接近重力方向
* 高度接近局部道路高度
* 与邻域道路表面连续
* 点云支持数量足够
* 多帧长期稳定存在
* 不属于短时间出现的动态表面

如果仍有多个道路 Surfel 覆盖同一像素，推荐按以下顺序选择：

1. 几何置信度更高
2. 平面 RMS 更小
3. 纹理观测评分更高
4. Octree 尺度更细
5. 与周围像素高度更连续

---

## 14. Surfel 边界裂缝处理

相邻 Surfel 平面可能存在毫米到厘米级误差，从而在 BEV 中产生细小黑缝。

建议依次采用：

1. 将有效 Polygon 向外扩展约 0.5 个 Texel
2. 重叠区域选择高置信度 Surfel
3. 对 1～2 个像素的小空洞进行邻域插值
4. 插值不能跨越明显高度或法向边界
5. 在平坦连续路面上使用轻微 Feather Blending

不要对整张道路图进行大范围图像修复，否则可能改变车道线和停止线的真实几何位置。

---

## 15. Tile 化输出

大范围道路地图应分 Tile 处理。

推荐参数：

```text
Tile 尺寸：50 m × 50 m
Tile 重叠：1～3 m
BEV 分辨率：2～3 cm/pixel
```

例如，50 m Tile 使用 2 cm/pixel：

[
\frac{50}{0.02}
===============

2500
]

因此图像大小为：

```text
2500 × 2500 pixels
```

未压缩 RGB 大小约为：

[
2500
\times
2500
\times
3
\approx
18.75\text{ MB}
]

每个 Tile 只加载：

* 与 Tile AABB 相交的 Surfel
* 能够观察这些 Surfel 的 Camera Frames
* Tile 周围一定范围内的当前帧动态遮挡点

Tile 重叠区域按照 Confidence 合并。

---

## 16. 推荐输出的辅助图层

### 16.1 RGB Map

```text
rgb_bev.png
```

保存最终道路正射 RGB 图。

### 16.2 Height Map

```text
height_bev.exr
```

每个 BEV Pixel 输出对应 Surfel 平面上的实际高度：

[
z(x,y)
]

### 16.3 Normal Map

```text
normal_bev.exr
```

输出：

[
(n_x,n_y,n_z)
]

可以用于：

* 道路坡度计算
* 横坡分析
* 路沿和几何边缘识别

### 16.4 Confidence Map

```text
confidence_bev.exr
```

综合以下因素：

* 平面拟合质量
* Camera 观察角
* GSD
* 图像清晰度
* 遮挡检查
* 观测数量
* 多帧颜色一致性

### 16.5 Source Map

```text
source_frame_bev.png
source_camera_bev.png
surfel_id_bev.png
```

保存每个 BEV Pixel 的：

* Source Frame ID
* Camera ID
* Surfel ID

这是生产环境中非常重要的可追溯信息。

### 16.6 LiDAR Intensity Map

```text
intensity_bev.png
```

将 LiDAR Intensity 融合到同一 BEV 坐标系。

道路标线在阴影或强光条件下 RGB 对比度较低时，Intensity Map 可以作为重要补充。

---

## 17. 第一版 MVP 范围

建议第一版实现以下模块：

```text
1. 使用最终 LIO 轨迹重新拼接点云
2. 地面和道路候选点过滤
3. 40/20/10/5 cm 自适应 Octree
4. PCA 平面拟合
5. Plane ∩ Voxel Polygon
6. 3 cm/texel Surfel Texture
7. Camera Pose 时间插值
8. Texel 到 Camera 投影
9. 双线性 RGB 采样
10. 当前 Scan Depth 遮挡检查
11. Top-1 最佳观测
12. 正交三角形 Rasterization
13. RGB、Confidence 和 Source ID 输出
```

第一版暂时不实现：

* 神经网络
* NeRF 或 Gaussian Splatting
* 全局光度联合优化
* 复杂纹理压缩
* 大范围图像补全
* 多帧颜色联合优化
* 完整非道路三维场景渲染

---

## 18. 推荐初始参数

| 参数                      |          建议初值 |
| ----------------------- | ------------: |
| Octree 尺度               | 40/20/10/5 cm |
| Texture Resolution      |    3 cm/texel |
| BEV Resolution          |    3 cm/pixel |
| 最大相机距离                  |       30～40 m |
| 最大观察角                   |           70° |
| 颜色融合方式                  |         Top-1 |
| 图像边界 Margin             |      20～50 px |
| Depth Consistency       | 5～15 cm，随距离变化 |
| Tile Size               |          50 m |
| Tile Overlap            |           2 m |
| Surfel Boundary Overlap |   0.5～1 texel |

---

## 19. 质量问题排查顺序

如果最终 BEV 中出现重影或错位，建议按照以下顺序检查：

1. LiDAR 单帧是否正确逐点运动补偿
2. LIO 局部轨迹是否准确
3. Camera 时间戳是否对应真实曝光时刻
4. Camera Pose 插值是否正确
5. LiDAR–Camera 外参是否准确
6. 相机畸变模型是否正确
7. Rolling Shutter 是否需要补偿
8. 动态遮挡判断是否正确
9. 最佳观测评分是否合理
10. Surfel 平面拟合和边界是否正确
11. 最后再调整颜色融合方法

常见现象与原因：

| 现象            | 优先怀疑                      |
| ------------- | ------------------------- |
| 所有投影同方向固定偏移   | 外参误差                      |
| 偏移随车速变化       | 时间同步误差                    |
| 同一图像不同位置误差不同  | 畸变模型或 Rolling Shutter     |
| 同一路面出现双车道线    | 轨迹误差或多帧平均                 |
| 道路上出现车辆颜色     | 动态遮挡判断不足                  |
| 相邻 Surfel 有黑缝 | Polygon 范围或 Rasterization |
| 车道线模糊         | Top-K 过多或 Pose 精度不足       |
| 远处纹理严重拉伸      | GSD 和观察角筛选不足              |

---

## 20. 最终数据流

```text
final_lidar_map
        │
        ▼
adaptive_octree
        │
        ▼
planar_leaf_nodes
        │
        ▼
cube_bounded_surfel_polygons
        │
        ▼
fixed_metric_texture_grids
        │
        ▼
camera_candidate_search
        │
        ▼
camera_pose_interpolation
        │
        ▼
visibility_validation
        │
        ▼
best_view_rgb_sampling
        │
        ▼
textured_surfel_map
        │
        ▼
orthographic_polygon_rasterization
        │
        ▼
top_down_rgb_road_map
```

---

## 21. 核心结论

该方案的核心原则是：

> 几何由 LiDAR 和 Surfel 平面保证准确性，纹理由 Camera 保证清晰度，最终 BEV 由正交多边形栅格化生成，而不是把稀疏彩色点直接投影到二维图像。

Textured Surfel 中：

```text
几何部分
├── 自适应尺度
├── 平面中心
├── 法向
├── 协方差
└── Plane–Voxel 有效边界

纹理部分
├── 固定米制分辨率
├── RGB
├── Confidence
├── Source Frame
├── Camera ID
└── GSD
```

这种设计可以同时满足：

* 大面积平坦道路使用较少几何 Primitive
* 车道线等精细纹理保持 2～3 cm 级采样
* 支持坡道和非水平路面
* 每个 BEV Pixel 都能追溯到真实三维表面和原始相机图像
* 不依赖 NeRF、GS 或其他神经网络
