# Road Layering 轨迹分层模块设计文档

## 1. 背景

在离线地图生产过程中，通常需要将多趟采集 Trip 的 LiDAR 数据基于优化后的车辆轨迹拼接成 BEV Intensity Image。

在存在高架、地面道路、下穿道路、匝道等多层道路结构的场景中，不同道路层在 XY 平面上可能发生重叠。如果直接根据所有 Trip 的优化轨迹累计点云并生成 BEV，会导致：

* 高架道路与地面道路重叠到同一张 BEV。
* 不同高度层的车道线、道路边界等结构相互污染。
* Height Map、Intensity Map 等二维地图资产出现多层混叠。
* 后续自动标注模型难以正确理解道路结构。

因此需要在地图构建之前增加 **Road Layering** 模块，对多趟轨迹进行分层，将属于同一道路 Surface 的 Trip 数据聚合在一起，再分别构建地图。

---

# 2. 目标

Road Layering 模块输入：

```text
Multi-trip optimized trajectories
```

输出：

```text
Surface 0
├── TripSegment
├── TripSegment
└── ...

Surface 1
├── TripSegment
├── TripSegment
└── ...

Ramp / Transition
├── TripSegment
└── ...
```

核心目标是：

> 根据多趟优化轨迹之间的空间位置、高度关系和轨迹连续关系，自动判断哪些 TripSegment 属于同一个道路 Surface，并识别不同高度层之间的分层关系。

最终不同 Surface 的数据分别用于：

```text
Surface 0 → LiDAR Frames → BEV Intensity 0
Surface 1 → LiDAR Frames → BEV Intensity 1
...
```

---

# 3. 典型场景

## 3.1 普通单层道路

```text
Trip A ────────────────────

Trip B ────────────────────

Trip C ────────────────────
```

所有轨迹高度接近：

```text
Surface 0
├── Trip A
├── Trip B
└── Trip C
```

---

## 3.2 高架与地面道路

XY 平面重叠：

```text
             Elevated

Trip C =====================     z ≈ 8 m
Trip D =====================


             Ground

Trip A ---------------------     z ≈ 0 m
Trip B ---------------------
```

应该输出：

```text
Surface 0
├── Trip A
└── Trip B

Surface 1
├── Trip C
└── Trip D
```

---

## 3.3 地面 → 匝道 → 高架

```text
Ground
────────────────────
                    \
                     \
                      \ Ramp
                       \
                        ───────────────── Elevated
```

同一 Trip 的轨迹可能为：

```text
0 m → 0 m → 1 m → 3 m → 5 m → 8 m → 8 m
```

不能因为这些轨迹点属于同一个 Trip，就认为它们属于同一个 Surface。

因此需要显式区分：

```text
LEVEL
RAMP / TRANSITION
```

---

# 4. 总体架构

整体流程：

```text
Multi-trip Optimized Trajectories
                │
                ▼
        1. Trip Segmentation
                │
                ▼
          TripSegments
                │
                ▼
        2. Spatial Index
           XY Grid Hash
                │
                ▼
     Candidate Segment Pairs
                │
                ▼
  3. Pairwise Relation Estimation
                │
       ┌────────┼────────┐
       ▼        ▼        ▼
 SAME_LEVEL CANNOT_LINK NONE
                │
                ▼
      4. Segment Graph
                │
                ▼
  5. Constrained Graph Clustering
                │
                ▼
           Surface Groups
                │
                ▼
      6. Ramp Association
                │
                ▼
       Layered Road Structure
                │
                ▼
       Layered BEV Mapping
```

---

# 5. Trip Segmentation

## 5.1 目的

原始轨迹由大量 Pose 构成，例如：

```text
P0 → P1 → P2 → P3 → ... → PN
```

直接以每个 Pose 作为图节点会造成：

* 图节点数量过大。
* 空间搜索规模过大。
* 单点 Z 噪声影响明显。
* 很难计算稳定的坡度和道路方向。

因此首先将单趟轨迹按照行驶距离划分为 TripSegment。

例如：

```text
Trip A

Pose Pose Pose Pose Pose Pose Pose Pose
 └──── Segment 0 ────┘
                  └──── Segment 1 ────┘
```

建议第一版采用固定距离切分，例如：

```text
segment_length = 5 ~ 10 m
```

具体参数通过数据实验确定。

---

# 6. TripSegment 数据结构

建议：

```cpp
enum class SegmentType {
    LEVEL,
    RAMP_UP,
    RAMP_DOWN,
    UNKNOWN
};

struct TripSegment {
    uint64_t id;

    uint64_t trip_id;
    uint32_t segment_id;

    SegmentType type;

    std::vector<Pose> poses;

    // XY spatial information
    BoundingBox2D bbox;

    Eigen::Vector3d center;

    // Geometry
    double mean_z;
    double z_std;

    double heading;
    double grade;

    double length;
};
```

其中：

```text
mean_z
```

表示 Segment 平均高度；

```text
heading
```

表示 Segment 主方向；

```text
grade = dz / ds
```

表示沿轨迹方向的平均坡度。

---

# 7. Ramp Detection

## 7.1 为什么 Ramp 需要单独检测

如果 Ramp 与普通 LEVEL Segment 一起参与同层聚类，可能产生：

```text
Ground
   ↕ SAME
Ramp Start
   ↕ SAME
Ramp Middle
   ↕ SAME
Ramp End
   ↕ SAME
Elevated
```

通过 SAME_LEVEL 的传递关系最终导致：

```text
Ground == Elevated
```

因此建议在 Trip Segmentation 阶段初步识别：

```text
RAMP_CANDIDATE
```

Ramp 不直接参与普通 LEVEL Surface 的 Union-Find 聚类。

---

## 7.2 Ramp 检测方法

不要直接使用相邻 Pose：

```text
dz / ds
```

因为 Z 可能存在噪声。

建议使用滑动空间窗口，在局部拟合：

```text
z(s) = a * s + b
```

其中：

```text
a = grade
```

根据连续多个窗口的 grade 判断：

```text
LEVEL

RAMP_UP

RAMP_DOWN
```

建议增加：

* grade threshold
* hysteresis
* minimum ramp length
* minimum accumulated height change

避免由于轨迹噪声或普通路面起伏频繁切换状态。

---

# 8. XY Spatial Index

## 8.1 为什么需要空间索引

假设有：

```text
N = 100000 TripSegments
```

如果所有不同 Trip 的 Segment 两两比较：

```text
O(N²)
```

计算量不可接受。

因此使用 XY Grid Hash 建立空间索引，只比较空间上可能接近的 Segment。

---

# 9. Grid Hash

地图 XY 平面划分为固定大小 Grid：

```text
┌──────┬──────┬──────┐
│      │      │      │
├──────┼──────┼──────┤
│      │      │      │
├──────┼──────┼──────┤
│      │      │      │
└──────┴──────┴──────┘
```

例如：

```text
grid_size = 10 ~ 20 m
```

数据结构：

```cpp
struct GridId {
    int x;
    int y;
};

std::unordered_map<GridId,
                   std::vector<SegmentId>> spatial_grid;
```

---

# 10. Segment 注册到 Grid

TripSegment 是一段线，而不是单点。

因此不能简单使用：

```text
segment.center
```

决定 Segment 属于哪个 Grid。

应该使用：

```text
segment.bbox
```

将 Segment 注册到 BBox 覆盖的所有 Grid。

例如：

```text
┌──────┬──────┬──────┐
│      │      │      │
├──────┼──────┼──────┤
│   ─────────────────│  Segment
├──────┼──────┼──────┤
│      │      │      │
└──────┴──────┴──────┘
```

该 Segment 可以同时出现在多个 Grid 的 index 中。

---

# 11. Expanded Bounding Box Candidate Search

为了保证不会由于 Grid 边界漏掉附近 Segment，对 Segment A：

```text
bbox(A)
```

向 XY 四周扩张：

```text
search_radius
```

得到：

```text
expanded_bbox(A)
```

即：

```text
xmin' = xmin - search_radius
xmax' = xmax + search_radius

ymin' = ymin - search_radius
ymax' = ymax + search_radius
```

示意：

```text
Expanded BBox

┌──────────────────────────┐
│                          │
│     ┌──────────────┐     │
│     │  Segment A   │     │
│     └──────────────┘     │
│                          │
└──────────────────────────┘
```

查询：

```text
expanded_bbox(A)
        ↓
覆盖哪些 Grid
        ↓
读取这些 Grid 内所有 Segment
        ↓
Candidate Segments
```

这样即使两个 Segment 位于不同 Grid，只要实际空间距离较近，也会进入候选集合。

---

# 12. Candidate Pair 去重

同一个 Segment 可能存在于多个 Grid，因此：

```text
A-B
```

可能被发现多次。

需要使用：

```cpp
std::unordered_set<PairKey> visited_pairs;
```

或者：

```cpp
PairKey = (min(A.id, B.id), max(A.id, B.id));
```

进行去重。

---

# 13. Segment Pair Relation Estimation

Spatial Grid 只负责：

> Candidate Generation。

进入候选集合后，需要精确判断两个 Segment 之间的关系。

接口可以设计为：

```cpp
LayerRelation EstimateLayerRelation(
    const TripSegment& a,
    const TripSegment& b);
```

---

# 14. Layer Relation

定义：

```cpp
enum class LayerRelation {
    SAME_LEVEL,
    CANNOT_LINK,
    NO_RELATION
};
```

对于不同 Trip：

```text
TripSegment A
TripSegment B
```

主要检查：

```text
XY distance / overlap
Z difference
heading difference
grade difference
```

---

# 15. SAME_LEVEL

如果：

```text
XY 距离近
+
高度接近
+
方向合理
+
坡度一致
```

则认为：

```text
A SAME_LEVEL B
```

例如：

```text
Trip A    ----------------

Trip B      --------------
```

且：

```text
Δz = 0.15 m
```

则创建：

```text
A ===== B
```

建议为 SAME_LEVEL Edge 保存置信度：

```cpp
struct SameLevelEdge {
    SegmentId u;
    SegmentId v;

    double score;
};
```

score 可以综合：

```text
Z consistency
Heading consistency
XY overlap
Grade consistency
```

---

# 16. CANNOT_LINK

如果两个 Segment：

```text
XY 很接近或存在明显覆盖
```

但：

```text
|Δz| > layer_separation_threshold
```

则认为它们明显处于不同道路层：

```text
A -------X------- B
```

建立：

```text
CANNOT_LINK
```

例如：

```text
Elevated
A =================   z = 8 m

        X

B -----------------   z = 0 m
Ground
```

CANNOT_LINK 是后续防止上下层错误合并的重要约束。

---

# 17. NO_RELATION

如果：

```text
距离较远
方向关系异常
空间重叠不足
数据不确定
```

则不建立任何分层关系：

```text
NO_RELATION
```

原则上：

> 宁可少建立一条 SAME_LEVEL Edge，也不要错误连接高架和地面道路。

---

# 18. Segment Graph

所有 TripSegment 构成 Graph：

```text
G = (V, E)
```

其中：

```text
V = TripSegments
```

图中存在三类 Edge。

---

## 18.1 CONTINUITY Edge

同一 Trip 相邻 Segment：

```text
A0 → A1 → A2 → A3
```

建立：

```text
CONTINUITY
```

表示：

> 车辆沿轨迹连续从一个 Segment 行驶到下一个 Segment。

注意：

```text
CONTINUITY != SAME_LEVEL
```

例如：

```text
Ground → Ramp → Elevated
```

同样具有 CONTINUITY。

因此 CONTINUITY Edge **不直接参与 Surface Union-Find 聚类**。

---

## 18.2 SAME_LEVEL Edge

不同 Trip 之间：

```text
A ===== B
```

表示两个 Segment 很可能属于同一个道路 Surface。

SAME_LEVEL Edge 是 Surface Clustering 的 Positive Edge。

---

## 18.3 CANNOT_LINK Constraint

不同 Trip：

```text
A -----X----- B
```

表示两个 Segment 一定或大概率不能属于同一个 Surface。

用于限制 Cluster Merge。

---

# 19. Graph 示例

```text
Trip A Ground

A0 → A1 → A2 → A3
     CONTINUITY

     ║     ║     ║
     ║     ║     ║
     ║     ║     ║ SAME_LEVEL

B0 → B1 → B2 → B3

     X     X     X
     │     │     │
     │     │     │ CANNOT_LINK

C0 → C1 → C2 → C3

Trip C Elevated
```

---

# 20. Surface Clustering

V1 推荐采用：

```text
Positive Edge Sorting
        +
Union-Find
        +
Cannot-Link Constraint
```

即：

> Constrained Agglomerative Clustering。

---

# 21. Union-Find 初始化

所有 TripSegment 开始都是单独 Cluster：

```text
{A}

{B}

{C}

{D}
```

---

# 22. Positive Edge Sorting

所有 SAME_LEVEL Edge：

```text
A-B  0.98
B-C  0.93
D-E  0.91
C-F  0.72
...
```

按照 score：

```text
高 → 低
```

排序。

优先处理置信度最高的 SAME_LEVEL 关系。

---

# 23. Cluster Merge

对于一条 Edge：

```text
A-B
```

找到：

```text
Cluster(A)
Cluster(B)
```

如果已经属于同一个 Cluster：

```text
continue
```

否则检查：

```text
Cluster(A)
和
Cluster(B)
```

之间是否存在任何 CANNOT_LINK。

---

# 24. Cannot-Link Check

例如：

```text
Cluster A:

A1
A2
A3


Cluster B:

B1
B2
```

如果存在：

```text
A2 X B1
```

那么：

```text
Cluster A
```

和：

```text
Cluster B
```

禁止合并。

即使当前处理的 SAME_LEVEL Edge 是：

```text
A3-B2
```

也必须拒绝。

因此 Cannot-Link 是 **Cluster-Level Constraint**，而不是只检查当前两个节点。

---

# 25. Cluster 数据结构

第一版可以：

```cpp
struct Cluster {
    std::unordered_set<SegmentId> members;

    std::unordered_set<SegmentId> forbidden;
};
```

其中：

```text
members
```

表示当前 Cluster 中的 TripSegment。

```text
forbidden
```

表示与 Cluster 内节点存在 CANNOT_LINK 的 Segment。

两个 Cluster 合并前：

```text
A.members ∩ B.forbidden
```

或者：

```text
B.members ∩ A.forbidden
```

只要非空：

```text
Reject Merge
```

---

# 26. Clustering 伪代码

```cpp
UnionFind uf(num_segments);

SortByScoreDescending(same_level_edges);

for (const auto& edge : same_level_edges) {
    int cluster_a = uf.Find(edge.u);
    int cluster_b = uf.Find(edge.v);

    if (cluster_a == cluster_b) {
        continue;
    }

    if (HasCannotLink(cluster_a, cluster_b)) {
        continue;
    }

    uf.Union(cluster_a, cluster_b);
}
```

最终：

```cpp
uf.Find(segment_id)
```

即为该 Segment 所属 Surface Group。

---

# 27. Surface

Surface 不需要在算法前期显式创建。

它可以直接作为 Graph Clustering 的输出。

例如：

```text
Surface 0
├── Trip1_S10
├── Trip2_S31
├── Trip4_S08
└── Trip7_S12


Surface 1
├── Trip3_S15
├── Trip5_S27
└── Trip8_S04
```

数据结构：

```cpp
struct RoadSurface {
    SurfaceId id;

    std::vector<SegmentId> segments;
};
```

因此：

> Surface 是一组通过 SAME_LEVEL 关系聚合，并且内部不存在 CANNOT_LINK 冲突的 TripSegment。

---

# 28. Ramp 处理

Ramp 不建议直接参与 LEVEL Surface 的 Union-Find。

首先：

```text
Trip Segmentation
```

阶段产生：

```text
RAMP_UP
RAMP_DOWN
```

Segment。

例如：

```text
Ground Surface
      │
      │
      ▼
RampSegment 0
      ↓
RampSegment 1
      ↓
RampSegment 2
      │
      ▼
Elevated Surface
```

Surface Clustering 完成后，再根据同一 Trip 的 CONTINUITY：

```text
Surface A
    ↓
Ramp
    ↓
Surface B
```

建立层间 Transition。

---

# 29. Ramp Group

多趟 Trip 可能经过同一个 Ramp：

```text
Trip A Ramp
Trip B Ramp
Trip C Ramp
```

可以进一步根据：

```text
XY overlap
grade
heading
height profile
```

将多个 Ramp Segment 聚合为：

```text
RampGroup
```

例如：

```cpp
struct RampGroup {
    RampId id;

    std::vector<SegmentId> segments;

    SurfaceId from_surface;
    SurfaceId to_surface;
};
```

这样可以恢复：

```text
Ground Surface
      ↓
   RampGroup
      ↓
Elevated Surface
```

---

# 30. 输出道路层结构

最终输出可以表示为：

```text
RoadLayeringResult
│
├── Surface 0
│     ├── Segment ...
│     └── Segment ...
│
├── Surface 1
│     ├── Segment ...
│     └── Segment ...
│
└── Ramp
      ├── from_surface = 0
      └── to_surface   = 1
```

例如：

```cpp
struct RoadLayeringResult {
    std::vector<RoadSurface> surfaces;
    std::vector<RampGroup> ramps;
};
```

---

# 31. BEV Mapping

完成 Road Layering 后：

```text
Surface 0
        ↓
所有属于该 Surface 的 TripSegment
        ↓
对应的 LiDAR Frames
        ↓
Point Cloud Accumulation
        ↓
BEV Intensity 0
```

同理：

```text
Surface 1
        ↓
BEV Intensity 1
```

因此原来的：

```text
All Trips
    ↓
One BEV
```

变成：

```text
All Trips
    ↓
Road Layering
    ↓
┌───────────┬───────────┬───────────┐
│ Surface 0 │ Surface 1 │ Surface 2 │
└───────────┴───────────┴───────────┘
      ↓           ↓           ↓
    BEV 0       BEV 1       BEV 2
```

---

# 32. 推荐模块结构

```text
road_layering/
│
├── trip_segment.h
├── trip_segmenter.h
├── trip_segmenter.cc
│
├── ramp_detector.h
├── ramp_detector.cc
│
├── spatial_grid.h
├── spatial_grid.cc
│
├── layer_relation.h
├── layer_relation_estimator.h
├── layer_relation_estimator.cc
│
├── segment_graph.h
├── segment_graph.cc
│
├── union_find.h
│
├── layer_clusterer.h
├── layer_clusterer.cc
│
├── road_surface.h
│
├── road_layering.h
└── road_layering.cc
```

顶层接口：

```cpp
class RoadLayering {
public:
    RoadLayeringResult Run(
        const std::vector<Trajectory>& trajectories);
};
```

---

# 33. 推荐配置参数

第一版可以包含：

```text
segment_length

grid_size
search_radius

same_level_max_z_diff
different_level_min_z_diff

max_heading_diff
max_grade_diff

ramp_grade_threshold
ramp_min_length
ramp_min_height_change

same_level_min_score
```

所有阈值通过实际数据统计确定，不建议在算法中写死。

---

# 34. V1 与后续演进

## V1

优先实现：

```text
Fixed-distance Trip Segmentation

+

Ramp Candidate Detection

+

Grid Hash Spatial Index

+

Expanded BBox Candidate Search

+

SAME_LEVEL / CANNOT_LINK

+

Sorted Positive Edges

+

Constrained Union-Find
```

优点：

* 实现简单。
* 算法可解释。
* 容易可视化和 Debug。
* 不需要复杂图优化库。
* 可以快速验证轨迹分层是否有效。

---

## V2

如果 V1 出现较多：

```text
Greedy Merge Error
```

可以将 Graph Solver 升级为：

```text
Minimum Cost Multicut
```

或者：

```text
Signed Graph Clustering
```

统一优化 SAME_LEVEL 与 DIFFERENT_LEVEL 的全局一致性。

前面的：

```text
TripSegment
Spatial Search
Edge Construction
Ramp Detection
```

都可以保持不变，只替换 Graph Solver。

---

# 35. 关键设计原则

整个模块需要遵守以下几个原则。

### 原则一：Same Trip 不等于 Same Surface

```text
CONTINUITY != SAME_LEVEL
```

因为：

```text
Ground → Ramp → Elevated
```

也是连续轨迹。

---

### 原则二：Spatial Grid 只负责 Candidate Generation

```text
Same Grid
```

不代表：

```text
Same Layer
```

真正的 Layer Relation 必须通过：

```text
XY geometry
+
Z
+
Heading
+
Grade
```

判断。

---

### 原则三：宁可断开，也不要错误合并

错误把：

```text
Ground
```

和：

```text
Elevated
```

合并会直接污染整张 BEV。

因此对于低置信度关系：

```text
NO_RELATION
```

通常优于错误的：

```text
SAME_LEVEL
```

---

### 原则四：Ramp 是 Transition，不是普通 Same-Level Connection

Ramp 负责表达：

```text
Surface A
    ↓
Surface B
```

但不能成为：

```text
Surface A == Surface B
```

的传递路径。

---

# 36. 总结

Road Layering 的核心流程可以概括为：

```text
1. 将每趟优化轨迹按照距离切分为 TripSegment，并检测 Ramp Candidate。

2. 使用 XY Grid + Expanded Bounding Box 找到空间上可能接近的不同 TripSegment。

3. 对候选 Segment Pair 根据 XY、Z、Heading、Grade 判断：
   - SAME_LEVEL
   - CANNOT_LINK
   - NO_RELATION

4. 将 TripSegment 构造成 Graph：
   - 同一 Trip 相邻 Segment → CONTINUITY
   - 同层 Segment → SAME_LEVEL
   - 上下层 Segment → CANNOT_LINK

5. 使用 Positive Edge Sorting + Union-Find + Cannot-Link Constraint 对 LEVEL Segment 聚类。

6. 每个 Cluster 形成一个 Road Surface。

7. Ramp Segment 根据 CONTINUITY 关系关联不同 Road Surface，形成层间 Transition。

8. 根据 Road Surface 分别组织 LiDAR 数据并生成独立 BEV Intensity Image。
```

最终实现从：

```text
Multi-trip Trajectories
```

到：

```text
Layered Road Surfaces
```

的转换，为后续多层道路场景下的 BEV、Height Map、Surfel Map 以及自动标注提供正确的空间分层基础。
