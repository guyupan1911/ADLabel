# ADLabel 数据湖与任务框架

本文总结 ADLabel 的核心架构定位：ADLabel 不是单一标注工具，而是建立在数据湖之上的自动驾驶数据生产平台。它通过数据库管理索引和版本，通过任务系统调度不同算法，持续生成建图、标注和训练所需的数据资产。

## 核心定位

ADLabel 可以概括为：

```text
ADLabel = 数据湖 + 数据库索引层 + 任务系统 + 算法执行器 + 标注/版本管理
```

其中：

- 数据湖保存原始数据和派生数据。
- 数据库保存 metadata、索引、关系、版本和查询入口。
- 任务系统从数据库中选取一批数据，固化为任务输入，再调用算法执行。
- 算法输出结果后，产物文件回到数据湖，产物索引回写数据库。

## 核心抽象

ADLabel 的核心抽象是：

```text
Data Lake + Database + Task
```

也就是：

```text
数据池保存文件
数据库管理索引和版本
任务消费一批数据并产生新的数据资产
```

所有业务都尽量统一成同一个模式：

```text
查询数据 -> 固化任务输入 -> 执行算法/人工标注 -> 保存产物 -> 回写索引 -> 形成版本
```

因此，ADLabel 不需要为每一种算法单独设计一套数据系统。无论是 BEV 生成、LIO 建图、语义标注、地图更新，还是训练数据导出，本质上都是不同类型的任务。

```text
raw data + metadata
  -> task input manifest
  -> algorithm / human annotation
  -> artifact
  -> indexed result
  -> versioned dataset / map / annotation
```

这个抽象里有五个稳定对象：

| 对象 | 含义 |
|---|---|
| Data Asset | 数据湖中的文件，例如 image、lidar、BEV、map tile、model output |
| Index Record | 数据库里的索引记录，例如 frame、pose、calibration、annotation |
| Task | 一次可复现的数据生产过程 |
| Manifest | 任务输入快照，例如 frames.jsonl、frames.parquet、frames.pb |
| Version | 可被训练或发布引用的数据版本，例如 semantic_map_version、dataset_version |

## 总体分层

```text
Raw / Derived Data Lake
  rosbag / mcap / nuScenes / KITTI / image / lidar / BEV / map tile / model output
        |
        v
Index / Metadata Database
  frames / assets / poses / calibrations / geohash / map versions / annotations
        |
        v
Task System
  query data -> build manifest -> run algorithm -> register outputs
        |
        v
Algorithm Executors
  lidar_topdown / LIO / loop closure / pose graph / map update / auto labeling
        |
        v
Annotation and Training Data
  semantic annotations / tracks / dataset versions / training manifests
```

## 数据湖

数据湖负责保存大文件，本身不承担复杂查询。

典型内容包括：

- 原始数据：`rosbag`、`mcap`、nuScenes、KITTI、图片、点云。
- 派生数据：BEV intensity image、彩色 BEV、拼接点云、localization map tile。
- 算法产物：LIO 轨迹、pose graph 结果、map patch、自动标注结果。
- 训练导出：训练 manifest、样本包、label version 快照。

原则是：

```text
大文件放数据湖，数据库只保存 URI 和可查询字段。
```

## 数据库索引层

数据库管理的是数据的目录、索引、关系和版本，不直接保存大文件。

第一阶段建议的核心表：

```text
datasets
scenes / trips
frames
frame_assets
ego_poses / frame_poses
calibrations
jobs
artifacts
bev_images
semantic_annotations
annotation_versions
dataset_versions
```

建图和定位任务需要额外管理：

```text
localization_map_versions
localization_map_tiles
map_update_jobs
pose_edges
map_matching_pairs
map_patches
map_patch_tiles
```

不同数据库可以负责不同类型的查询：

| 数据系统 | 主要职责 |
|---|---|
| PostgreSQL/PostGIS | 在线标注、空间查询、pose、map version、语义几何 |
| Parquet / Iceberg / Delta | 大规模离线训练样本索引和 dataset version |
| ClickHouse | 任务日志、质量指标、统计分析 |
| LanceDB / Milvus | embedding 检索、相似场景搜索、hard case mining |
| 对象存储 / NAS | 保存图片、点云、BEV、map tile、模型输出 |

## Frame 抽象

Frame 是 ADLabel 使用原始数据的统一索引概念。

对 nuScenes 来说：

```text
nuScenes sample      -> ADLabel frame
nuScenes sample_data -> ADLabel frame_asset
```

对 rosbag / mcap 来说：

```text
某个时间戳的一组传感器消息 -> ADLabel frame
具体 sensor topic/message -> frame_asset
```

Frame 本身不必强制保存为 proto 文件。推荐关系是：

```text
数据库表 = 主索引和真相来源
FrameProto / manifest = 下游任务输入快照
```

也就是说，任务运行前可以从数据库查询一批 frames，然后组装为：

```text
frames.pb
frames.jsonl
frames.parquet
```

这些文件用于保证任务输入可复现，但不替代数据库。

## 任务系统

任务系统的标准流程是：

```text
1. 用户或系统创建任务
2. 根据 scene / trip / bbox / geohash / time range 查询数据库
3. 固化任务输入 manifest
4. 调用算法执行器
5. 产物写入数据湖
6. 产物索引和质量指标回写数据库
```

任务表可以抽象为：

```text
jobs
  job_id
  job_type
  status
  input_query
  input_manifest_uri
  config_uri
  output_uri
  created_at
  finished_at
```

## MapReduce 执行模式

ADLabel 的任务系统可以衔接类似 main repo localization map builder 的 MapReduce 框架。核心思想是：

```text
Database = indexed source of truth
Manifest = frozen task input
MapReduce = distributed execution pattern
Data Lake = artifact storage
Registrar = database write-back
```

也就是说，数据库不直接承担大规模计算和 shuffle，而是负责生成任务输入和登记任务输出。

标准流程：

```text
1. TaskPlanner 查询数据库
   frames / poses / assets / calibrations

2. 生成任务输入 manifest
   frames.jsonl / frames.parquet / frames.pb

3. Mapper 读取 manifest
   emit(key, frame_ref)

4. Shuffle / group by key
   key 可以是 geohash、trip_id、scene_id、time window

5. Reducer 处理一个 key 下的所有数据
   读取数据湖中的 lidar/image/pose
   执行算法
   输出 artifact

6. Registrar 统一回写数据库
   jobs / artifacts / bev_products / frame_poses / map_patches
```

对应关系：

| MapReduce 角色 | ADLabel 中的含义 |
|---|---|
| input | 数据库查询出的任务 manifest |
| mapper | 按 geohash/trip/time 对 frame 分桶 |
| shuffle | 按 key 聚合 frame refs |
| reducer | 对一个分桶执行算法 |
| output | 数据湖中的产物文件 |
| registrar | 把产物 URI、bbox、quality、version 写回数据库 |

### 示例：按 geohash 生成 BEV topdown

当数据库中已经有大量 frames，并且希望统一按 geohash 生成一次 BEV topdown image，可以设计为：

```text
frames table
  -> mapper: 查询 frame，读取 global pose，计算 geohash
  -> emit(geohash, frame_ref)
  -> shuffle: group by geohash
  -> reducer: lidar_topdown_by_geohash
  -> output: bev_intensity / height_map / bev.jpg
  -> registrar: 回写 bev_products / height_maps / bev_frame_links
```

Mapper 输出示例：

```json
{
  "key": "9q8yyk",
  "value": {
    "frame_id": "frame_10001",
    "trip_id": "trip_001",
    "timestamp": 1531883530449377,
    "lidar_uri": "s3://bucket/trip_001/lidar/000001.bin",
    "pose": "...",
    "calibration": "..."
  }
}
```

Reducer 输入：

```text
geohash = 9q8yyk
frames = [
  frame_10001,
  frame_10002,
  frame_10003,
  ...
]
```

Reducer 输出：

```text
derived/lidar_topdown/{job_id}/9q8yyk/bev_intensity.png
derived/lidar_topdown/{job_id}/9q8yyk/height_map.tiff
derived/lidar_topdown/{job_id}/9q8yyk/bev.jpg
derived/lidar_topdown/{job_id}/9q8yyk/metadata.json
```

回写数据库：

```text
bev_products
height_maps
bev_frame_links
task_partitions
artifacts
```

### 设计注意点

1. Reducer 不应频繁在线查数据库。  
   输入 manifest 应尽量带齐 `frame_id`、`lidar_uri`、pose、calibration、sensor 信息，reducer 只读 manifest 和数据湖文件。

2. geohash tile 需要 overlap。  
   如果只把 frame 分配给 ego pose 所在 geohash，tile 边缘可能缺点云。Mapper 应支持输出当前 geohash 和邻近 geohash，或根据点云覆盖 bbox 计算相关 geohash。

3. 每个 key 是一个可重试 partition。  
   可以用 `task_partitions` 管理每个 geohash 的状态：

```text
task_partitions
  partition_id
  job_id
  partition_key
  status
  input_manifest_uri
  output_uri
  error_message
```

4. 数据库不是 shuffle 系统。  
   大规模 group by 可以由 Spark、Ray、本地 pipe、DuckDB/Parquet 完成。数据库负责索引和结果登记。

5. 所有输出必须可追溯。  
   每个 BEV / map tile / pose result 都要记录 `job_id`、输入 manifest、source frame ids、config 和版本信息。

## 第一阶段任务设计

第一阶段建议先把三个任务跑通：

```text
1. trip 定位任务：LIO + GNSS 融合定位
2. 区域建图任务：lidar_topdown 生成 BEV / height map
3. 数据湖可视化：查看数据资产、轨迹和区域覆盖
```

这三个任务都强依赖时间索引、空间索引、任务状态和产物 URI，因此第一阶段可以统一使用：

```text
PostgreSQL + PostGIS 作为主数据库
本地目录 / NAS / MinIO / S3 作为数据湖
MapLibre GL + OSM 作为默认地图可视化方案
Parquet 作为训练或批处理导出格式
```

### 任务一：trip LIO / GNSS 融合定位

目标：输入一段 trip 的 frames，运行 LIO 和 GNSS 融合定位，并回写多种 pose 结果。

```text
trip frames
  -> 固化 frames manifest
  -> LIO 生成相对轨迹
  -> GNSS / IMU / wheel 等信息融合
  -> 生成 global pose 或 fused pose
  -> 回写 frame_poses 和 trajectories
```

推荐数据库：

```text
PostgreSQL + PostGIS
```

原因：

- pose 是结构化数据，适合按 frame/time/job 查询。
- 轨迹是空间对象，适合用 PostGIS `LINESTRINGZ` 管理。
- 后续可视化、BEV 生成、语义地图反投影都依赖全局 pose。
- 同一个 frame 会有多种 pose 版本，必须保留历史而不是覆盖。

核心表：

```text
trips
frames
frame_assets
frame_poses
trajectories
jobs
```

其中 `frame_poses` 不只保存一套 pose，而是保存多种结果：

```text
pose_type:
  raw
  lio_relative
  gnss_fused
  optimized_global
```

大文件仍然放数据湖：

```text
derived/trips/{trip_id}/lio/{job_id}/trajectory.pb
derived/trips/{trip_id}/lio/{job_id}/report.json
```

数据库只保存 URI、pose、quality、job metadata。

### 任务二：lidar_topdown

目标：输入一段区域的 frames，生成静态标注底图。

```text
nuScenes / trip 入库
  -> importer 创建 frames / frame_assets / poses / calibrations
  -> 数据库查询某个 scene / bbox / geohash 的 frames
  -> 生成 frames manifest
  -> lidar_topdown 读取 lidar、pose、calibration
  -> 拼接点云
  -> 输出 BEV intensity image / height map / BEV RGB
  -> 回写 bev_products、height_maps 和 bev_frame_links
```

推荐数据库：

```text
PostgreSQL + PostGIS
```

原因：

- BEV 产物有明确空间范围，需要按 bbox/geohash 查询。
- height map 和 BEV image 是大文件，应放数据湖。
- 标注任务需要根据 BEV 的 global origin、resolution、bbox 把像素坐标转成全局坐标。
- 训练导出需要知道每个 BEV 由哪些 frames 生成。

核心表：

```text
bev_products
  bev_id
  job_id
  product_type: intensity / rgb / height_map
  uri
  width
  height
  resolution
  origin_x / origin_y / origin_z
  bbox_geom
  geohash_list

bev_frame_links
  bev_id
  frame_id

height_maps
  height_map_id
  bev_id
  uri
  resolution
  bbox_geom
```

数据湖产物示例：

```text
derived/lidar_topdown/{job_id}/bev_intensity.png
derived/lidar_topdown/{job_id}/height_map.tiff
derived/lidar_topdown/{job_id}/bev.jpg
derived/lidar_topdown/{job_id}/metadata.json
```

后续标注系统在 `bev_intensity.png` 上标注车道线、人行道、路面标志等语义几何，并借助 `height_map` 转成全局 3D 坐标。

### 任务三：数据湖可视化

目标：可视化数据湖内有哪些数据，以及哪些 trip 已经有全局定位结果。

```text
查询 datasets / trips / jobs / artifacts
  -> 展示数据湖资产列表
  -> 查询已有 global pose 的 trip
  -> 输出 trajectory GeoJSON / MVT / KML
  -> 在 OSM / Google Earth / MapLibre 地图上可视化
```

推荐数据库：

```text
PostgreSQL + PostGIS
```

原因：

- 可视化本质是空间检索：按 bbox 查询 trip、trajectory、BEV、语义地图覆盖。
- 有全局定位结果的 trip 可以存成 PostGIS `LINESTRINGZ`。
- 前端可以直接从 PostGIS 输出 GeoJSON 或 MVT vector tile。
- 数据湖资产也需要统一 registry，避免只靠目录树浏览。

核心表：

```text
data_assets
  asset_id
  asset_type: raw_trip / trajectory / bev_image / height_map / semantic_map
  uri
  dataset_id
  trip_id
  job_id
  bbox_geom
  time_range
  status

trajectories
  trajectory_id
  trip_id
  pose_type
  job_id
  trajectory_geom
  quality_summary
```

推荐地图方案：

```text
开发和默认部署：MapLibre GL + OpenStreetMap
需要商业地图或卫星影像：Google Maps / Google Earth / Mapbox
需要完全自控：自建 OSM tile server + MapLibre GL
```

系统内部建议以 Web 地图为主：

```text
PostGIS -> GeoJSON/MVT API -> MapLibre GL / OSM
```

Google Earth 可以作为导出能力：

```text
trajectory.kml
```

### 三个任务的数据库选择总结

| 任务 | 推荐数据库 | 文件存储 | 说明 |
|---|---|---|---|
| LIO / GNSS 融合定位 | PostgreSQL + PostGIS | 数据湖 | 管 pose、trajectory、quality、job metadata |
| lidar_topdown | PostgreSQL + PostGIS | 数据湖 | BEV/height map 文件放数据湖，bbox/index 放数据库 |
| 数据湖可视化 | PostgreSQL + PostGIS | 数据湖 | 用空间查询驱动地图可视化 |
| 训练导出 | Parquet，可后续 Iceberg | 数据湖 | 给训练 repo 批量读取 |
| 质量统计 | 后续 ClickHouse | 数据湖/日志 | 大规模任务指标分析 |
| 相似场景检索 | 后续 LanceDB | 数据湖 | embedding / hard case mining |

第一阶段不要引入过多数据库。PostgreSQL/PostGIS 已经能覆盖这三个任务的核心需求：空间查询、事务、版本管理、任务状态和可视化索引。

## 后续任务：map_update

目标：输入一个 trip 的 frames，跑 LIO 和全局优化，更新 trip 的全局定位结果，必要时更新 base map。

```text
trip frames
  -> LIO 生成相对轨迹
  -> 查询 base map 相关 geohash / frames
  -> 回环检测和 base map matching
  -> pose graph optimization
  -> 写入 optimized frame poses
  -> 生成 map patch 或新的 localization map version
```

数据库需要保存的不只是最终 pose，还包括：

```text
map_update_jobs
frame_poses
pose_edges
map_matching_pairs
localization_map_versions
localization_map_tiles
map_patches
map_patch_tiles
```

这样系统可以追溯：

- 这个 trip 用了哪个 base map。
- 哪些 frame 和 base map 匹配成功。
- 哪些约束进入了 pose graph。
- 哪些 geohash 被更新。
- 新 map version 和旧 map version 的差异。

## 标注与训练数据

静态标注流程：

```text
BEV intensity image
  -> 人工或自动标注 lane / crosswalk / road marking
  -> semantic_annotations
  -> annotation_versions
  -> dataset_versions
  -> training manifest
```

训练 repo 不需要直接理解 ADLabel 内部实现。它只需要读取 ADLabel 导出的 dataset version：

```text
dataset_version
  frames
  sensor uri
  pose
  calibration
  BEV image
  annotation geometry
```

## 设计原则

1. 原始数据不改动，只通过 URI 被引用。
2. 数据库管理索引、关系、版本和查询，不保存大文件。
3. Frame 是统一数据抽象，FrameProto 是可选任务输入格式。
4. 每个任务都要有输入 manifest，保证可复现。
5. 算法结果既要保存产物文件，也要保存可查询的 metadata。
6. 建图、标注、训练导出都应围绕 dataset/map/annotation version 组织。

## 一句话总结

ADLabel 的合理定位是：基于数据湖的自动驾驶数据生产平台。它用数据库管理不同类型的数据索引和版本，用任务系统调度建图、定位、标注和训练导出算法，最终为自动驾驶模型训练持续生产可追溯、可复现、可查询的数据资产。
