# ADLabel 系统架构与地图标注设计

本文是 ADLabel 当前唯一保留、唯一权威的系统设计文档，汇总并取代此前分散的架构、数据库、数据湖任务、LaneMapDB分析和Codex讨论记录。

本文保留仍然有效的设计，删除概念性重复，并以最近一次讨论确定的架构为准。实现与本文冲突时，应先更新本文或补充 ADR，再修改代码。

## 1. 产品定位

ADLabel 不是单一标注工具，而是建立在数据湖之上的自动驾驶数据生产平台：

```text
ADLabel
  = 数据湖
  + 数据库索引层
  + 在线业务服务
  + 任务与算法执行系统
  + 地图浏览和人工标注前端
  + 标注/地图/数据集版本管理
```

平台围绕五类稳定对象组织：

| 对象 | 含义 |
|---|---|
| Data Asset | 数据湖中的原始或派生文件 |
| Index Record | 数据库里的可查询索引和关系 |
| Job | 一次可追踪、可重试的数据生产过程 |
| Manifest | Job 的冻结输入快照 |
| Version | 可被发布或训练引用的不可变版本 |

所有业务尽量归一为：

```text
查询数据
  -> 冻结输入 manifest
  -> 算法执行或人工标注
  -> 生成 artifact
  -> 回写索引和质量指标
  -> 形成 annotation/map/dataset version
```

## 2. 已确定的设计原则

1. 原始数据不可变，是 source of truth。
2. 大文件放数据湖；数据库保存 URI、索引、关系、状态和版本。
3. 派生数据独立保存，不覆盖原始数据或旧版本。
4. PostgreSQL/PostGIS 是在线业务和空间数据的主数据库。
5. Frame 是算法访问原始数据的统一契约，但数据库才是平台索引的主真相来源。
6. 每个 Job 必须读取冻结 manifest，不能在执行过程中依赖变化中的在线查询结果。
7. 每个产物必须记录输入、配置、代码/模型版本、生产 Job 和校验和。
8. 世界级浏览使用 WGS84；高精编辑和算法计算使用 UTM/ENU 米制坐标。
9. 人工 2D 几何是权威标注；Height Map 生成的 3D 几何是可重算派生数据。
10. 多人协作依赖任务租约、revision、Changeset 和数据库事务。
11. 在线业务优先 FastAPI；已有 C++ 算法通过 Worker 或 gRPC 集成。
12. 前端、数据库和 C++ 算法通过稳定交换协议协作，不共享内部类。
13. 自动后处理先于人工审核；人工结果再次后处理必须以 candidate diff 方式返回。
14. 训练 GT 必须绑定 Annotation、Map、Height、Pose、Calibration 和算法版本。
15. 第一阶段少引入基础设施；新数据库和分布式引擎必须由真实规模与查询需求驱动。

## 3. 总体架构

```text
                    React + deck.gl
       数据浏览 / 地图图层 / 几何编辑 / 审核 / Diff
                           |
                        HTTP/JSON
                           v
                    FastAPI Online API
      用户权限 / 查询 / 任务 / 标注 / Changeset / Version
                  |                         |
                  v                         v
        PostgreSQL + PostGIS          Job Queue / Scheduler
      索引 / 空间几何 / 事务                |
                  |                         v
                  |                  C++ / Python Workers
                  |             LIO / BEV / 后处理 / 投影
                  |                         |
                  +------------+------------+
                               v
                    Data Lake / S3 / NAS
        raw data / MCAP / image / lidar / BEV / GT / reports
                               |
                               v
                  Parquet / GeoParquet Export
                     DuckDB / Training Repo
```

平台分成六个职责清晰的平面：

```text
Storage Plane     数据湖和对象存储
Index Plane       PostgreSQL/PostGIS
Compute Plane     C++/Python Worker
Control Plane     Job、Manifest、Artifact、Version
Service Plane     FastAPI在线业务服务
Experience Plane  React + deck.gl前端
```

## 4. 数据存储分工

### 4.1 数据湖

数据湖保存体积大、版本多、主要按批次读取的文件：

```text
rosbag / MCAP / xray
LiDAR point cloud
camera image
BEV intensity / RGB
Height Map / GeoTIFF / COG
localization map tile
model prediction / checkpoint
annotation export / training GT
job manifest / config / report / log
```

对象存储中的文件应尽量不可变。更新通过创建新 URI 和新版本实现，不覆盖旧产物。

推荐派生数据路径：

```text
derived/{job_type}/{job_id}/{partition_key}/{artifact_name}
```

例如：

```text
derived/lidar_topdown/job_001/9q8yyk/bev_intensity.tif
derived/lidar_topdown/job_001/9q8yyk/height_map.tif
derived/bev_postprocess/job_002/result.pb
derived/projection/job_003/camera_gt.parquet
```

### 4.2 PostgreSQL/PostGIS

PostgreSQL/PostGIS负责：

```text
在线查询和事务
用户、权限和任务状态
Trip/Frame/Asset索引
Pose和Calibration版本
轨迹与bbox空间查询
正式标注几何和拓扑
Changeset、审核和版本
产物注册、引用和删除状态
```

不建议把点云、图片、BEV、Height Map或大规模模型输出直接放入数据库。

### 4.3 Parquet、GeoParquet和DuckDB

```text
Parquet
  大规模结构化metadata、训练manifest和批量结果

GeoParquet
  离线地图几何候选、地图diff、QA和交换

DuckDB
  本地查询Parquet/GeoParquet、统计和训练样本筛选
```

PostGIS与GeoParquet不是替代关系：

| PostGIS | GeoParquet |
|---|---|
| 在线服务、事务、并发编辑 | 离线批量、交换、分析 |
| 正式标注和地图版本 | 候选结果和导出快照 |
| 空间索引和局部查询 | 大规模顺序扫描 |

### 4.4 可选的后期系统

第一阶段不引入以下系统：

```text
LanceDB / Milvus  embedding和hard case检索
ClickHouse        大规模任务指标和日志分析
Iceberg / Delta   超大规模可变数据湖表
```

只有 PostgreSQL/PostGIS、Parquet/DuckDB 无法满足明确需求时才增加。

## 5. 核心数据模型

### 5.1 Dataset、Trip、Frame和Asset

```text
Dataset
  -> Trip / Scene
      -> Frame
          -> FrameAsset
```

Frame 表示某个统一时间点的数据索引；FrameAsset 表示具体传感器数据。

对 nuScenes：

```text
sample      -> Frame
sample_data -> FrameAsset
```

对 MCAP/rosbag：

```text
同步时间点的一组消息 -> Frame
topic/message         -> FrameAsset
```

建议核心字段：

```text
datasets
  dataset_id, name, source_type, status, metadata

trips
  trip_id, dataset_id, source_uri, start_time, end_time, bbox, status

frames
  frame_id, trip_id, timestamp, prev_id, next_id

frame_assets
  asset_id, frame_id, sensor_id, asset_type, uri, encoding, checksum
```

### 5.2 Pose和Calibration

Pose与Calibration必须独立版本化，不能只在Frame中保存一份可变字段：

```text
frame_poses
  frame_id
  pose_type: raw / lio_relative / gnss_fused / optimized_global
  pose_version
  job_id
  translation / rotation
  quality

calibrations
  calibration_id
  vehicle / sensor
  valid_time_range
  intrinsic / distortion / extrinsic
  version
```

任务 manifest 可以内嵌冻结后的 Pose/Calibration，算法不应在运行中反复查询数据库。

### 5.3 Asset、Artifact和Version

```text
data_assets
  平台已知的原始或派生数据资产

artifacts
  某个Job实际生成的产物及其provenance

versions
  对一组稳定产物和索引的不可变引用
```

Artifact至少记录：

```text
artifact_id
job_id
artifact_type
uri
checksum
bbox / time_range
schema_version
producer_version
created_at
```

更新和删除采用版本化与逻辑删除：

```text
active=false / deleted_at
  -> 引用检查
  -> 后台Garbage Collector物理删除
  -> physical_deleted_at
```

## 6. Frame与原始数据访问

Frame是算法模块之间访问原始数据的统一契约，不是全平台唯一存储格式。

最新边界：

```text
数据库表
  在线主索引和真相来源

FrameProto / frames.pb / frames.parquet
  Job输入快照或算法内部交换格式
```

Frame不承载点云或图片本体，只保存标识、时间和数据引用：

```protobuf
message FrameRef {
  string frame_id = 1;
  string trip_id = 2;
  int64 timestamp_ns = 3;
  repeated AssetRef assets = 4;
  PoseRef pose = 5;
  CalibrationRef calibration = 6;
}
```

原始数据访问层将异构来源隐藏在统一接口后：

```text
DataSource
  MCAP / file directory / KITTI / nuScenes importer

RawDataReader
  FileRawDataReader
  McapRawDataReader
  DispatchRawDataReader
```

算法不应出现针对 rosbag、MCAP、KITTI 的业务分支。MCAP reader应缓存文件句柄，避免每帧重复解析索引。

## 7. Job、Manifest和Artifact

标准任务流程：

```text
1. 创建Job
2. 根据trip/bbox/geohash/time/version查询数据库
3. 冻结输入manifest和config
4. 生成可重试partition
5. Worker读取manifest和数据湖文件
6. 输出artifact和quality report
7. Registrar统一回写数据库
8. 创建或更新候选Version
```

建议表：

```text
jobs
  job_id, job_type, status
  input_query, input_manifest_uri, config_uri
  code_version, created_by
  output_prefix, started_at, finished_at, error

job_partitions
  partition_id, job_id, partition_key
  status, attempt, input_manifest_uri
  output_uri, quality, error

artifacts
  artifact_id, job_id, partition_id
  artifact_type, uri, checksum, bbox, metadata
```

Manifest必须包含足够信息，使Worker无需频繁在线查数据库：

```text
frame_id / trip_id / timestamp
asset URI和checksum
Pose及其版本
Calibration及其版本
算法输入版本
目标CRS / region / bbox
```

## 8. 计算与调度模型

算法应首先实现成可独立测试的C++/Python库，并提供稳定CLI入口：

```text
library
  核心算法和内部数据结构

CLI
  读取manifest/protobuf/config
  调用library
  写出result/report

Worker
  负责租约、重试、日志、资源和Registrar
```

规模演进：

```text
Stage 1  单进程CLI，单trip或单partition
Stage 2  多进程/Kubernetes Job，partition级并行
Stage 3  根据负载选择Ray、Spark或专用调度器
```

不提前绑定唯一分布式引擎：

```text
Ray    适合Python/ML/GPU和trip级任务
Spark  适合大规模ETL、shuffle和瓦片批处理
K8s Job适合隔离良好的CLI批任务
```

数据库不是shuffle系统。大规模group-by由执行引擎完成，数据库只负责规划输入和登记结果。

## 9. 首批数据任务

### 9.1 Trip定位

```text
Trip frames
  -> LIO相对轨迹
  -> GNSS/IMU融合
  -> global/optimized pose
  -> frame_poses + trajectories
```

轨迹使用PostGIS `LINESTRINGZ`建立空间索引；完整报告和轨迹protobuf放数据湖。

### 9.2 lidar_topdown

```text
按bbox/geohash查询frames
  -> 冻结manifest
  -> 读取LiDAR/Pose/Calibration
  -> 拼接点云
  -> BEV intensity / RGB / Height Map
  -> 注册bev_products、height_maps、frame links
```

按geohash执行时必须考虑点云覆盖范围和tile overlap，不能只根据ego pose所在geohash分桶。

建议表：

```text
bev_products
  bev_id, job_id, product_type, uri
  width, height, resolution
  origin, bbox, crs, geohash_list

height_maps
  height_map_id, bev_id, uri
  resolution, bbox, crs, nodata, vertical_datum

bev_frame_links
  bev_id, frame_id
```

### 9.3 自动标注与后处理

```text
BEV模型输出
  -> C++ lane graph后处理
  -> 自动候选Annotation Version
  -> 人工编辑和审核
```

### 9.4 地图更新

```text
Trip LIO
  -> base map matching / loop closure
  -> pose graph optimization
  -> map patch
  -> localization map version
```

需要保存 `map_update_jobs`、`pose_edges`、`map_matching_pairs`、`map_patches` 和输入base map版本。

### 9.5 训练导出

```text
Dataset Version
  -> 冻结frames/assets/poses/calibrations/annotations
  -> Parquet/GeoParquet/Protobuf manifest
  -> training repository
```

训练仓库不直接读取在线数据库的“最新状态”。

## 10. 地图浏览

ADLabel地图浏览器需要显示：

```text
OSM世界底图
所有Trip的空间分布和轨迹
Localization Map
BEV Intensity
Height Map调试层
语义地图元素和标注任务
```

推荐前端：

```text
React + TypeScript
deck.gl
TanStack Query
ADLabel现有UI组件库
```

deck.gl负责大规模地理数据的GPU渲染：

| 数据 | 图层 |
|---|---|
| OSM/BEV/Localization瓦片 | TileLayer + BitmapLayer |
| Trip和车道线 | PathLayer |
| Polygon标注 | PolygonLayer / GeoJsonLayer |
| 顶点和车辆 | ScatterplotLayer |
| 点云 | PointCloudLayer |

MapLibre适合普通地图应用，但ADLabel还需要轨迹、点云、编辑图层和后续3D能力。当前最终选择是deck.gl，不再以早期文档中的MapLibre作为默认方案。

### 10.1 分级加载

| Zoom | 显示内容 |
|---|---|
| 0-5 | Trip聚合点或热力图 |
| 6-11 | 城市级严重抽稀轨迹 |
| 12-15 | 当前bbox内简化轨迹 |
| 16-21 | 选中Trip、完整轨迹和高精地图层 |

```http
GET /api/trajectories?bbox=...&zoom=...
GET /api/trips/{trip_id}/trajectory?resolution=full
```

世界视图不能一次加载全部原始轨迹点或BEV。

### 10.2 图层顺序

```text
1. OSM base layer
2. Localization raster overlay
3. BEV intensity raster overlay
4. Trip trajectories
5. Saved semantic elements
6. Current editable geometry
7. Vertices / handles / selection / warnings
```

BEV是独立overlay，应支持显示、透明度、亮度、对比度和左右对比。

## 11. BEV和Web瓦片

xMap中相关图层：

```text
Lossless Map
  全局融合后的定位地图俯视强度图

Layered Map / annotation
  某个SMT轨迹组的BEV intensity图
```

生产链路：

```text
点云/建图结果
  -> intensity image
  -> geohash地理配准GeoTIFF
  -> Web Mercator
  -> XYZ PNG瓦片
  -> OSS/S3/CDN
  -> deck.gl TileLayer + BitmapLayer
```

`offboard/map/localization_map/web_tile_generation`使用Scala、Spark和GeoTrellis，将已经地理配准的GeoTIFF转换为zoom 8-21、256x256的XYZ瓦片。它支持lossless、annotation和16位Height Map。

该模块不负责根据车辆Pose确定BEV在世界中的位置；地理配准必须在切瓦片前完成。

ADLabel推荐输出：

```text
tiles/{map_version}/lossless/{z}/{x}/{y}.png
tiles/{map_version}/bev_intensity/{z}/{x}/{y}.png
tiles/{map_version}/height_map/{z}/{x}/{y}.png
```

## 12. 地图人工标注

目标元素：

```text
lane boundary
lane centerline
road edge
stop line
crosswalk
sidewalk
junction
traffic light / sign / pole
```

几何类型：

| 元素 | 几何 |
|---|---|
| boundary/centerline/road edge/stop line | LineStringZ |
| crosswalk/sidewalk/junction | PolygonZ |
| traffic light/sign/pole/node | PointZ |

地图实体不是单纯几何：

```text
Entity = geometry + properties + ID + relations + version
```

### 12.1 前端交互

必须支持：

- 绘制LineString和Polygon。
- 添加、插入、拖动和删除顶点。
- 顶点与线段吸附。
- 分割、合并、延长和共享节点。
- 选择、高亮、属性编辑和关系编辑。
- 撤销/重做、自动保存和校验问题定位。
- Changeset、提交审核和版本发布。

第一版只做：

```text
lane_boundary
stop_line
crosswalk
```

这三类覆盖长线、短线和Polygon的主要交互。

### 12.2 几何与拓扑

几何描述位置；拓扑描述连接和归属：

```text
Lane A successor -> Lane B
Lane -> left/right boundary
Stop line -> controlled lanes
Crosswalk -> junction
Traffic light -> lane/junction
```

几何校验：

```text
点数、闭合、自相交、短线段、坐标范围、高度跳变
```

拓扑校验：

```text
前后继双向一致
共享节点连接
左右边界方向正确
引用实体存在
Stop line关联正确Lane
Junction入口/出口连通
桥上桥下不错误连接
```

### 12.3 在线后端分层

```text
API Route / Controller
  HTTP、认证、请求和响应

Pydantic Schema
  外部数据契约

SQLAlchemy Model
  数据库映射

Repository
  CRUD和空间查询

Service
  Changeset、版本、审核和业务事务

Geometry / Topology Validator
  业务校验
```

不要把业务流程塞进Controller或ORM Model。

## 13. 坐标系统与Height Map赋高

```text
世界浏览       WGS84 / EPSG:4326
高精地图编辑   UTM或区域ENU，单位米
Height Map     与建图相同的米制CRS
3D标注         map frame中的(x,y,z)
相机GT         image pixel frame
```

每个区域必须记录：

```text
map_frame
EPSG或ENU origin
horizontal datum
vertical datum
axis convention
height_map_version
```

推荐保存：

```text
geometry_2d  人工权威数据
geometry_3d  Height Map/点云/人工高度生成的派生数据
```

赋高流程：

```text
2D LineString/Polygon
  -> 每0.2-0.5m densify
  -> GeoTIFF affine transform到(row,col)
  -> 双线性采样
  -> NoData和异常跳变检查
  -> LineStringZ/PolygonZ
```

记录：

```text
height_map_version
sampling_method
densify_interval_m
z_source: height_map / point_cloud / manual
z_offset
invalid_vertex_count
algorithm_version
```

桥梁、隧道和多层道路不能只依赖单层Height Map，必须允许人工或点云来源覆盖。

## 14. 标注数据库与版本

建议表：

```text
annotation_versions
  version_id, parent_version_id, region_id
  base_map_version, height_map_version
  status, created_by, created_at

annotation_elements
  element_id, version_id, element_type
  geometry_2d, geometry_3d, properties
  revision, z_status, source
  created_by, updated_by, timestamps

annotation_relations
  relation_id, version_id, relation_type
  source_element_id, target_element_id, properties

annotation_changesets
  changeset_id, version_id, user_id
  status, description, timestamps

annotation_operations
  operation_id, changeset_id, operation_type
  element_id, before_value, after_value, sequence
```

正式几何存PostGIS；批量候选和发布快照可以导出GeoParquet。

## 15. 多人协作

第一版采用：

```text
任务区域租约 + 元素revision乐观锁
```

任务租约必须有过期时间，避免用户异常退出后永久占用。

元素更新：

```sql
UPDATE annotation_elements
SET geometry_2d = :geometry,
    revision = revision + 1,
    updated_by = :user_id
WHERE element_id = :element_id
  AND revision = :expected_revision;
```

更新0行返回`409 Conflict`，不能静默覆盖。

第一版不需要多人光标和实时共同编辑。自动保存、租约、revision冲突和定时刷新足以支撑可靠生产；有明确需求后再加入WebSocket presence。

## 16. C++ BEV自动标注后处理

现有模块：

```text
offboard/map/auto_annotation/bev
```

内部链路：

```text
BevInferenceFrame protobuf
  -> BevFrame
  -> LaneGraphGenerator
  -> LaneGraph matcher / merger / optimizer
  -> global LaneGraph
```

`BevInferenceFrame`包含单帧BEV元素、score、label、拓扑矩阵和Pose。`BevFrame`负责置信度、类型和方向过滤；`LaneGraphGenerator`负责：

```text
BEV局部2D -> Pose -> 地图3D
Polyline采样
中心线和边界节点
前驱后继
中心线左右边界
虚拟Lane过滤
```

主流程：

```text
模型输出
  -> C++后处理
  -> 自动候选Annotation Version
  -> 人工编辑
  -> 审核和发布
```

人工修改结果不应默认再次通过自动后处理。局部拓扑重算必须产生candidate version和diff，由用户确认合并。

## 17. FastAPI与C++集成

是否使用Drogon不决定是否需要数据转换。浏览器数据与C++内部`LaneGraph`不同，任何方案都需要适配：

```text
Drogon  JSON -> C++交换模型 -> LaneGraph
FastAPI JSON/Pydantic -> Protobuf -> C++交换模型 -> LaneGraph
```

定义稳定的ADLabel Annotation Proto：

```text
AnnotationElement
TopologyRelation
PostProcessRequest
PostProcessResult
ValidationIssue
```

C++提供双向Adapter：

```text
Annotation Proto -> LaneGraph
LaneGraph -> Annotation Proto
```

推荐集成顺序：

1. C++ CLI +异步Worker：第一版首选，隔离好、易复现、容易复用Bazel代码。
2. gRPC C++ Service：局部后处理确实需要低延迟交互时采用。
3. pybind11：不优先用于生产在线服务，避免C++崩溃拖垮FastAPI。

在线流程：

```text
POST /annotation-versions/{id}/postprocess
  -> FastAPI创建Job并冻结输入
  -> Worker写request.pb
  -> C++ CLI写result.pb
  -> Registrar创建candidate version和diff
  -> 前端预览并确认
```

## 18. Drogon与xMap的借鉴边界

Drogon是C++ Web框架。xMap使用它把HTTP Controller、C++地图Model/Mapper和PostgreSQL连接在同一进程中：

```text
React/deck.gl
  -> Drogon Controller
  -> xMap C++ Model / Mapper / Validator
  -> PostgreSQL/PostGIS
```

xMap的典型分层：

```text
Controller  HTTP入口
Model       Lane、Boundary、StopLine、Junction等业务对象
Mapper      Model与数据库之间的CRUD和空间查询
```

Drogon适合大量直接复用xMap C++在线业务代码的场景。ADLabel是更通用的数据生产平台，主在线服务仍推荐FastAPI；C++通过Worker/gRPC接入。

## 19. 3D标注投影为相机GT

发布后的3D地图元素结合Pose和Calibration投影到相机：

```text
P_map
  -> T_ego_map
P_ego
  -> T_camera_ego
P_camera
  -> camera model + distortion
pixel
```

输入必须冻结：

```text
annotation_version
map_version
height_map_version
pose_version
calibration_version
projection_algorithm_version
```

投影前对曲线densify，投影后处理：

```text
Z_camera > 0
图像边界裁剪
近远距离裁剪
相机平面穿越裁剪
相机畸变
可见性和遮挡
```

遮挡应结合LiDAR深度或渲染深度图。否则被建筑、车辆或道路结构遮挡的元素会错误成为可见GT。

导出：

```text
dataset_version/
  manifest.parquet
  annotations_3d.geoparquet
  camera_labels/
```

每个GT必须可追溯到原始地图元素和所有输入版本。

## 20. API边界

建议核心API：

```http
GET  /api/trips
GET  /api/trajectories
GET  /api/trips/{trip_id}/trajectory
GET  /api/map-layers/{map_version}

GET  /api/annotations?bbox=...&version_id=...
POST /api/changesets
POST /api/changesets/{id}/operations
POST /api/changesets/{id}/validate
POST /api/changesets/{id}/submit

POST /api/annotation-versions/{id}/postprocess
POST /api/annotation-versions/{id}/enrich-height
POST /api/annotation-versions/{id}/publish

POST /api/projection-jobs
GET  /api/jobs/{job_id}
```

HTTP使用JSON/GeoJSON；大批量和跨语言任务使用Protobuf、Parquet或Manifest URI。

## 21. 推荐代码结构

```text
adlabel/
  api/
    routes/
    schemas/
  db/
    session.py
    migrations/
  models/
  repositories/
  services/
    task_service.py
    annotation_service.py
    version_service.py
    map_service.py
  workers/
  manifests/

web/
  features/map-explorer/
  features/map-annotation/
  map/layers/
  api/
  store/

mapping/
  io/
  data/
  ops/
  tools/
  protos/

contracts/
  annotation.proto
  job_manifest.proto
  artifact.proto
```

实现中保留现有C++ `mapping/` 结构；Python在线服务和前端在功能成熟后逐步加入，不要求一次搭完。

## 22. 推荐实施顺序

1. 完成Trip/Frame/Asset/Pose/Calibration导入和PostGIS索引。
2. 打通一条真实Trip的轨迹、OSM和BEV展示。
3. 接入XYZ瓦片和bbox/zoom分层加载。
4. 实现lane boundary、stop line、crosswalk的2D编辑。
5. 建立Changeset、revision、任务租约和审核版本。
6. 接入Height Map，生成并验证3D几何。
7. 将C++ BEV后处理包装成CLI Worker。
8. 导入自动候选版本并支持人工修订。
9. 实现局部拓扑重算和candidate diff。
10. 选择一个相机打通3D到2D投影。
11. 增加遮挡、批量GT导出和Dataset Version。
12. 再扩展centerline、sidewalk、junction、点云、3D编辑和大规模调度。

## 23. 当前不做或尚未锁定

以下不是第一阶段承诺：

```text
多人实时光标和Google Docs式共同编辑
统一采用Ray或Spark作为唯一执行引擎
Iceberg/Delta Lake
ClickHouse
LanceDB/Milvus
全国级自建OSM瓦片基础设施
浏览器直接处理原始点云或全部BEV帧
```

待真实数据验证：

- MCAP chunk和随机访问性能参数。
- Frame manifest最终采用length-prefixed protobuf还是Parquet。
- Job单帧失败、partition失败和整体失败的准确语义。
- 多层道路的Height Map表达方式。
- 局部后处理是否需要常驻gRPC服务。
- 轨迹和语义元素何时需要MVT而不是GeoJSON。

## 24. 术语速查

| 术语 | 定义 |
|---|---|
| Data Lake | 保存原始和派生大文件的存储层 |
| Asset | 数据湖中一个可引用文件或对象 |
| Artifact | 某次Job产生且带provenance的Asset |
| Manifest | Job的冻结输入快照 |
| Frame | 算法访问某时间点多传感器数据的索引契约 |
| Model | 后端业务数据模型，不是机器学习模型 |
| Mapper/Repository | 数据库访问层 |
| Controller/API Route | HTTP入口层 |
| Geometry | Point/LineString/Polygon及其坐标 |
| Topology | 地图实体之间的连接和归属关系 |
| Changeset | 一组可审核的标注修改 |
| Annotation Version | 一组稳定标注元素和关系的版本 |
| Candidate Version | 自动算法产生、尚未确认合并的候选版本 |
| Online Service | 持续运行并快速响应请求的服务进程 |
| Worker | 执行耗时算法和批任务的进程 |

## 25. 文档维护规则

本文件是当前架构的唯一真相来源。新增设计时：

1. 已确定的全局架构更新本文。
2. 有明显取舍的技术决策写ADR，并从本文链接。
3. 模块级接口细节写模块文档，不复制整套架构。
4. 历史讨论不再另建平行的总文档；有价值的结论直接合并到本文。
5. 任何“最新”“默认”“唯一”等表述都必须与实际阶段和范围一致。

后续 Codex 会话可直接使用：

```text
请先阅读 E:\ADLabel\docs\architecture.md，
然后继续 ADLabel 的数据湖、任务系统、地图标注、C++后处理或训练GT设计。
```
