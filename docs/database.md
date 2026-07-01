# 数据库、数据格式与自动驾驶数据管理总结

本文总结围绕 LanceDB、SQL、Parquet、DuckDB、数据库索引、原始数据存储、更新删除策略等内容的讨论，并结合自动驾驶 BEV 自动标注/高精地图数据产线给出推荐理解。

---

## 1. 数据库的本质

数据库可以理解为：

> 针对某类数据和某类查询任务设计的数据管理系统。

数据库的核心目标不仅是“保存数据”，更重要的是：

- 高效查询
- 高效写入
- 数据一致性
- 并发访问
- 权限控制
- 事务管理
- 索引维护
- 版本管理
- 故障恢复
- 数据组织

从最直观的角度看，数据库的核心价值是：

> 通过合理的数据组织和索引，让查询、写入、更新、删除、并发和恢复都更高效、更可靠。

如果只用文件系统管理数据，比如每个 clip 一个 JSON 文件，那么查询“所有夜晚 + 路口 + pose_quality > 0.9 + 非重复的 clip”时，通常需要遍历所有文件、打开 JSON、解析字段、判断条件，效率很低。

数据库会通过索引和查询执行器加速这些操作。

---

## 2. 查询语言是什么？

查询语言是你和数据系统沟通的“指令语言”。

它用来表达：

- 我要哪些数据？
- 从哪里取？
- 按什么条件筛选？
- 怎么排序？
- 是否聚合统计？
- 是否关联其他表？

数据库系统负责把查询语言翻译成实际执行计划。

### 2.1 SQL

SQL 是最常见的查询语言，常用于关系型数据库。

示例：

```sql
SELECT clip_id, trip_id
FROM clips
WHERE has_intersection = true
  AND pose_quality > 0.9
  AND split = 'train';
```

SQL 是声明式语言。用户只表达“我要什么结果”，而不需要写清楚数据库内部如何扫描、如何使用索引、如何排序。

### 2.2 除了 SQL，还有其他查询语言

不同数据库或数据系统有不同查询方式：

| 系统 | 查询语言/方式 | 适合场景 |
|---|---|---|
| PostgreSQL / MySQL / SQLite / DuckDB | SQL | 结构化表格数据、统计、Join |
| MongoDB | Document Query | JSON 文档、嵌套字段 |
| Elasticsearch / OpenSearch | Query DSL | 全文检索、日志检索 |
| Neo4j | Cypher | 图结构、拓扑关系、路径查询 |
| Prometheus | PromQL | 监控指标、时间序列 |
| LanceDB | SDK API + SQL-like filter + vector search | embedding 相似检索、AI 样本管理 |
| GraphQL Server | GraphQL | 前端 API 查询 |

所以：

```text
数据库系统 ≠ 查询语言
SQL 是查询语言
PostgreSQL / MySQL / DuckDB / LanceDB 是数据库或数据系统
```

---

## 3. SQL 和 LanceDB 的区别

LanceDB 不是 SQL 这种查询语言，而是一个数据库系统。

更准确地说：

```text
SQL：查询语言
LanceDB：面向 AI / embedding / 多模态数据的数据库系统
PostgreSQL / MySQL / SQLite：传统关系型数据库系统
```

LanceDB 支持类似 SQL 的条件过滤，但它的核心优势不只是结构化查询，而是：

- embedding/vector 管理
- 向量相似检索
- 多模态数据管理
- 训练数据索引
- hard case mining
- 模型中间特征管理
- prediction / label / split / version 管理

传统 SQL 更擅长：

```text
找满足明确字段条件的数据
```

例如：

```sql
SELECT *
FROM clips
WHERE has_intersection = true
  AND pose_quality > 0.9;
```

LanceDB 更擅长：

```text
在满足条件的数据中，找和某个样本在 embedding 空间中最相似的数据
```

例如：

```python
table.search(failed_clip_embedding) \
     .where("has_intersection = true AND pose_quality > 0.9") \
     .limit(100)
```

### 3.1 最简单的理解

```text
SQL 数据库：我知道我要找什么字段。
LanceDB：我知道一个样本，想找和它相似的样本。
```

或者：

```text
SQL：找满足条件的数据。
LanceDB：找满足条件 + 语义/视觉/特征相似的数据。
```

---

## 4. LanceDB 是什么？

LanceDB 可以理解成：

> 面向 AI/多模态数据的数据库，尤其擅长管理原始数据索引、metadata、embedding/vector、模型特征和训练样本。

它适合存储和查询：

- image embedding
- BEV embedding
- LiDAR embedding
- text embedding
- 模型中间 feature
- CLIP feature
- perception feature
- prediction feature
- hard case feature
- metadata
- label version
- model version
- train/val/test split

在自动驾驶项目中，它可以作为：

> clip/frame/label/prediction/embedding 的统一索引和检索层。

但它不一定替代 S3/NAS/文件系统。更推荐：

```text
大文件：S3 / NAS / 本地文件系统
索引和特征：LanceDB
```

---

## 5. LanceDB 的查询方式

LanceDB 没有一种单独的“LanceQL”作为唯一查询语言。

它主要使用：

1. Python / TypeScript SDK API
2. SQL / SQL-like filter expression
3. Vector search API
4. Full-text / hybrid search API

典型 Python 查询：

```python
import lancedb

db = lancedb.connect("./lancedb")
table = db.open_table("clips")

results = (
    table.search(query_vector)
    .where("has_intersection = true AND pose_quality > 0.9")
    .limit(100)
    .to_pandas()
)
```

这表示：

```text
向量相似检索 + SQL-like metadata filter
```

---

## 6. LanceDB 是如何实现这些功能的？

LanceDB 的能力不是靠“SQL 语法更高级”，而是靠底层存储格式、索引结构和查询执行器共同实现的。

它底层基于 Lance 格式。

可以理解为：

```text
LanceDB = 数据库 API / 查询层 / 向量搜索层
Lance = 底层文件格式 / 表格式 / 列式存储 / 索引 / 版本管理
```

### 6.1 列式存储

Lance 使用列式存储，类似 Parquet。

列式存储的好处是：如果只查询某几列，不需要读取整行里的所有字段。

例如：

```sql
SELECT clip_id
FROM clips
WHERE pose_quality > 0.9;
```

只需要读取：

```text
clip_id 列
pose_quality 列
```

不需要读取 image、label、embedding、prediction 等字段。

### 6.2 向量索引

LanceDB 支持向量近似最近邻索引，例如：

- IVF
- HNSW
- PQ
- SQ
- IVF-PQ
- HNSW-PQ

这些索引用来加速：

```text
找和 query_embedding 最相似的 topK 样本
```

### 6.3 scalar index + vector search

自动驾驶数据常见查询不是单纯向量检索，而是：

```text
在夜晚 + 路口 + pose质量高 + 非重复样本中，
找和某个失败 case 最相似的 100 个 clip。
```

这需要：

```text
metadata filter + vector search
```

LanceDB 支持这种混合查询。

### 6.4 full-text / hybrid search

LanceDB 还可以支持全文检索和 hybrid search，例如：

```text
文本关键词搜索 + 向量检索 + metadata filter
```

适合搜索 scene description、错误描述、样本说明等。

---

## 7. Lance 格式是什么？

Lance 格式可以理解成：

> 面向 AI/多模态数据的开源列式数据格式 + 表格式。

它类似 Parquet / Iceberg / Delta Lake 这类数据湖格式，但更偏向 AI 场景：

- embedding
- 图片
- 视频
- 文本
- 随机读取
- 向量检索
- 训练数据管理
- 版本管理

可以理解为：

```text
LanceDB 负责“怎么查”
Lance 负责“数据怎么存、怎么组织、怎么版本化”
```

Lance 不只是一个单文件格式，还包括：

- Lance File Format
- Lance Table Format
- Lance Index Format
- Catalog Spec

简单类比：

```text
Parquet：通用大数据列式文件格式，适合分析和批量扫描。
Lance：面向 AI/多模态数据的列式格式，更重视随机访问、embedding、向量索引和训练数据读取。
```

---

## 8. LanceDB 是否是自动驾驶数据集或 AI 数据集的通用标准？

不是。

截至目前，LanceDB / Lance 还不能说是自动驾驶数据集或 AI 数据集的“通用标准”。

更准确的判断是：

> LanceDB / Lance 是一个正在快速发展的 AI/multimodal 数据管理方案，适合 embedding、向量检索、样本管理、hard case mining，但还不是行业通用标准。

自动驾驶数据集目前仍然各有自己的格式和 API：

- nuScenes：JSON schema + devkit
- Waymo Open Dataset：TFRecord / proto schema
- Argoverse 2：自己的 parquet/json/sensor schema
- KITTI：目录 + txt 标定格式
- nuPlan / NAVSIM：自己的场景与仿真接口

所以不要把 LanceDB 当成必须遵循的标准，而应该把它当成可替换的数据索引和检索组件。

真正应该稳定定义的是项目自己的：

- trip_id
- clip_id
- frame_id
- timestamp_ns
- sensor_name
- pose
- calibration
- asset_uri
- label_version
- map_version
- model_version
- coordinate frame
- clip/frame schema

---

## 9. Parquet 是什么？

Parquet 是一种列式文件格式，不是数据库软件。

它类似：

```text
.txt     文本文件格式
.csv     表格文本格式
.json    半结构化数据格式
.parquet 列式表格数据格式
```

Parquet 常用于大数据分析、数据湖和机器学习数据集 metadata 管理。

它适合保存结构化表格数据，例如：

```text
clip_id
trip_id
timestamp_ns
geohash
pose_quality
bev_uri
label_uri
split
```

Parquet 的优势：

- 文件更小
- 读取更快
- 有 schema 和数据类型
- 压缩效果好
- 可以只读需要的列
- 适合 Spark / DuckDB / Pandas / PyArrow / Polars / Hive / Trino 等工具

Parquet 不太适合直接存超大原始数据，例如：

- rosbag
- 大点云 pcd/bin
- 原始图片
- 视频
- 大型 BEV 图
- 模型 checkpoint

更推荐：

```text
大文件放 S3 / NAS / 文件系统
Parquet 存 URI + metadata
```

---

## 10. DuckDB 是什么？

DuckDB 是一个嵌入式分析型数据库。

可以理解成：

> 像 SQLite 一样轻量、本地运行，但更擅长分析 Parquet / CSV / 大表数据的 SQL 数据库。

DuckDB 特别适合：

- 直接查询 Parquet 文件
- 本地分析数据集
- 统计样本分布
- 筛选训练样本
- 生成 train/val/test split
- 检查 label 版本
- 分析模型预测结果

示例：

```sql
SELECT clip_id, bev_uri, label_uri
FROM 'clips.parquet'
WHERE has_intersection = true
  AND pose_quality > 0.9
  AND split = 'train';
```

DuckDB 和 SQLite 的类比：

```text
SQLite ≈ 本地小型业务数据库
DuckDB ≈ 本地小型数据仓库 / 分析数据库
```

DuckDB 和 LanceDB 的区别：

| 对比 | DuckDB | LanceDB |
|---|---|---|
| 核心能力 | SQL 分析 | 向量检索 + AI 数据管理 |
| 查询对象 | Parquet/CSV/表格数据 | embedding、多模态数据、metadata |
| 典型问题 | 统计有多少路口 clip | 找和某个失败 case 相似的 clip |
| 是否擅长 vector search | 不是核心能力 | 是核心能力 |

---

## 11. 原始数据、数据库和索引的关系

在自动驾驶数据系统中，通常不是把所有原始数据都放进数据库。

更推荐：

```text
原始大文件保存在对象存储 / NAS / 本地文件系统
数据库或表系统在上层建立索引、metadata、特征和状态表
```

典型结构：

```text
原始数据层：S3 / NAS / 本地文件系统 / HDFS
      ↓
不同索引层：
  - PostgreSQL：任务状态、人工审核、版本管理
  - LanceDB：embedding、相似检索、hard case mining
  - DuckDB/Parquet：离线分析、统计、数据扫描
  - Elasticsearch/OpenSearch：日志、文本、模糊检索
  - Redis：缓存、在线快速查询
```

也就是：

```text
原始数据一份
上层多个索引
每个索引服务一种高频查询任务
```

---

## 12. 应用数据库时，通常需要创建索引吗？

是的。

通常有两层索引：

### 12.1 业务层索引

也就是 Indexer / DataAdapter 做的事情。

它扫描原始数据，生成：

- clip_id
- frame_id
- trip_id
- timestamp_ns
- sensor_name
- image_uri
- lidar_uri
- pose
- calibration version
- geohash
- label_uri
- prediction_uri
- split

这一步解决：

```text
原始数据在哪里？
每一帧是什么时间？
相机和雷达怎么对齐？
每个 clip 包含哪些 frame？
对应的 pose/calib 是哪个版本？
BEV 图和 label 存在哪里？
```

### 12.2 数据库内部索引

数据库在表字段上建立加速结构，例如：

- clip_id index
- timestamp index
- geohash index
- label_version index
- model_version index
- pose_quality index
- split index
- embedding vector index

这些索引用来加速高频查询。

---

## 13. 自动驾驶项目推荐的数据分层

推荐结构：

```text
raw data storage:
  rosbag / xray / nuScenes / Argoverse / lidar / camera / calib

asset storage:
  clips / frames / bev_intensity / height_map / labels / predictions

metadata index:
  clip_id / frame_id / timestamp / sensor / pose / calib / geohash / uri

AI index:
  embedding / feature / hardcase_score / duplicate_score / split

production DB:
  task status / review status / map version / publish status
```

不同系统分工：

| 系统 | 适合管理什么 |
|---|---|
| S3 / NAS / 文件系统 | rosbag、pcd、image、BEV、label、prediction 大文件 |
| Parquet | metadata 表、索引表、训练 manifest |
| DuckDB | 本地 SQL 分析、统计、筛训练集 |
| PostgreSQL | 任务状态、人工审核、生产状态、版本管理 |
| LanceDB | embedding、相似检索、hard case mining、AI 特征索引 |
| Elasticsearch | 日志、错误、文本描述检索 |
| Redis | 在线缓存 |

---

## 14. 更新和删除怎么处理？

更新和删除是数据库工具和数据管理系统必须考虑的核心问题。

但在“原始数据在对象存储/文件系统，数据库只管索引”的架构里，要分两层：

```text
1. 数据库里的记录怎么更新/删除
2. 原始文件本身怎么更新/删除
```

传统数据库天然支持：

```sql
UPDATE clips
SET pose_quality = 0.95
WHERE clip_id = 'clip_001';

DELETE FROM clips
WHERE clip_id = 'clip_001';
```

数据库会维护表记录、索引、事务和一致性。

但如果数据库记录里有：

```text
bev_uri = s3://dataset/clips/clip_001/bev.png
```

删除数据库记录并不一定会自动删除对象存储里的真实文件。

所以数据湖系统中常区分：

```text
逻辑删除：数据库标记这个 clip 不再使用
物理删除：真正删除 S3/NAS 上的文件
```

---

## 15. 为什么推荐逻辑删除？

推荐先做逻辑删除，而不是立即物理删除。

例如：

```text
is_deleted = true
deleted_at = 2026-06-27
delete_reason = bad_pose
```

原因：

- 方便回滚
- 方便审计
- 避免误删
- 保证训练可复现
- 避免其他任务还在引用这个文件
- 避免破坏已有地图版本或模型实验

查询训练集时加：

```sql
SELECT *
FROM clips
WHERE is_deleted = false
  AND split = 'train';
```

---

## 16. 删除原始数据是否应该通过数据库？

是的，最好不要绕过数据库或元数据系统直接删除原始数据。

推荐原则：

> 删除原始数据也应该通过统一的数据管理入口完成，由数据库记录状态，再由后台清理任务删除底层文件。

不推荐：

```bash
rm /data/clips/clip_001/bev.png
aws s3 rm s3://dataset/clips/clip_001/
```

推荐流程：

```text
请求删除 clip_001
      ↓
数据库标记 is_deleted = true
      ↓
检查是否还有任务/模型/地图版本引用它
      ↓
确认可删
      ↓
后台 garbage collector 删除 S3/NAS 文件
      ↓
数据库记录 physical_deleted = true
```

这样可以避免：

```text
数据库里 URI 还存在，但真实文件已经没了
```

---

## 17. 推荐的 asset 表设计

可以设计一张 asset_table：

```text
asset_table
- asset_id
- clip_id
- asset_type       # bev / height_map / label / prediction / lidar / image
- asset_uri
- asset_version
- producer         # indexer / model_v1 / human_edit
- created_at
- is_active
- is_deleted
- physical_deleted
```

示例：

```text
asset_id   clip_id    asset_type   version   is_active   uri
a001       clip_001   label        v1        false       s3://labels/v1/clip_001.json
a002       clip_001   label        v2        true        s3://labels/v2/clip_001.json
```

这样可以支持：

- 多版本 label
- 多版本 prediction
- 逻辑删除
- 物理清理
- 审计追踪
- 训练可复现

---

## 18. 自动驾驶数据系统中，更新更推荐版本化

不要轻易覆盖旧文件。

比如 label 从 v1 修正到 v2：

```text
label_version = v1
label_uri = s3://labels/v1/clip_001.json

label_version = v2
label_uri = s3://labels/v2/clip_001.json
```

模型训练时记录：

```text
model_v3 使用 label_version = v2
```

这样比直接覆盖 `label.json` 更可靠。

---

## 19. 对 BEV 自动标注项目的推荐落地路线

### 第一阶段：基础 metadata 管理

```text
原始数据：S3 / NAS / 本地文件系统
索引表：Parquet
查询工具：DuckDB
```

产物：

```text
clips.parquet
frames.parquet
assets.parquet
poses.parquet
calibrations.parquet
labels.parquet
predictions.parquet
```

用途：

- 统计样本分布
- 筛选训练样本
- 检查 label version
- 生成 train/val/test split
- 检查 pose_quality

### 第二阶段：生产状态管理

引入 PostgreSQL 管理：

- task status
- annotation status
- review status
- map version
- production status
- delete request
- audit log

### 第三阶段：AI 检索和 hard case mining

引入 LanceDB 管理：

- BEV embedding
- image embedding
- LiDAR embedding
- prediction embedding
- hardcase_score
- duplicate_score
- similarity search
- training view

---

## 20. 最终总结

核心理解：

```text
原始数据不要全部塞进数据库。
原始大文件保存在 S3 / NAS / 本地文件系统。
数据库或表系统在上层建立不同用途的索引。
```

不同工具的定位：

```text
Parquet：列式文件格式，适合保存 metadata 表。
DuckDB：本地分析数据库，适合用 SQL 查询 Parquet。
PostgreSQL：生产状态和关系数据管理。
LanceDB：AI 数据、embedding、相似检索、hard case mining。
Lance：LanceDB 底层的 AI 多模态列式格式/表格式。
S3/NAS：保存原始大文件。
```

最推荐的架构：

```text
S3 / NAS / 本地文件系统
  存 raw data / assets

Indexer / DataAdapter
  扫描原始数据，生成 frame/clip/asset metadata

Parquet + DuckDB
  做离线分析和训练集筛选

PostgreSQL
  管任务状态、审核状态、版本和删除流程

LanceDB
  管 embedding、相似样本检索、hard case mining

Garbage Collector
  根据数据库标记安全删除底层文件
```

一句话总结：

> 自动驾驶数据系统最好采用“底层文件存储 + 上层多种数据库索引”的架构。原始数据保存一份，不同数据库针对不同查询任务建立不同索引；更新和删除通过统一元数据系统管理，避免文件和数据库记录不一致。

---

## 21. GeoParquet 是什么？

GeoParquet 可以理解成：

> 在 Parquet 里标准化保存地理空间数据的一套规范。

前面已经说过，Parquet 是一种列式表格文件格式。GeoParquet 则是在 Parquet 基础上增加地理空间约定，用来说明：

- 哪一列是 geometry
- geometry 是 Point / LineString / Polygon 还是 MultiPolygon
- geometry 用什么编码方式存储
- 坐标系 CRS 是什么
- bbox / extent 如何记录
- metadata 如何描述
- 不同 GIS / 数据分析工具如何互相识别

所以 GeoParquet 本质上仍然是 `.parquet` 文件，只是它遵循了一套地理空间 metadata 规范。

可以理解为：

```text
Parquet:
  通用列式表格文件格式

GeoParquet:
  Parquet + 地理空间 geometry 列 + CRS / bbox / metadata 规范
```

---

## 22. 为什么需要 GeoParquet？

普通 Parquet 只知道自己在存表格数据，例如：

```text
id | name | geometry
```

但普通 Parquet 本身不知道 `geometry` 这一列到底是什么：

```text
是 WKT 字符串？
是 WKB 二进制？
是经纬度？
是 UTM 坐标？
是 Point？
是 LineString？
是 Polygon？
坐标系是什么？
```

如果没有统一规范，不同工具可能各自定义 geometry 的存储方式：

```text
工具 A：geometry 存 WKB
工具 B：geometry 存 WKT
工具 C：经纬度拆成 lon / lat 两列
工具 D：坐标系写在单独 json 里
```

这样虽然都能“存空间数据”，但不同工具之间很难互通。

GeoParquet 解决的问题是：

> 让不同 GIS、数据分析、云数仓和空间计算工具都能用同一种方式读写 Parquet 里的空间几何数据。

---

## 23. GeoParquet 适合存什么？

GeoParquet 主要适合存 GIS / 地图里的 vector geometry：

- Point
- LineString
- Polygon
- MultiPoint
- MultiLineString
- MultiPolygon
- GeometryCollection

例如道路中心线：

```text
road_id | road_type | speed_limit | geometry
r001    | highway   | 80          | LINESTRING(...)
r002    | city_road | 50          | LINESTRING(...)
```

例如建筑物面：

```text
building_id | height | geometry
b001        | 30.5   | POLYGON(...)
b002        | 18.0   | POLYGON(...)
```

例如 POI 点：

```text
poi_id | name        | category | geometry
p001   | gas station | fuel     | POINT(...)
```

---

## 24. GeoParquet 和 GeoJSON / Shapefile 的区别

| 格式 | 特点 | 适合场景 |
|---|---|---|
| Shapefile | 老牌 GIS 格式，生态广，但字段、编码和文件数量限制较多 | 传统 GIS 交换 |
| GeoJSON | 文本 JSON，人可读，Web 友好 | 小规模数据、接口传输、调试 |
| Parquet | 高效列式表格，不专门表达空间语义 | 大规模表格分析 |
| GeoParquet | Parquet + 空间几何标准 | 大规模地理空间数据湖、云分析、批处理 |

简单说：

```text
GeoJSON 更适合小数据和调试。
GeoParquet 更适合大规模空间数据分析和数据湖。
```

如果只有几十条车道线，GeoJSON 很方便。

如果是：

- 全国道路
- 全球建筑物
- 上亿 POI
- 大规模地图要素
- 自动驾驶地图切片索引
- 自动标注候选几何结果

GeoParquet 更适合。

---

## 25. GeoParquet 对自动驾驶 / 高精地图项目的意义

GeoParquet 和 BEV 自动标注、高精地图产线比较相关，因为自动驾驶地图里很多语义元素本质上都是几何对象：

```text
lane boundary     → LineString / MultiLineString
lane centerline   → LineString / MultiLineString
road edge         → LineString
stop line         → LineString
crosswalk         → Polygon
traffic light     → Point
traffic sign      → Point
lane area         → Polygon
road area         → Polygon
```

因此，GeoParquet 很适合作为：

- 自动标注候选结果
- 后处理 polyline / polygon 结果
- 地图语义元素离线中间格式
- 地图要素分析格式
- 地图 QA / diff / 统计输入格式
- QGIS / GeoPandas / DuckDB spatial 可读取的数据交换格式

例如可以设计：

```text
lane_boundaries.parquet
- lane_boundary_id
- clip_id
- geohash
- semantic_type      # solid / dashed / road_edge
- confidence
- source             # model / rule / human
- label_version
- map_version
- geometry           # LineString
```

```text
crosswalks.parquet
- crosswalk_id
- clip_id
- geohash
- confidence
- source
- label_version
- geometry           # Polygon
```

```text
traffic_lights.parquet
- traffic_light_id
- clip_id
- geohash
- confidence
- source
- geometry           # Point
```

---

## 26. 使用 GeoParquet 时要特别注意坐标系

GeoParquet 很重视 CRS，也就是 Coordinate Reference System，坐标参考系。

自动驾驶数据里可能同时存在多种坐标：

```text
WGS84 经纬度
UTM
局部 ENU
map local frame
vehicle frame
LiDAR sensor frame
camera image pixel frame
BEV pixel frame
```

GeoParquet 更适合存已经恢复到地图坐标系或地理坐标系的几何结果，例如：

```text
适合 GeoParquet：
- UTM 下的 lane boundary
- map frame 下的 road edge
- WGS84 下的 POI / traffic light
- map local frame 下的自动标注 polyline，但必须清楚记录 CRS / 坐标定义
```

不太适合直接作为 GeoParquet 的数据：

```text
- BEV image 像素坐标
- camera image 像素坐标
- LiDAR sensor frame 原始点云
- rosbag 原始数据
```

如果必须存局部 ENU 或自定义 map frame，也要在 metadata 里明确说明原点、轴方向、单位、版本，否则很容易出现跨数据错位。

---

## 27. GeoParquet 在推荐架构中的位置

加入 GeoParquet 后，自动驾驶数据系统可以这样分工：

| 系统/格式 | 适合管理什么 |
|---|---|
| S3 / NAS / 文件系统 | rosbag、pcd、image、BEV、height map、label、prediction 大文件 |
| Parquet | clip/frame/asset metadata 表、训练 manifest |
| GeoParquet | lane boundary、centerline、road edge、crosswalk、traffic light 等地图几何结果 |
| DuckDB | 查询 Parquet / GeoParquet，做离线统计和训练集筛选 |
| DuckDB spatial / GeoPandas / QGIS | 读取和分析 GeoParquet 空间几何 |
| PostgreSQL | 任务状态、人工审核、生产状态、版本管理 |
| LanceDB | embedding、相似检索、hard case mining、AI 特征索引 |

推荐流程：

```text
BEV 模型输出
      ↓
后处理生成 polyline / polygon / point
      ↓
恢复到 map / UTM / WGS84 坐标系
      ↓
保存为 GeoParquet
      ↓
QGIS / GeoPandas / DuckDB spatial / 地图编辑器读取
      ↓
人工审核 / QA / 地图入库
```

也就是说：

```text
BEV 图像、height map、原始点云：仍然放对象存储或文件系统
clip/frame 索引：用 Parquet / DuckDB
地图几何结果：用 GeoParquet
embedding 和 hard case：用 LanceDB
生产状态和版本：用 PostgreSQL
```

---

## 28. 更新后的最终理解

在前面的总结基础上，可以补充一句：

> Parquet 适合保存普通表格 metadata；GeoParquet 适合保存带地图几何对象的表格数据；LanceDB/Lance 适合保存和检索 AI embedding、多模态特征与 hard case；PostgreSQL 适合管理生产状态和版本；S3/NAS 适合保存原始大文件。

对于 BEV 自动标注 / 高精地图项目，更完整的推荐架构是：

```text
S3 / NAS / 本地文件系统
  存 raw data / assets

Indexer / DataAdapter
  扫描原始数据，生成 frame/clip/asset metadata

Parquet + DuckDB
  管 metadata，做离线分析和训练集筛选

GeoParquet + GeoPandas / QGIS / DuckDB spatial
  管 lane boundary / centerline / crosswalk / road edge 等地图几何结果

PostgreSQL
  管任务状态、审核状态、版本和删除流程

LanceDB
  管 embedding、相似样本检索、hard case mining

Garbage Collector
  根据数据库标记安全删除底层文件
```

一句话总结：

> GeoParquet 不是数据库软件，而是 Parquet 的地理空间扩展规范。它非常适合保存自动驾驶地图中的车道线、道路边界、人行横道、交通灯等几何语义结果；原始大文件仍然放对象存储/文件系统，GeoParquet 负责让地图几何结果标准化、可查询、可交换。

---

## 29. 地图数据为什么还需要 PostGIS？

PostGIS 不是和 GeoParquet / Parquet / LanceDB 互相替代的关系，而是处在不同层次。

PostGIS 是 PostgreSQL 的空间扩展，适合做正式地图数据库、地图编辑器后端、空间查询服务和人工审核入库。它支持 Point、LineString、Polygon 等几何对象，也支持空间索引和空间函数，例如范围查询、距离查询、相交判断、缓冲区查询等。

可以理解为：

```text
PostGIS = 正式、可编辑、可查询、支持事务的空间数据库
GeoParquet = 离线、批量、文件化、适合数据湖的空间数据格式
```

---

## 30. PostGIS 适合管理什么？

PostGIS 适合这些场景：

```text
正式语义地图数据库
地图编辑器后端
人工审核 / 修改 / 入库
按空间范围查询地图元素
车道线、道路边界、停止线、人行横道、交通灯等要素管理
地图版本、changeset、权限、事务、并发编辑
空间拓扑检查
对外提供地图查询服务
```

例如正式地图要素可以放在 PostGIS 表中：

```text
lane_boundary
- id
- map_version
- type
- geometry      # LineString
- created_at
- updated_at

crosswalk
- id
- map_version
- geometry      # Polygon

traffic_light
- id
- map_version
- geometry      # Point
```

PostGIS 可以很自然地做空间查询：

```sql
SELECT *
FROM lane_boundary
WHERE ST_Intersects(
  geometry,
  ST_MakeEnvelope(xmin, ymin, xmax, ymax, srid)
);
```

或者查某个位置附近的交通灯：

```sql
SELECT *
FROM traffic_light
WHERE ST_DWithin(geometry, ego_position, 100.0);
```

---

## 31. 为什么不是所有地图数据都放 PostGIS？

不是 PostGIS 不行，而是不同数据的职责不同。

自动驾驶地图产线里除了正式地图，还有大量原始数据、离线中间结果、自动标注候选、训练样本、模型预测、embedding 和临时版本。

这些数据通常有几个特点：

```text
数据量大
版本多
批处理多
读多写少
经常按批次重算
很多结果只是中间产物
需要和 S3/NAS 上的大文件关联
需要训练集筛选、统计、hard case mining
```

如果全部放进 PostGIS，系统会变重，而且 PostGIS 会逐渐变成“实验中间结果堆场”。

更合理的是：

```text
中间结果 / 自动标注候选：GeoParquet / Parquet
正式地图结果：PostGIS
embedding / hard case：LanceDB
任务和版本状态：PostgreSQL
原始大文件：S3 / NAS / 本地文件系统
```

---

## 32. PostGIS 和 GeoParquet 的分工

| 对比 | PostGIS | GeoParquet |
|---|---|---|
| 本质 | 空间数据库 | 空间数据文件格式 |
| 运行方式 | PostgreSQL 服务 | 普通 `.parquet` 文件 |
| 强项 | 查询、事务、并发、编辑、空间索引 | 批量存储、分析、交换、数据湖 |
| 适合 | 正式地图库、地图编辑器、地图服务 | 离线产物、中间结果、训练数据、批处理 |
| 更新删除 | 强 | 弱，通常写新版本 |
| 多用户编辑 | 强 | 不适合 |
| 大规模离线扫描 | 可以，但不是最轻 | 很适合 |
| 云对象存储 | 可接入，但不是最自然 | 很自然 |

所以二者不是替代关系，而是上下游关系：

```text
自动标注模型输出
      ↓
几何后处理 / 拓扑推断
      ↓
候选地图要素 GeoParquet
      ↓
离线质检 / 人工审核
      ↓
通过审核的结果写入 PostGIS
      ↓
发布为正式语义地图 / routing graph / tile / map package
```

---

## 33. PostGIS 不适合直接管理哪些东西？

这些大文件和 AI 中间产物不建议直接放 PostGIS：

```text
rosbag / xray
点云 pcd / bin
相机图片
BEV intensity image
height map raster
模型 checkpoint
prediction pkl
大规模 embedding
临时 debug 可视化结果
```

更推荐做法是：

```text
S3 / NAS / 本地文件系统：保存大文件
PostGIS / PostgreSQL / Parquet / LanceDB：保存 URI、metadata、索引、版本和状态
```

例如：

```text
S3:
  s3://dataset/clips/clip_001/bev.png
  s3://dataset/clips/clip_001/height_map.tif
  s3://dataset/clips/clip_001/label.json

PostGIS / metadata table:
  clip_id = clip_001
  bev_uri = s3://dataset/clips/clip_001/bev.png
  label_uri = s3://dataset/clips/clip_001/label.json
  geometry = LINESTRING(...)
  map_version = v3
```

---

## 34. 对 BEV 自动标注 / 高精地图项目的推荐分层

更完整的地图数据管理架构可以是：

```text
S3 / NAS / 本地文件系统
  存 rosbag、xray、点云、图片、BEV 图、height map、label、prediction 等大文件

Parquet
  存 clip/frame/asset metadata、训练集索引、普通结构化字段

DuckDB / Spark
  查 Parquet，做离线统计、训练集筛选、大规模批处理分析

GeoParquet
  存 lane boundary、centerline、road edge、stop line、crosswalk、traffic light 等空间几何候选结果

PostgreSQL
  管任务状态、审核状态、生产状态、版本、删除流程、权限

PostGIS
  管正式语义地图、人工审核后的地图要素、地图编辑器后端、空间查询服务

LanceDB
  管 embedding、相似样本检索、hard case mining、AI 数据检索
```

一句话：

> 应该用 PostGIS，但不要只用 PostGIS。PostGIS 适合正式地图库和地图服务；GeoParquet 适合离线候选结果和批量交换；Parquet/DuckDB 适合 metadata 分析；LanceDB 适合 embedding 和 hard case；S3/NAS 适合保存原始大文件。

---

## 35. 最终推荐架构图

```text
原始采集数据 / 数据集
rosbag / xray / lidar / camera / calib / pose
        |
        v
S3 / NAS / 本地文件系统
        |
        v
Indexer / DataAdapter
生成 frame / clip / asset metadata
        |
        +----------------------+
        |                      |
        v                      v
Parquet + DuckDB          LanceDB
metadata 查询              embedding / hard case / 相似检索
训练集筛选
        |
        v
BEV 生成 / 模型推理 / 自动标注
        |
        v
GeoParquet
候选 lane boundary / centerline / crosswalk / road edge
        |
        v
离线质检 / 人工审核 / 地图编辑
        |
        v
PostGIS
正式语义地图 / 地图编辑器 / 空间查询 / 发布服务
        |
        v
地图版本 / routing graph / tile / map package
```
