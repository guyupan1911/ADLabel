# LaneMapDB 代码结构与业务流程分析

本文分析目录：

```text
/home/guyu/projects/mapping_build/lanemap_keyframe/scripts
```

重点是其中 `LaneMapDB` 相关代码，以及 `scripts` 目录下围绕 LaneMapDB 使用的辅助脚本。

## 1. LaneMapDB 的定位

`LaneMapDB` 是建图 2.0 相关的数据管理服务。它不是单纯保存点云或图片的数据库，而是保存各种建图资产的元信息、空间索引、版本状态和业务流状态。

核心职责可以概括为：

1. 管理原始 trip / clip 数据。
2. 管理 fusion / render / label / check 等建图中间产物和版本。
3. 支持通过 polygon / tile / region 做空间查询。
4. 支持人工 check、finetune、返修、补采、现实变化区域等业务流。
5. 支持回环 benchmark、回环标注、回环修正相关数据管理。
6. 通过 FastAPI 对外提供 HTTP 接口。

实际的大文件资产，例如传感器原始数据、render 结果、fusion 结果、label 文件、check 图片等，通常不直接存在 MySQL 表里，而是放在文件系统、共享盘或 S3 路径中。数据库保存的是路径、空间范围、版本、状态和关系。

## 2. 目录分层

`LaneMapDB` 目录主要分成三层：

```text
LaneMapDB/
├── infrastructure/   # 表结构、基础 DAO、MySQL 封装
├── engine/           # 面向业务的 API 封装
├── application/      # FastAPI HTTP 服务
├── unit_test/        # 测试、维护、刷库脚本
├── README.md
└── test_query_position.html
```

设计意图是：

```text
application
  -> 接收 HTTP 请求
  -> 调用 engine 业务接口
  -> engine 调用 infrastructure 表对象
  -> infrastructure 调用 database.py 执行 SQL
  -> MySQL 保存元数据和空间索引
```

这套结构接近轻量 DAO + Service + Web API 的设计。

## 3. infrastructure 层

### 3.1 database.py

`database.py` 是最底层的 MySQL 访问封装。

主要类是：

```python
class MySQLDatabase:
```

它接收一个 `pool`，调用：

```python
self.db = pool.connection()
```

这里的 `pool` 是 SQL 连接池，不是数据湖。连接池由 `dbutils.pooled_db.PooledDB` 创建，内部维护多个到 MySQL 的 TCP 连接，业务代码需要访问数据库时从池里取连接，用完再归还，避免频繁创建和销毁连接。

`database.py` 封装的能力包括：

- 创建表：`create_table()`
- 复制表：`copy_table()`
- 删除表：`delete_table()`
- 添加列：`add_column()`
- 修改列类型：`modify_column()`
- 删除列：`delete_column()`
- 插入：`insert()`
- 更新：`update()`
- 删除：`delete()`
- 查询：`query()` / `select()` / `count()` / `sum()`
- 几何对象转 SQL：`to_sql_polygon()` / `to_sql_trajectory()` / `to_sql_point()` / `to_sql_geometry()`

空间字段通过 MySQL Spatial 处理，例如：

```sql
ST_GeomFromText(...)
ST_CONTAINS(...)
ST_INTERSECTS(...)
ST_AsText(...)
```

这些都是 MySQL 官方的 Spatial Function，用来保存和查询 `geometry` 类型。

需要注意的是，这里大量 SQL 是手动字符串拼接完成的。这在内部工具中可以工作，但从工程角度看有几个风险：

- 表名、字段名、条件字符串如果来自外部输入，会有 SQL 注入风险。
- 没有统一 migration 版本管理。
- schema 变更依赖运行时 add / modify column。
- 类型检查主要靠调用方约定。

更标准的做法可以是 SQLAlchemy Core / ORM、Alembic migration，或者至少对值使用参数化 SQL，对标识符做白名单校验。

### 3.2 database_api.py

`database_api.py` 在 `database.py` 之上封装了“表”的概念。

核心类：

```python
class DataBaseAPI:
```

它约定每个子类需要定义：

```python
self.data_table = '表名'
self.data_column_object = {
    '字段名': 'MySQL字段类型',
}
```

也就是说：

- `database.py` 封装 MySQL 操作。
- `database_api.py` 封装一张表应该如何创建、插入、更新、查询。
- 各个 `*_db.py` 文件定义具体业务表。

重要函数：

#### get_data_obj()

```python
def get_data_obj(self):
    return namedtuple(self.data_table, list(self.data_column_object.keys()))
```

这个函数根据表结构生成一个 `namedtuple` 类型。它类似一个轻量 struct，用来构造一行数据。

例如 `lm_raw_data` 表有字段 `id/path/trajectory/...`，那么 `get_data_obj()` 会生成一个可以这样使用的对象：

```python
RawDataObj(id=..., path=..., trajectory=...)
```

`namedtuple` 本身不会强制检查 Python 类型，类型是否正确主要靠后续 SQL 插入和业务约定。

#### create_table()

创建表时会自动追加：

```text
created_time
updated_time
```

然后调用底层 `database.create_table()`。

#### update_table()

`update_table()` 的作用是让数据库里的表结构和 `data_column_object` 尽量对齐。

流程是：

1. 复制 `data_column_object`。
2. 自动追加 `created_time` / `updated_time`。
3. 对每个字段尝试 `add_column()`。
4. 如果添加失败，就调用 `modify_column()`。

所以它主要是“更新表有哪些 column，以及这些 column 的 SQL 类型”。

如果列已经存在，即使类型一样，`add_column()` 也会失败，然后进入 `modify_column()`。因此它可能会重复执行一遍 `ALTER TABLE MODIFY COLUMN`。

#### _get_insert_obj()

这个函数把 Python 的一行数据转换成可以插入 MySQL 的字典。

它会做几件事：

1. `data_obj._asdict()` 把 namedtuple 变成 dict。
2. 检查字段集合必须和 `data_column_object` 完全一致。
3. 如果字段类型包含 `geometry`，就转成 `ST_GeomFromText(...)`。
4. 如果字段类型包含 `json`，就 `json.dumps()`。
5. 自动添加 `created_time` 和 `updated_time`。

#### update()

`update()` 用于更新一部分字段。

它会：

1. 检查传入字段必须是表字段的子集。
2. 对 geometry/json 字段做转换。
3. 自动更新 `updated_time`。
4. 调用底层 `database.update()`。

#### get_id()

```python
def get_id(self):
    return self.__get_ids(1)[0]
```

内部使用：

```python
snowflake.client.get_guid()
```

这是为了给每一行生成全局唯一 ID。Snowflake 类 ID 通常由时间戳、机器号、序列号等组合生成，不依赖 MySQL 自增主键，适合多服务、多任务并发写入。

## 4. infrastructure 中的主要表

### 4.1 raw_data_db.py：lm_raw_data

`lm_raw_data` 表保存的是原始 trip / clip 数据的元信息。

它不是直接保存 lidar/camera/imu 数据内容，而是保存这些数据所在路径、轨迹、采集信息、质量指标和状态。

主要字段包括：

- `id`：全局唯一 ID。
- `collect_task`：采集任务。
- `collect_version`：采集版本。
- `collect_time`：采集时间。
- `task_id`：任务 ID。
- `produce_time`：数据生产时间。
- `path`：处理后或可用数据路径。
- `origin_path`：原始数据路径。
- `log_path`：日志路径。
- `city_code`：城市。
- `trajectory`：轨迹，MySQL geometry LineString。
- `sd_road_ids`：关联的 SD road id，json。
- `data_source`：数据来源。
- `version`：版本。
- `is_delete`：是否逻辑删除。
- `clips`：clip 信息，json。
- `delete_reason`：删除原因。
- `headtime` / `tailtime`：数据时间范围。
- `rtk_good_ratio`：RTK 质量比例。
- `precipitation` / `precipitation_flag`：降水相关质量信息。
- `vehicle_name`：车辆名。
- `raw_data_length`：数据长度。
- `s3_exist`：S3 是否存在。
- `lat_max/lat_min/lon_max/lon_min`：轨迹经纬度范围。
- `used_time`：使用时间。

典型空间查询：

```python
def query_by_polygon(self, polygon):
    return ... WHERE (
        ST_CONTAINS(polygon, trajectory)
        OR ST_INTERSECTS(polygon, trajectory)
    )
```

含义是：返回所有轨迹完全在 polygon 内，或者与 polygon 有交集的 raw data 行。

`check_and_update_missing_data()` 会遍历 `path` 并用 `os.path.exists()` 检查路径是否存在。如果不存在，就把 `is_delete=1`，`delete_reason='data missing'`。

这说明这套代码默认运行环境能直接访问 `path` 指向的共享盘或挂载路径。如果数据只在 S3 上，单纯 `os.path.exists()` 是不够的，需要改成 S3 SDK 或统一存储抽象。

### 4.2 fusion_data_db.py：lm_fusion_data

`lm_fusion_data` 保存 fusion / 多趟融合 / pose graph 优化结果的元信息。

典型字段：

- `id`
- `polygon`
- `city_code`
- `task_id`
- `produce_time`
- `path`
- `log_path`
- `used_raw_ids`
- `increment_raw_ids`
- `version`
- `is_latest_version`

从 `finetune_task_api.py` 的使用可以推测，`fusion_data.path` 下会包含：

```text
origin_keyframes.json
optimization_keyframes.json
loop_tasks.json
```

这些文件用于后续按 polygon 过滤 keyframe、生成 finetune task、筛选相关 loop。

### 4.3 render_data_db.py：lm_render_data

`lm_render_data` 管理 render / label / tile 版本。

这里的 render 更像是面向标注和质检的地图产品版本，而不是单张图片本身。

主要字段包括：

- `id`
- `tile_id`
- `city_code`
- `task_id`
- `produce_time`
- `path`
- `log_path`
- `fusion_id`
- `current_fusion_id`
- `version`
- `archive_status`
- `last_archive_time`
- `archive_times`
- `lock_user`
- `is_deleted`
- `isolation_status`
- `last_isolation_time`
- `isolation_reason`
- `check_status`
- `last_check_time`
- `is_latest_version`
- `is_latest_hd_version`
- `is_latest_upload_version`
- `label_mileage`
- `render_finish`
- `is_publish_version`

`tile_id` 在代码中不是 100m x 100m 的局部地图块，而是经纬度网格 ID，例如：

```text
113.720000_22.700000_0.020000
```

从 `check_data_api.py` 的 `get_tile_id_based_on_lon_lat()` 可以看出，tile 按 `0.02` 度划分，约为公里级范围，不是之前讨论的 WebMercator 100m grid。

`lm_render_data` 的作用包括：

- 管理某个 tile 当前最新上传版本。
- 管理 label / HD label 最新版本。
- 管理归档状态。
- 管理 check 状态。
- 管理发布版本。
- 记录该 render 使用了哪些 fusion 数据。

### 4.4 label_data_db.py：lm_label_data

`lm_label_data` 保存 label 版本信息。

从 `render_data_api.archive()` 可以看到，当 render 被 archive 时，会复制：

```text
label.json -> label_{render_id}.json
```

然后插入一条 label data 记录。

因此 `lm_label_data` 是 render 版本和 label 文件版本之间的索引表。

### 4.5 check_data_db.py：lm_check_data

`lm_check_data` 保存质检问题、人工 check 区域、返修区域、变化区域等空间对象。

主要字段包括：

- `id`
- `polygon`
- `tile_id`
- `city_code`
- `task_id`
- `produce_time`
- `path`
- `log_path`
- `render_id`
- `version`
- `type`
- `auto_check_status`
- `human_check_status`
- `human_repair_status`
- `todo_path`
- `road_type`
- `fusion_id`
- `change_time`
- `trip_id`
- `trip_clips`
- `dv_url`
- `loc_url`

`CheckDataAPI` 里定义的 type 语义大致是：

```text
0: 重影
1: 模糊不清
2: 现实变更
3: 不完整
4: 定位失败
5: capsule
6: 建图 check 区域
```

它可以通过 polygon 查询，也可以和 render/fusion/raw data 关联起来生成 finetune task。

### 4.6 area_to_collect_db.py：lm_area_to_collect

`lm_area_to_collect` 保存需要补采的区域。

典型字段包括：

- `id`
- `polygon`
- `render_id`
- `solved_render_id`
- `tile_id`
- `status`
- `text`
- `type`

状态含义在 engine 里大致是：

```text
-1: invalid
 0: init
 1: assigned
 2: sent into raw production
 3: solved
```

这个表用于标记“这个区域数据不够，需要重新采集或补采”。

### 4.7 area_be_changed_db.py：lm_area_be_changed

`lm_area_be_changed` 保存现实发生变化的区域。

典型字段包括：

- `id`
- `polygon`
- `head_time`
- `tail_time`
- `status`
- `render_id`
- `solved_render_id`
- `tile_id`
- `text`

从 engine 中可以看出：

- `status=0` 表示 unresolved。
- `status=1` 表示 solved。
- 当状态第一次变成 solved，会记录 `solved_render_id`。

它和 check / capsule 流程有关，用来管理“现实变化”导致的地图更新需求。

### 4.8 region_data_db.py：lm_region_data

`lm_region_data` 保存建图区域、region、tile 相关空间信息。

典型字段包括：

- `id`
- `polygon`
- `sd_road_ids`
- `region_version`
- `city_version`
- `sd_version`

这个表更像是规划层面的区域索引，用来描述哪些区域要建图、补图或参与任务生成。

### 4.9 loop_benchmark_db.py：lm_loop_benchmark

`lm_loop_benchmark` 保存回环质量、回环标注、回环 benchmark 数据。

主要字段包括：

- `id`
- `loop_id`
- `tile_id`
- `city_code`
- `path`
- `source`
- `store_status`
- `isolation`
- `src_tile_id`
- `point`
- `longitude`
- `latitude`
- `prelabel`
- `inlier_num`
- `labeled`
- `label_task_id`
- `label_path`
- `difficulty_level`
- `adjust_failed_status`
- `scene`
- `check_id`
- `produce_time`

`source` 大致表示来源：

```text
0: production
1: finetune
2: manual
```

代码会把回环 JSON 保存到类似路径：

```text
/bev/loop_benchmark/loop_data/product/{tile_id}/{id}.json
/bev/loop_benchmark/loop_data/finetune/{tile_id}/{id}.json
/bev/loop_benchmark/loop_data/label/{tile_id}/{id}.json
```

它还会比较 prelabel 和人工 label 之间的相对位姿误差，包括 rotation error 和 translation error。

这说明它不只是保存回环，还用于构建回环质量评估数据集。

### 4.10 capsule_* 表

`capsule_*_db.py` 是 capsule 业务流相关表，整体上和 raw / region / render / label 的设计类似，但面向 capsule 流程。

常见文件包括：

```text
capsule_raw_data_db.py
capsule_region_data_db.py
capsule_region_plus_data_db.py
capsule_render_data_db.py
capsule_label_data_db.py
```

从命名和字段可以推测，capsule 流程也需要管理：

- capsule 区域。
- capsule 原始数据。
- capsule render 结果。
- capsule label 版本。
- capsule 区域 plus data。
- 和现实变化区域、mini tile、tile 的关联。

## 5. engine 层

### 5.1 engine_api.py

`engine_api.py` 创建 MySQL 连接池。

核心代码类似：

```python
self.pool = PooledDB(
    creator=pymysql,
    host='...',
    user='...',
    password='...',
    database='...',
    port=...,
    autocommit=True,
    maxconnections=20,
)
```

它支持：

```text
production
 test
```

生产库和测试库的 host、user、database、port 都硬编码在代码里。

这说明 LaneMapDB 是一个中心化 MySQL 服务，不是本地文件数据库。

### 5.2 raw_data_api.py

`raw_data_api.py` 是 `RawDataDBAPI` 的业务封装。

主要能力：

- 查询 raw data。
- 更新 raw data 删除状态。
- 更新 delete reason。

例如：

```python
update_delete_info(id, is_delete, delete_reason)
```

用于把某段原始数据标记为不可用。

### 5.3 render_data_api.py

`render_data_api.py` 是 render / label / check / publish 版本管理的核心业务 API。

主要能力包括：

1. 插入新的 render 版本。
2. 根据 tile 更新最新上传版本。
3. 查询最新 render / label / HD label。
4. 根据 render_id 查询 check polygon。
5. archive render。
6. start_check。
7. start_label。
8. start_label_hd。
9. isolate render。
10. 更新 label mileage。
11. 更新 publish version。

典型流程是：

```text
render 结果产生
  -> insert render_data
  -> 标记 latest upload version
  -> start_check
  -> 生成 / 插入 check_data
  -> 人工或自动 check
  -> archive
  -> label_data 生成
  -> 更新 latest label / latest hd label / publish version
```

其中 `archive()` 会复制 label 文件并插入 label 表，说明 render_data 和 label_data 是版本联动关系。

### 5.4 check_data_api.py

`check_data_api.py` 管理质检问题和返修流程。

主要能力：

- 根据 lon/lat 计算 tile_id。
- 插入 check data。
- 接收 check result JSON。
- 更新 check / repair 状态。
- 触发 dreamviewer 视频生成。
- 触发 finetune task 生成。
- archive check data。

`insert_check_result()` 负责把前端或检查工具产生的 JSON 结果写入数据库。

它会根据问题类型字符串映射到内部 type，例如：

```text
Ghosting -> 0
Blurred -> 1
Reality Changed -> 2
Incomplete -> 3
Localization Failed -> 4
Capsule -> 5
Check Polygon -> 6
```

### 5.5 finetune_task_api.py

`finetune_task_api.py` 负责根据 check_data 生成 finetune 任务输入。

它不直接做 scan matching 或位姿优化，而是把相关数据筛出来，组织成一个可供 finetune 使用的任务目录。

主要流程：

1. 输入 `check_data_id`。
2. 读取 check_data 的 polygon。
3. 找到对应 render_data。
4. 找到 render 关联的 fusion_id。
5. 读取 fusion path 下的：

```text
origin_keyframes.json
optimization_keyframes.json
loop_tasks.json
```

6. 用 polygon 查询 raw_data，找到穿过这个区域的 trip。
7. 选择与 polygon 内 raw trajectory 交集最多的 fusion。
8. 把 polygon 转 UTM 并 buffer 约 100m。
9. 过滤 keyframes 和 loop tasks。
10. 输出：

```text
origin_keyframes.json
optimization_keyframes.json
loop_tasks.json
finetune_info.json
```

这说明 finetune task 是从已有 fusion 结果里裁剪出来的局部任务。

### 5.6 area_to_collect_api.py

`area_to_collect_api.py` 管理需要补采的区域。

业务含义是：如果某个区域数据不足、地图不完整，系统或人工可以插入一个 polygon，后续采集或生产流程可以根据这个表生成补采任务。

### 5.7 area_be_changed_db.py

这个文件在 `engine` 目录下，名字里带 `_db`，但实际承担的是业务 API 角色。

它管理现实变化区域，例如道路改造、车道变化、施工变化等。

如果某个变化区域被新的 render 版本解决，会把状态改为 solved，并记录 solved_render_id。

## 6. application 层

### 6.1 lanemap_db_app.py

`application/lanemap_db_app.py` 是 FastAPI 服务入口。

它把 engine 层的能力暴露成 HTTP 接口。

主要接口包括：

#### render_data

```text
/render_data/archive/{id}
/render_data/archive_batch/
/render_data/start_check/{id}
/render_data/start_label/{id}
/render_data/start_label_hd/{id}
/render_data/get_check_data_polygons/{id}
/render_data/update_label_mileage/
```

#### check_data

```text
/check_data/generateFinetuneFilesFromCheckId/{check_data_id}
/check_data/insert_check_result/
/check_data/start_repair/{id}
/check_data/start_repair_batch/
/check_data/archive/
```

#### area_to_collect

```text
/area_to_collect/insert_atc/
/area_to_collect/insert_atc_batch/
/area_to_collect/query_atc/{tile_id}
```

#### area_be_changed

```text
/area_be_changed/insert_abc/
/area_be_changed/insert_abc_batch/
/area_be_changed/query_abc/{tile_id}
```

#### raw_data

```text
/raw_data/update_delete_info/
```

这个文件说明 LaneMapDB 不是只给 Python 脚本直接调用，也可以作为一个服务被前端页面、标注平台、质检平台或自动化任务系统调用。

需要注意几个工程问题：

1. `mode` 是在 `__main__` 中设置的全局变量，如果用 `uvicorn module:app` 方式启动，可能没有初始化。
2. 多个 route 函数复用了相同函数名，例如 `archive`，FastAPI 可以注册成功，但 OpenAPI operation name 可能冲突。
3. 接口中会直接构造 engine 对象并访问硬编码数据库配置。

## 7. unit_test 目录

`unit_test` 目录并不只是普通单元测试，也包含很多刷库、维护、生产修复脚本。

常见类型包括：

- infrastructure 表创建 / 更新测试。
- production / test 数据库维护脚本。
- FastAPI HTTP 接口测试。
- render_data / check_data / finetune_task 业务测试。
- loop benchmark 插入和更新测试。
- capsule region 测试。
- 手动插入 change region / check data 的脚本。
- 状态刷写脚本。

因此运行这些脚本要非常谨慎。部分脚本会连接 production，并执行 insert/update/delete。

## 8. test_query_position.html

`test_query_position.html` 是一个静态地图调试页面。

它使用 Leaflet / Folium 生成地图，并画出若干 GeoJSON LineString。

它不是服务端程序，也不是 LaneMapDB 的核心业务代码，主要用于可视化某些轨迹或查询结果在地图上的位置。

使用方式通常是：

```text
直接用浏览器打开这个 HTML
```

或者在本地起一个简单 HTTP server 后访问。

## 9. scripts 目录下 LaneMapDB 以外的辅助脚本

`lanemap_keyframe/scripts` 下除了 `LaneMapDB`，还有大量围绕数据库和建图资产的辅助脚本。

可以按功能分成几类。

### 9.1 raw data / task 生成

相关文件示例：

```text
generate_task_file_by_collect_task_id.py
generate_task_file_by_task_id.py
generate_task_file_from_datahub.py
generate_test_task_file_by_rawdata.py
get_raw_data_reproduction_task.py
upload_raw_data_reproduction.py
submit_raw_data_pipeline_c.py
submit_multi_raw_data_pipeline.py
submit_multi_raw_data_pipeline_new.py
```

这些脚本通常用于从数据库或外部系统里取 raw data，生成 pipeline 输入任务，或者提交 raw data 生产任务。

### 9.2 keyframe / polygon / finetune 任务裁剪

相关文件示例：

```text
get_keyframes_in_polygons.py
get_keyframes_in_polygons_function.py
get_keyframes_in_polygons_function_new.py
get_polygons_and_pose_data.py
create_finetune_tasks.py
```

这类脚本和 `finetune_task_api.py` 思路一致，重点是：

- 根据 polygon 找相关 raw data / keyframe。
- 从 fusion 结果里裁剪局部 keyframe。
- 生成 finetune 或检查任务输入。

### 9.3 finetune / check 状态更新

相关文件示例：

```text
check_polygons_finetune_status_new.py
update_polygons_finetune_status.py
update_polygons_finetune_status_new.py
execute_polygons_data_statistics.py
finetune_statistic.py
```

这些脚本用于统计或更新 finetune/check 相关状态。

### 9.4 raw trajectory / pose 工具

相关文件示例：

```text
raw_traj_to_origin_keyframes.py
poses_dispatch_c.py
check_traj.py
trans_utils.py
```

这些脚本围绕轨迹、pose、keyframe 转换和检查。

### 9.5 S3 / 文件同步

相关文件示例：

```text
s3_download.py
download_files_from_s3.py
upload_files_recursively_to_s3.py
sync_raw_data_pcd.py
tmp_upload.py
```

说明 LaneMapDB 保存的很多 path 实际可能指向共享盘或 S3 资产。

### 9.6 清理和运维脚本

相关文件示例：

```text
delete_raw_data.py
delete_datas.py
delete_parser_datas.py
update_raw_data_is_deleted_in_database.py
tmp_check_metafile.py
tmp_submit_tasks.py
```

这些脚本用于删除、清理、检查、修复数据库和文件资产。

## 10. 关键业务流程推断

### 10.1 原始数据入库和空间查询

```text
采集数据 / raw data 生产
  -> 生成 path / origin_path / log_path / trajectory
  -> 写入 lm_raw_data
  -> 后续可以用 polygon 查询经过该区域的 trip
```

`lm_raw_data` 是整个系统的基础入口。后续 check、finetune、region 查询都需要根据空间区域找到相关 raw data。

### 10.2 fusion 结果管理

```text
多趟 raw data / keyframe / loop / pose graph optimization
  -> 生成 fusion 结果目录
  -> 写入 lm_fusion_data
  -> fusion path 下保存 origin_keyframes / optimization_keyframes / loop_tasks
```

fusion data 是后续 render 和 finetune 的来源。

### 10.3 render / label / check 版本管理

```text
fusion result
  -> render 生成 tile 级标注底图或地图产品
  -> 写入 lm_render_data
  -> start_check
  -> check_data 插入问题区域
  -> start_label / archive
  -> label_data 记录 label 版本
  -> 更新 latest label / latest hd label / publish version
```

`lm_render_data` 管的是 tile 级产品版本，`lm_label_data` 管 label 文件版本，`lm_check_data` 管问题区域和返修状态。

### 10.4 check -> finetune task

```text
check_data polygon
  -> 找 render_data
  -> 找 fusion_data
  -> polygon 查询 raw_data
  -> 过滤 origin_keyframes / optimization_keyframes / loop_tasks
  -> 输出局部 finetune 任务目录
```

这一步不是直接做优化，而是给局部修复或人工检查准备数据。

### 10.5 补采区域和现实变化区域

```text
地图质量不足 / 数据缺失
  -> lm_area_to_collect
  -> 后续采集任务处理

现实道路变化
  -> lm_area_be_changed
  -> 后续 capsule / render / check 流程处理
```

这两类表解决的是“哪里需要重新采集”和“哪里现实已经变化”。

### 10.6 回环 benchmark

```text
生产 / finetune / 人工产生 loop
  -> 保存 loop json
  -> 写入 lm_loop_benchmark
  -> 人工标注后写 label_path / difficulty / error
  -> 用于评估回环质量
```

`lm_loop_benchmark` 更像是回环质量数据集和回环标注管理系统。

## 11. 设计优点

1. 数据资产不直接塞进数据库，只保存路径和元数据，适合大文件系统。
2. 利用 MySQL Spatial 支持 polygon / trajectory 空间查询。
3. raw / fusion / render / label / check / finetune 流程之间有清晰 ID 关联。
4. 通过 FastAPI 暴露业务能力，可以被前端、平台、脚本共同使用。
5. Snowflake ID 适合多任务并发写入。
6. 通过 test / production mode 区分环境。

## 12. 主要风险和改进点

### 12.1 SQL 字符串拼接风险

当前很多 SQL 通过字符串拼接生成。建议至少：

- 值使用参数化 SQL。
- 表名、字段名使用白名单。
- range 条件避免直接接收外部字符串。

更长期可以考虑 SQLAlchemy Core / ORM + Alembic。

### 12.2 数据库配置硬编码

生产和测试库地址、用户名、密码写在代码中。

建议改成：

- 环境变量。
- 配置文件。
- Kubernetes secret。
- Vault / secret manager。

### 12.3 unit_test 中包含生产维护脚本

`unit_test` 目录里有不少脚本会连生产库并执行更新。

建议区分：

```text
unit_test/        真正单元测试
maintenance/     生产维护脚本
examples/         调用示例
```

### 12.4 表结构更新方式较粗糙

`update_table()` 会尝试 add column，失败就 modify column。

这不是真正的 migration 管理。建议使用 Alembic 或版本化 SQL migration。

### 12.5 路径假设强

代码里有很多固定路径：

```text
/mnt/road
/road_data
/bev/loop_benchmark
/lanemap_workspace
```

如果换环境，容易出现路径不存在或权限问题。

### 12.6 FastAPI app 的 mode 初始化方式不稳

如果不是通过脚本 `__main__` 启动，而是通过标准 uvicorn module 方式启动，`mode` 可能没有正确初始化。

建议将 mode 放入 app state 或配置对象。

## 13. 推荐阅读顺序

如果要系统理解 LaneMapDB，建议按下面顺序看：

1. `LaneMapDB/README.md`
2. `infrastructure/database.py`
3. `infrastructure/database_api.py`
4. `infrastructure/raw_data_db.py`
5. `infrastructure/fusion_data_db.py`
6. `infrastructure/render_data_db.py`
7. `infrastructure/check_data_db.py`
8. `infrastructure/label_data_db.py`
9. `infrastructure/area_to_collect_db.py`
10. `infrastructure/area_be_changed_db.py`
11. `infrastructure/loop_benchmark_db.py`
12. `engine/engine_api.py`
13. `engine/raw_data_api.py`
14. `engine/render_data_api.py`
15. `engine/check_data_api.py`
16. `engine/finetune_task_api.py`
17. `engine/area_to_collect_api.py`
18. `engine/area_be_changed_db.py`
19. `application/lanemap_db_app.py`
20. `scripts/get_keyframes_in_polygons*.py`
21. `scripts/create_finetune_tasks.py`
22. `scripts/upload_LaneMapDB_pipeline_c.py`
23. `unit_test/` 中对应业务的测试或维护脚本

这个顺序先理解底层 SQL 和表，再理解业务 API，最后看 HTTP 服务和外围脚本，会比较顺。

## 14. 总结

LaneMapDB 的核心不是保存地图本身，而是保存建图生产系统里的“资产索引”和“业务状态”。

它通过 MySQL + Spatial 管理：

- 哪些 raw trip 经过某个区域。
- 哪些 fusion 结果覆盖某个区域。
- 某个 tile 当前有哪些 render / label / check / publish 版本。
- 哪些区域需要补采或已经现实变化。
- 哪些 check 区域需要生成 finetune 任务。
- 哪些 loop closure 需要进入 benchmark 或人工标注。

从工程结构上看，它是一套面向建图生产的轻量数据中台：MySQL 保存元数据，文件系统/S3 保存大文件，FastAPI 对外提供业务接口，脚本负责批量导入、任务生成和状态维护。
