# ADLabel + LaneMapDB 讨论记忆

这份文档用于给后续 Codex 会话提供上下文。新的对话中可以直接让 Codex 先阅读本文，再继续 ADLabel 数据湖、数据库索引层、任务系统和 LaneMapDB 迁移/参考设计相关工作。

## 1. 背景

ADLabel 的目标不是一个单一标注工具，而是一个自动驾驶数据生产平台。

核心抽象是：

```text
ADLabel = 数据湖 + 数据库索引层 + 任务系统 + 算法执行器 + 标注/版本管理
```

稳定对象包括：

```text
Data Asset
Index Record
Task
Manifest
Version
```

基本原则：

```text
大文件放数据湖
数据库保存 URI、metadata、索引、关系、版本和任务状态
任务系统从数据库查询数据，固化 manifest，执行算法，产物回写数据湖，索引回写数据库
```

## 2. LaneMapDB 的定位

LaneMapDB 是建图 2.0 产线中的数据库和业务 API 系统。

它不是数据湖本身，而是：

```text
建图/标注产线的 MySQL Spatial 索引库 + 业务状态库
```

它管理：

- 原始 trip / clip 数据索引。
- fusion / render / label / check 等建图中间产物和版本。
- polygon / tile / region 空间查询。
- 人工 check、finetune、返修、补采、现实变化区域等业务状态。
- 回环 benchmark、回环标注、回环修正相关数据。
- FastAPI 对外服务接口。

LaneMapDB 大文件不直接进 MySQL，而是用 path 指向共享盘、S3 或本地文件系统。

## 3. LaneMapDB 与 ADLabel 的关系

LaneMapDB 可以作为 ADLabel 数据库索引层和业务访问层的一个已有实现雏形。

对应关系：

```text
ADLabel Database / Index Layer
  -> LaneMapDB infrastructure/database.py
  -> LaneMapDB infrastructure/database_api.py
  -> LaneMapDB infrastructure/*_db.py

ADLabel Task / Business API Layer
  -> LaneMapDB engine/engine_api.py
  -> LaneMapDB engine/*_api.py

ADLabel Web / Service Layer
  -> LaneMapDB application/lanemap_db_app.py
```

更细对应：

```text
database.py
  = SQL/MySQL 基础能力
  = ADLabel DB Adapter / SQL Adapter

database_api.py
  = 通用表操作基类
  = ADLabel Table DAO / Repository Base

*_db.py
  = 具体表的 DAO / Repository

engine_api.py
  = 数据库连接池 / 环境选择

engine/*_api.py
  = 面向业务流程的 Service

application/lanemap_db_app.py
  = HTTP API
```

## 4. LaneMapDB 可以复用的设计思想

LaneMapDB 的分层思想是正确的：

```text
Database Adapter
  -> Table DAO / Repository
  -> Business Service / Engine
  -> HTTP API
```

ADLabel 可以参考这个分层，但不建议原样复制 LaneMapDB 的手写 SQL 和业务表。

推荐迁移思想，而不是迁移实现细节。

## 5. 为什么不建议直接照搬 LaneMapDB

LaneMapDB 当前实现的问题：

1. 表是建图业务定制的，不够通用。
2. SQL 多数手写字符串拼接，安全性和维护性一般。
3. MySQL 配置硬编码在代码里。
4. schema migration 方式较粗糙，通过 add column / modify column 动态补齐。
5. 缺少完整的 frame / asset / pose / job / artifact 抽象。
6. task manifest 和 artifact registry 不是通用设计。
7. 空间查询使用 MySQL Spatial，可以工作，但复杂 GIS 能力和生态不如 PostGIS。

所以 ADLabel 不应直接复制：

```text
LaneMapDB database.py + database_api.py + 手写 SQL DAO
```

而应该采用成熟开源方案实现类似分层。

## 6. ADLabel 推荐数据库技术栈

如果 ADLabel 要做长期系统，推荐：

```text
PostgreSQL + PostGIS
SQLAlchemy 2.x
GeoAlchemy2
Alembic
Pydantic
FastAPI
Repository / Service 分层
```

对应关系：

```text
LaneMapDB database.py
  -> SQLAlchemy Engine / Session

LaneMapDB database_api.py
  -> SQLAlchemy ORM Model + Repository Base

LaneMapDB raw_data_db.py / render_data_db.py / check_data_db.py
  -> ADLabel models + repositories

LaneMapDB engine/*_api.py
  -> ADLabel services

LaneMapDB lanemap_db_app.py
  -> ADLabel FastAPI routes
```

## 7. ORM 是什么

ORM = Object Relational Mapping，对象关系映射。

它把关系型数据库中的：

```text
table / row / column
```

映射成程序里的：

```text
class / object / attribute
```

例如：

```text
Trip class    -> trips 表
Trip.id       -> trips.id 字段
一个 Trip 对象 -> trips 表里的一行
```

ORM 让业务代码尽量用对象方式操作数据库，而不是到处手写 SQL。

LaneMapDB 的 `database_api.py + *_db.py` 可以理解成一个手写的轻量 DAO / ORM 雏形。

更成熟的通用方案是 SQLAlchemy。

## 8. 是否已有通用 ORM 方案

有。Python 生态最推荐：

```text
SQLAlchemy
```

其他常见 ORM：

```text
Django ORM
SQLModel
Tortoise ORM
Prisma
TypeORM
Hibernate
Entity Framework
Diesel
SeaORM
```

对 ADLabel 来说，如果后端是 Python / FastAPI，推荐：

```text
SQLAlchemy 2.x + Alembic + Pydantic
```

如果需要空间查询：

```text
SQLAlchemy 2.x + GeoAlchemy2 + PostgreSQL/PostGIS
```

## 9. ADLabel 是否应该封装数据库和表

应该。

ADLabel 仍然需要封装：

```text
1. 数据库连接、事务、session
2. 表结构和字段类型
3. 单表和常用查询
4. 业务流程
5. HTTP API
```

但建议用现代方案实现：

```text
db/session.py
  -> 数据库连接、session、transaction

models/*.py
  -> ORM table model

repositories/*.py
  -> 单表和常用查询封装

services/*.py
  -> 业务流程封装

api/routes/*.py
  -> FastAPI 接口
```

而不是每张表都手写 SQL 字符串。

## 10. 推荐 ADLabel 目录结构

```text
adlabel/db/
  session.py
  base.py
  geometry.py
  migrations/

adlabel/models/
  dataset.py
  trip.py
  frame.py
  asset.py
  pose.py
  calibration.py
  job.py
  artifact.py
  annotation.py
  map.py

adlabel/repositories/
  trip_repository.py
  frame_repository.py
  asset_repository.py
  pose_repository.py
  job_repository.py
  artifact_repository.py

adlabel/services/
  data_lake_service.py
  task_service.py
  bev_service.py
  annotation_service.py
  map_service.py

adlabel/api/
  routes/
```

## 11. ADLabel 核心表建议

ADLabel 不应该直接使用 LaneMapDB 的业务表作为全部核心表。

建议核心通用表：

```text
datasets
trips
frames
frame_assets
frame_poses
calibrations
jobs
artifacts
bev_products
height_maps
semantic_annotations
annotation_versions
dataset_versions
```

建图业务可以作为 mapping domain：

```text
mapping_raw_data
mapping_fusion_data
mapping_render_data
mapping_check_data
mapping_loop_benchmark
mapping_area_to_collect
mapping_area_be_changed
```

这样既保留 LaneMapDB 的建图业务经验，又不会把 ADLabel 绑定成单一建图产线系统。

## 12. DuckDB 与 PostgreSQL 的区别

DuckDB 和 PostgreSQL 都可以是关系型数据库，但运行方式不同。

分类不是同一个维度：

```text
维度 1：数据模型
  关系型数据库 / 文档数据库 / 图数据库 / 向量数据库

维度 2：运行方式
  嵌入式数据库 / 客户端-服务器数据库
```

所以：

```text
DuckDB 是关系型数据库，也是嵌入式数据库
SQLite 是关系型数据库，也是嵌入式数据库
PostgreSQL 是关系型数据库，也是客户端-服务器数据库
MySQL 是关系型数据库，也是客户端-服务器数据库
```

DuckDB：

```text
关系型数据库 + 嵌入式运行方式 + 偏离线分析
不需要单独部署服务
通常直接在 Python/C++ 进程里打开本地数据库文件或 Parquet 文件
查询主要还是 SQL / SQL-like API
```

PostgreSQL：

```text
关系型数据库 + 服务端运行方式 + 偏在线业务/事务/空间索引
通常需要部署数据库服务
适合多用户、多进程、权限、事务、并发、版本管理
```

ADLabel 推荐分工：

```text
PostgreSQL/PostGIS + SQLAlchemy
  -> 在线索引、任务状态、多人协作、空间查询、版本管理

DuckDB + Parquet
  -> 离线统计、训练样本筛选、批量导出、质量分析
```

## 13. DuckDB API 是否也是数据库封装

是。

DuckDB 的 Python API 和 C++ API 封装了：

```text
数据库实例
连接 / session
表
SQL 执行
查询结果
事务
文件格式读取
```

但它不是 ORM。

DuckDB API 更像：

```text
嵌入式分析数据库 API
SQL Execution API
Relation API
Result API
```

它通常仍然需要写 SQL 或 SQL-like 表达式。

## 14. LaneMapDB 和 ADLabel 的最终判断

LaneMapDB 可以作为 ADLabel 的可行 v0 参考，尤其适合：

```text
管理 trip
查询 polygon 内 raw data
管理建图产物
管理 render / label / check 状态
生成 finetune task
可视化轨迹和区域
```

但如果 ADLabel 要成为通用自动驾驶数据湖平台，还需要补齐：

```text
frame 级索引
asset 索引
pose/calibration 多版本
通用 jobs / artifacts / manifest
dataset version
annotation version
map version
训练导出
embedding / hardcase 检索
标准 schema migration
配置和权限管理
```

一句话：

```text
LaneMapDB 的分层思想可以复用；
LaneMapDB 的手写 SQL 实现不建议原样复用；
ADLabel 应该用 SQLAlchemy/PostGIS/Alembic 等成熟方案实现同类能力。
```

## 15. 下次 Codex 使用方式

新开对话时，可以这样说：

```text
请先读取 /home/guyu/projects/ADLabel/docs/adlabel_lanemapdb_codex_memory.md，
然后继续设计 ADLabel 的数据库索引层和任务系统。
```

这份文档不是正式 Codex skill，而是项目上下文记忆。

不建议命名为 `skill`，除非要做成可安装的 Codex skill，包含明确触发条件、工作流程和工具约束。

建议命名方式：

```text
adlabel_lanemapdb_codex_memory.md
adlabel_lanemapdb_design_context.md
adlabel_database_architecture_notes.md
```
