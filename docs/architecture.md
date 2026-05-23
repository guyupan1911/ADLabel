# ADLabel Architecture

本文档描述 ADLabel 的整体架构、各模块职责，以及模块之间如何协作。
关键技术选型的详细论证记录在 [decisions/](decisions/) 下的 ADR 中，本文不重复展开，只在相关位置交叉引用。

## Table of Contents

- [Design Principles](#design-principles)
- [System Overview](#system-overview)
- [Data Model](#data-model)
- [Module Breakdown](#module-breakdown)
  - [io — DataSource 抽象](#io--datasource-抽象)
  - [indexer — Frame 索引生成](#indexer--frame-索引生成)
  - [data — RawDataReader](#data--rawdatareader)
  - [ops — Operator 抽象](#ops--operator-抽象)
  - [pipeline — 编排](#pipeline--编排)
  - [proto / common — 基础设施](#proto--common--基础设施)
- [End-to-End Data Flow](#end-to-end-data-flow)
- [Process and Deployment Model](#process-and-deployment-model)
- [Evolution Path](#evolution-path)
- [Open Questions](#open-questions)
- [References](#references)

---

## Design Principles

ADLabel 的所有模块设计都遵循以下五条原则。后续每次模块设计冲突时，按这些原则的顺序判优。

1. **数据来源解耦**
   下游算法不应该感知数据来自 rosbag、MCAP 还是 KITTI 目录。任何对数据来源的 if-else 都收敛在 `io/` 和 `data/` 两层。

2. **原始数据不可变 + 派生数据独立存储**
   原始传感器数据是不可变的 source of truth；算法产出（运动补偿点云、检测结果、标注）是可重生的派生数据。两者分开存储，保证派生数据可以随算法迭代重生成而不污染原始数据。

3. **Frame 是模块间唯一通用契约**
   模块之间不直接传递点云/图像对象，只传递 Frame 索引。需要原始数据时，统一通过 `RawDataReader` 按 Frame 取。

4. **依赖注入优于自管理**
   所有跨模块的资源（reader、cache、模型）由调用方装配后注入，组件内不自行 new。这一条直接决定了 pipeline 内 reader 句柄能不能复用。

5. **现在写的代码要为分布式留好接口，但不为分布式提前优化**
   单进程跑通永远是第一优先级。分布式（Ray）作为 Stage 3 的扩展点存在，但不影响当下的代码组织。

## System Overview

ADLabel 在概念上分三层，从下到上依次是：**数据层 → 访问层 → 计算层**。

```
                        ┌─────────────────────────────────────┐
计算层 (compute)        │   Operator Pipeline                  │
                        │   LIO  →  Stitcher  →  Detection ... │
                        └──────────────────┬──────────────────┘
                                           │ 按 Frame 取数据
                                           ▼
                        ┌─────────────────────────────────────┐
访问层 (access)         │   RawDataReader (dispatcher)         │
                        │   ├── FileRawDataReader              │
                        │   └── McapRawDataReader              │
                        └──────────────────┬──────────────────┘
                                           │
                                           ▼
                        ┌─────────────────────────────────────┐
数据层 (data)           │   Frame 索引   +   原始数据           │
                        │   frames.bin       MCAP / blobs       │
                        └──────────────────▲──────────────────┘
                                           │ Indexer 写入
                        ┌──────────────────┴──────────────────┐
                        │   DataSource (rosbag/mcap/kitti...)  │
                        └─────────────────────────────────────┘
```

**三个层各自的稳定性不同**：

| 层 | 变化频率 | 影响范围 |
|---|---|---|
| 数据层（Frame proto + 存储格式） | 低，需要谨慎演进 | 改动会让历史索引失效 |
| 访问层（RawDataReader 接口） | 中，新增后端时扩展 | 改动只影响计算层接口面 |
| 计算层（Operator 实现） | 高，每次新算法就有改动 | 局部，不影响其他模块 |

把"经常变"的层放在最上层、把"很少变"的层放在最下层，是这个分层的核心动机。

## Data Model

### Frame：模块间唯一通用契约

Frame 是 ADLabel 一切流转的核心数据结构。它**不包含**点云/图像本身，只包含元数据 + 定位原始数据所需的索引。

```protobuf
message Frame {
    // ===== 标识 =====
    string fid = 1;                  // 唯一 ID
    double timestamp = 2;            // sensor / publish time，秒
    string trip_id = 3;
    string sensor_name = 4;
    SensorType sensor_type = 5;
    string prev_id = 6;
    string next_id = 7;

    // ===== Pose / 标定 =====
    Pose3DMessage lio_pose_3d = 8;
    Pose3DMessage sensor_to_imu_extrinsic = 9;

    // ===== 原始数据定位（互斥，由 indexer 决定走哪种）=====
    oneof raw_data_ref {
        FileRef file_ref = 30;
        McapRef mcap_ref = 31;
    }

    // ===== 派生数���（算法产出，独立存储为文件）=====
    string compensated_point_cloud_uri = 40;
    string filtered_point_cloud_uri = 41;
    string object_detection_uri = 42;
}

message FileRef {
    string uri = 1;                  // file:// 或 s3:// 等
}

message McapRef {
    string mcap_uri = 1;             // 指向某个 trip 的 MCAP 文件
    string topic = 2;                // /lidar/front/points
    uint64 log_time_ns = 3;          // MCAP 的索引键
    uint32 message_size = 4;         // 唯一推荐冗余的字段，便于预读策略
}
```

详细字段说明见 [modules/frame.md](modules/frame.md)。
关于"为什么 McapRef 不存字节偏移"的论证见 [decisions/0007-no-mcap-byte-offsets.md](decisions/0007-no-mcap-byte-offsets.md)。

### TripMetadata：trip 级别的汇总

```protobuf
message TripMetadata {
    string trip_id = 1;
    string source_uri = 2;           // 原始数据来源 (rosbag/mcap/kitti dir)
    double start_time = 3;
    double end_time = 4;

    repeated SensorSummary sensors = 5;
    PipelineInfo pipeline = 6;
    RunStatistics statistics = 7;
}
```

每个 trip 一份 `trip_metadata.pb`，配合 `frames.bin`（一连串 Frame）构成 trip 的全部索引。

### Why MCAP for raw data

`McapRef` 这种"轻量索引 + 指回单一 MCAP 容器"的设计，对应行业里的"原始数据层 + 索引层"分层：

- **MCAP** ([mcap.dev/guides](https://mcap.dev/guides) / [foxglove/mcap](https://github.com/foxglove/mcap)) 是 Foxglove 推出、ROS 2 官方推荐的标准日志容器格式，自描述 + 编码无关 + 内置 chunk/message index 支持随机访问（详见 [Introducing the MCAP File Format](https://foxglove.dev/blog/introducing-the-mcap-file-format) 和 [MCAP Indexing](https://mmhaskell.com/blog/2025/12/22/mcap-indexing) 对内部索引结构的拆解）
- 顶级 AV 公司在此基础上额外做"训练特征/标签层"——参考 [LanceDB 的 AV/ML 数据栈](https://www.lancedb.com/blog/unifying-the-av-ml-stack-lancedb)、[Argoverse 2 用 Feather/Arrow 重编码 frame 数据](https://github.com/argoverse/av2-api)、以及 [列存随机访问优化研究 arxiv 2504.15247](https://arxiv.org/html/2504.15247v1) 把 Parquet 随机读优化 60x 的工作

ADLabel Stage 1 只做到第一层（原始 MCAP + 轻量 Frame 索引）。第二层（Lance/Parquet 训练数据）作为 Stage 4 的扩展点。综述视角的对照参考 [Awesome-Data-Centric-Autonomous-Driving](https://github.com/LincanLi98/Awesome-Data-Centric-Autonomous-Driving)。

## Module Breakdown

### io — DataSource 抽象

**职责**：把异构数据来源（rosbag / MCAP / KITTI / nuScenes / 普通 PCD 目录）统一成一个可遍历的传感器消息流。

这种"统一抽象 + 多后端实现"的思路在 AV 数据工具里几乎是标配：[Argoverse 2 API](https://github.com/argoverse/av2-api) 把 Feather/Arrow 后端封装在统一的 dataset 接口下，nuScenes devkit 用 SQLite + blob 但暴露统一的 `NuScenes` 类，OpenPCDet/mmdetection3d 的 Dataset 抽象同理。ADLabel 把这层显式提到主代码层，让算法对数据来源完全无感。

**核心接口**：

```cpp
namespace adlabel::io {

class DataSource {
public:
    virtual ~DataSource() = default;
    virtual bool Open() = 0;
    virtual bool HasNext() const = 0;
    virtual bool ReadNext() = 0;

    virtual std::string CurrentTopic() const = 0;
    virtual double      CurrentTimestamp() const = 0;
    virtual SensorType  CurrentSensorType() const = 0;

    virtual PointCloudView GetPointCloud() const = 0;
    virtual ImageView      GetImage() const = 0;
    virtual ImuSampleView  GetImu() const = 0;
    virtual GnssView       GetGnss() const = 0;

    virtual void Close() = 0;
};

}  // namespace adlabel::io
```

**实现矩阵**（按计划逐步补齐）：

| 实现 | Stage | 说明 |
|---|---|---|
| `McapDataSource` | Stage 1 | 主力实现，借助 Foxglove MCAP C++ 库 |
| `RosbagDataSource` | Stage 1 | 通过 Python 预处理转 MCAP，C++ 不再依赖 ROS。详见 [decisions/0004-no-ros-dependency.md](decisions/0004-no-ros-dependency.md) |
| `KittiDataSource` | Stage 2 | 目录结构 + timestamps.txt |
| `NuScenesDataSource` | Stage 2 | SQLite metadata + blob 文件 |

**关键设计**：返回类型用 `*View` 而不是 `sensor_msgs::*` 或具体 PCL 类型。`*View` 是 ADLabel 自定义的轻量结构（指向源数据的指针 + 字段描述），避免接口绑定任何具体生态。

### indexer — Frame 索引生成

**职责**：遍历 DataSource，给每帧分配 fid、关联标定外参、生成 Frame 索引和 TripMetadata。

> 这个模块原型为 `drive/map/dumper`。新名称 `indexer` 更贴合"生成索引"的本质（详见 [decisions/0009-rename-dumper-to-indexer.md](decisions/0009-rename-dumper-to-indexer.md)）。

**两种工作模式**：

```
Mode A: 索引 only（推荐，原始数据是 MCAP 时）
  DataSource → 遍历 → Frame[mcap_ref=...] → frames.bin
  原始数据本身不动，frames.bin 指回 MCAP 内的 (topic, log_time)

Mode B: 索引 + 落盘（兼容传统 dump 流程）
  DataSource → 遍历 → 解码并落盘 PCD/JPEG → Frame[file_ref=...] → frames.bin
```

第一版以 Mode A 为主。Mode B 保留是为了兼容 KITTI 这种"原始数据已经是文件"的场景。

**输入**：DataSource 配置 + SensorRig（标定）+ 输出目录
**输出**：`frames.bin`（一组 Frame）+ `trip_metadata.pb` + （Mode B 时）原始数据文件

详细见 [modules/indexer.md](modules/indexer.md)。

### data — RawDataReader

**职责**：根据 Frame 的 `raw_data_ref` 自动分派到正确的后端，返回**类型化**的传感器数据。

> ⚠️ 这一层不叫 "Filesystem"。Filesystem 暗示"按路径读字节"，但 MCAP 后端是按 (topic, log_time) 查询。详见 [decisions/0010-rawdatareader-naming.md](decisions/0010-rawdatareader-naming.md)。

**核心接口**：

```cpp
namespace adlabel::data {

class RawDataReader {
public:
    virtual ~RawDataReader() = default;
    virtual pcl::PointCloud<PointXYZIRT>::Ptr ReadPointCloud(const Frame& frame) = 0;
    virtual cv::Mat                          ReadImage(const Frame& frame) = 0;
    virtual std::vector<ImuSample>           ReadImu(const Frame& frame) = 0;
};

}  // namespace adlabel::data
```

**实现结构**：

```
RawDataReader (interface)
    ├── FileRawDataReader      ← 处理 FileRef
    ├── McapRawDataReader      ← 处理 McapRef，内部维护 mcap_uri → handle 缓存
    └── DispatchRawDataReader  ← 默认实现，根据 frame.raw_data_ref_case() 分派
```

**MCAP 句柄缓存**是这一层最关键的性能点。一个 MCAP 文件 open 一次代价不小（要解析 summary section 建索引）。`McapRawDataReader` 内部维护 `unordered_map<mcap_uri, shared_ptr<McapReader>>`，让多个 Frame 共享同一个 reader 句柄。

**派生数据不走这一层**：算法产出（运动补偿点云、检测结果）是文件 + URI，不需要分派逻辑，直接读文件即可。RawDataReader 专心处理"原始数据"。

详见 [modules/data_reader.md](modules/data_reader.md)。

### ops — Operator 抽象

**职责**：把每种算法（LIO、stitcher、detection）封装成一个可单独测试、可被 pipeline 编排的单元。

**核心接口**：

```cpp
namespace adlabel::ops {

class Operator {
public:
    virtual ~Operator() = default;
    virtual std::string Name() const = 0;
    virtual void Process(std::vector<Frame>* frames) = 0;
};

}  // namespace adlabel::ops
```

**Operator 是 C++ 类，不是 bin 程序**。一个 bin 可以装配多个 Operator 串起来跑，所有 Operator 共享同一个 RawDataReader 实例（见 [decisions/0006-operator-dependency-injection.md](decisions/0006-operator-dependency-injection.md)）。

**典型 Operator**：

```cpp
class LioOperator : public Operator {
public:
    LioOperator(std::shared_ptr<data::RawDataReader> reader, LioConfig config);
    void Process(std::vector<Frame>* frames) override;
    // 写入 frames[i].lio_pose_3d
    // 派生点云写文件，URI 填到 frames[i].compensated_point_cloud_uri
};

class TopdownStitcherOperator : public Operator { /* ... */ };
class DetectionOperator        : public Operator { /* ... */ };
```

**Operator 输出策略**（混合模式）：
- 小数据（pose、检测框元信息）→ 直接修改 Frame 字段
- 大数据（运动补偿点云、可视化图）→ 落盘 + 在 Frame 里加 URI

这种混合是 Frame 索引的现状（参考 `drive/map/proto/frame.proto`）的自然延伸，也避免每个 Operator 都要解决"产出怎么传给下一个"。

详见 [modules/operator.md](modules/operator.md)。

### pipeline — 编排

**职责**：把一组 Operator 顺序装配，按统一接口执行。

```cpp
namespace adlabel::pipeline {

class Pipeline {
public:
    explicit Pipeline(std::shared_ptr<data::RawDataReader> reader);
    void Add(std::unique_ptr<ops::Operator> op);
    void Run(std::vector<Frame>* frames);

private:
    std::shared_ptr<data::RawDataReader> reader_;
    std::vector<std::unique_ptr<ops::Operator>> operators_;
};

}  // namespace adlabel::pipeline
```

Pipeline 持有 reader 的 shared_ptr，构造时把同一个 reader 注入给所有添加进来的 Operator——这样所有 Operator 共享 MCAP 句柄缓存。

第一版 Pipeline **只支持顺序执行**。DAG 调度、并发、跨进程是 Stage 3+ 的事，那时候用 Ray 包一层即可（见 [Evolution Path](#evolution-path)）。

### proto / common — 基础设施

- **proto**：Frame、TripMetadata、各 Operator 的 Config 等 protobuf 定义
- **common**：URI 解析、文件操作、SensorRig（标定加载）、日志/gflags 包装

这两层和 `drive/map/{proto,common}` 直接对应，迁移时几乎可以原样搬过来。

## End-to-End Data Flow

完整的"从 rosbag 到 BEV 底图"链路：

```
                        rosbag (input)
                            │
                            │  [predeposit, Python tool]
                            │  rosbags + mcap library
                            ▼
                        trip_001.mcap
                            │
                            │  [adlabel-indexer]
                            │  McapDataSource → 遍历 → 给每帧分配 fid
                            │  关联 SensorRig 外参
                            ▼
                ┌──────────────────────────┐
                │  trips/001/frames.bin    │
                │  trips/001/trip_meta.pb  │
                └──────────┬───────────────┘
                           │  [adlabel-pipeline]
                           ▼
              ┌─────────────────────────────┐
              │  Load frames.bin             │
              │  Construct RawDataReader     │
              │  Build Pipeline:             │
              │    1. LioOperator            │
              │    2. TopdownStitcherOp      │
              │    3. DetectionOperator      │
              └─────────────┬───────────────┘
                            │
                            │  Operators 通过 RawDataReader
                            │  按需从 MCAP 读原始数据
                            │  Pose 写 Frame 字段
                            │  派生数据写文件 + URI 回填
                            ▼
              ┌─────────────────────────────┐
              │  trips/001/                  │
              │    bev_map.png               │
              │    detections/*.json         │
              │    compensated_clouds/*.pcd  │
              │    frames.bin (updated)      │
              └─────────────────────────────┘
```

注意几处关键性质：

- **MCAP 不会被 indexer 复制或修改**——它是不可变 source of truth
- **Operator 之间通过 frames（in-memory vector）传递**，不是文件——避免反复序列化
- **每个 Operator 失败可以独立重跑**，因为派生数据是文件、Frame 字段是幂等写入

## Process and Deployment Model

ADLabel 在不同规模下的进程模型：

### 小规模：单进程 bin

```
$ adlabel-pipeline --frames trips/001/frames.bin --output trips/001/
```

一个进程，所有 Operator + RawDataReader 共享同一份内存。Stage 1 的形态。

### 中规模：bin + 脚本并行

```bash
parallel -j 16 ./adlabel-pipeline --frames trips/{}/frames.bin ::: trip_list.txt
```

trip 级并行用 `parallel` 或 `multiprocessing.Pool` 拉起多个 bin 进程。每个进程内部还是单进程 pipeline。

### 大规模：Ray 分布式（Stage 3）

```python
import ray

@ray.remote(num_cpus=4)
def run_pipeline(trip_id, mcap_uri):
    return adlabel_py.run_pipeline(trip_id, mcap_uri)  # pybind11 包 C++ 库

futures = [run_pipeline.remote(t.id, t.uri) for t in trips]
ray.get(futures)
```

C++ 库通过 pybind11 暴露 Python 接口，Ray 在外层调度。**C++ 内核零改动**——这是 Operator + Pipeline 抽象在分布式场景的兑现。

为什么是 Ray 而不是 Spark：见 [decisions/0002-distributed-ray.md](decisions/0002-distributed-ray.md)。

简短动因：Ray 在 ML/AI 负载下面比 Spark 更贴合 AV 自动标注场景。[Anyscale 的 Ray vs Spark vs SageMaker 对比](https://www.anyscale.com/blog/offline-batch-inference-comparing-ray-apache-spark-and-sagemaker)显示 Ray Data 在离线图像分类上比 SageMaker Batch Transform 快 ~17x、比 Spark 快 ~2x；[The Streaming Batch Model（arxiv 2501.12407）](https://arxiv.org/abs/2501.12407)论证了在 CPU 解码 + GPU 推理混合负载下 Ray 调度模型的吞吐优势（多模态训练 +31%）。Spark 在 [JVM 边界和粗粒度调度](https://www.anyscale.com/glossary/ray-vs-apache-spark-technical-differences)上对 Python/PyTorch 原生代码不友好，但仍是数据湖 ETL 和 scenario mining 的最佳选择，所以 Stage 4 是两者并存而非二选一（pipeline 调研可参考 [Mixpeek ML pipeline 框架对比](https://mixpeek.com/curated-lists/best-open-source-ml-pipelines)）。

## Evolution Path

ADLabel 不是一次性架构，而是按数据规模和复杂度演进：

```
Stage 1 (current)
  范围: 静态场景，BEV 底图
  规模: 单 trip
  形态: 单进程 pipeline
  关键产出: indexer + LIO + stitcher + 静态检测

Stage 2
  范围: 动态场景，4D 标注
  规模: 单 trip
  形态: 单进程 pipeline + 新增 Operator（轨迹关联、4D box 估计）
  新增模块: ops/tracking, ops/box_estimator

Stage 3
  范围: 大规模批处理
  规模: 1k - 100k trip
  形态: Ray 分布式，trip 级并行
  新增模块: bindings/python (pybind11), 调度脚本

Stage 4
  范围: 数据湖集成
  规模: 100k+ trip
  形态: Spark scenario mining + Ray 标注 + Iceberg/Lance 索引
  新增模块: 上游 ETL（不在 ADLabel 主仓内，外挂）
```

每个 stage 都不需要前一阶段重构。这是分层设计的核心收益。

## Open Questions

记录一些当前还没决定、需要在实现过程中验证的设计点：

1. **派生数据的 URI 命名规范**：是按 `<trip>/<sensor>/<fid>.<ext>` 还是 `<trip>/derived/<operator>/<fid>.<ext>`？前者按传感器组织、后者按 operator 组织。倾向后者（同一个 operator 重跑能整体清空），但需要在第一个 stitcher 跑通后回头确认。

2. **Frame 索引格式**：`frames.bin` 现在计划用 length-prefixed protobuf 序列。规模上 100k trip × 1k frames/trip = 1 亿 Frame 条目时，是否需要换成 Parquet/Lance 这种列存？Stage 1 不解决，但要在 indexer 接口上预留 writer 抽象。

3. **MCAP chunk 大小默认值**：4 MB / 8 MB / 16 MB？需要在真实数据上跑一次基准（顺序读 vs 随机读 vs 压缩比），定一个推荐值放进默认 indexer config。

4. **多 trip 的 Frame 索引合并**：是每个 trip 一份 frames.bin、调用方手工合并，还是提供一个汇总入口？倾向前者（保持 trip 独立性），但调度脚本会需要做 trip 列表管理，待 Stage 3 设计时一起定。

5. **Operator 失败语义**：单帧失败、整 trip 失败、整 pipeline 失败的边界？目前倾向"单帧失败 skip + 计入 RunStatistics、整 trip 失败抛异常"，但需要在第一个端到端跑通后验证是否够用。

## References

本节集中列出本架构借鉴或对照的外部资料，便于深入。文中相关章节已经做了内联引用，这里按主题再聚合一遍。

### 数据格式与存储

- [MCAP — official guides](https://mcap.dev/guides) — Foxglove 推出的标准日志容器格式
- [foxglove/mcap (GitHub)](https://github.com/foxglove/mcap) — C++/Python/Go/Rust 多语言实现
- [Introducing the MCAP File Format](https://foxglove.dev/blog/introducing-the-mcap-file-format) — 设计动机与对比 rosbag/Parquet
- [MCAP Indexing — Monday Morning Haskell](https://mmhaskell.com/blog/2025/12/22/mcap-indexing) — chunk index / message index / summary section 内部结构
- [Foxglove MCAP product page](https://foxglove.dev/product/mcap)

### AV 数据栈与数据湖

- [LanceDB — Unifying the AV/ML Stack](https://www.lancedb.com/blog/unifying-the-av-ml-stack-lancedb) — 现代 AV 数据 lakehouse 架构（原始层 + 索引层 + 训练层）
- [Argoverse 2 API](https://github.com/argoverse/av2-api) — Feather/Arrow 格式的 frame 数据组织
- [nuScenes](https://www.nuscenes.org/) — SQLite metadata + blob 文件格式
- [Awesome-Data-Centric-Autonomous-Driving](https://github.com/LincanLi98/Awesome-Data-Centric-Autonomous-Driving) — 综述：AV 大数据系统、数据挖掘、闭环
- [Efficient Random Access in Columnar Storage（arxiv 2504.15247）](https://arxiv.org/html/2504.15247v1) — Parquet 随机访问优化研究

### 分布式计算与 ML pipeline

- [Anyscale — Ray vs Apache Spark](https://www.anyscale.com/glossary/ray-vs-apache-spark-technical-differences) — 技术差异对比
- [Anyscale — Ray vs Spark vs SageMaker for offline batch inference](https://www.anyscale.com/blog/offline-batch-inference-comparing-ray-apache-spark-and-sagemaker) — 性能基准
- [The Streaming Batch Model（arxiv 2501.12407）](https://arxiv.org/abs/2501.12407) — 异构 CPU/GPU 集群调度模型
- [Holistic Heterogeneous Scheduling for Autonomous Applications（arxiv 2508.09503）](https://arxiv.org/html/2508.09503v1) — 自驾应用多 XPU 细粒度调度
- [Mixpeek — Best Open Source ML Pipeline Frameworks](https://mixpeek.com/curated-lists/best-open-source-ml-pipelines) — 调度器对比

### 自动标注研究

- [SPAM: Efficient Annotations for the Trackers of Tomorrow（arxiv 2404.11426）](https://arxiv.org/html/2404.11426v1) — 自动标注 + 少量人工达到 human-parity（3-20% 标注成本）
- [Automatic Labeling for LiDAR-based Moving Object Segmentation（arxiv 2201.04501）](https://arxiv.org/abs/2201.04501) — 离线 pipeline + 占用栅格 + Kalman 追踪生成训练标签
- [Un(Human)supervised Open-World Pointcloud Labeling（arxiv 2507.20397）](https://arxiv.org/html/2507.20397) — 可扩展点云自动标注，覆盖开放集

### 算法与开源参考

- [Autoware](https://autoware.org/) — 完整自驾开源栈
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) — Lidar-Inertial Odometry 经典实现
- [OpenPCDet](https://github.com/open-mmlab/OpenPCDet) — 点云检测框架，Dataset 抽象的参考实现


