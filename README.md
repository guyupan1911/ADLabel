# ADLabel

> An open-source auto-labeling toolkit for autonomous driving — covering static BEV maps to 4D dynamic scenes.
>
> 面向自动驾驶的开源自动标注工具链，覆盖静态 BEV 底图到 4D 动态场景。

**Status**: 🚧 Active development — Stage 1 (static BEV) in progress.

---

## What is ADLabel

ADLabel 是一个端到端的自动标注 pipeline，输入车端采集的多传感器原始数据（LiDAR / Camera / IMU / GNSS），输出可用于人工核验或下游训练的 BEV 标注底图与 4D 标注。

设计目标按优先级排序：

1. **从原始数据到可用标注的完整链路**，不依赖任何商用标注平台
2. **解耦于具体数据来源**：rosbag、MCAP、KITTI、nuScenes 通过统一接口接入
3. **分布式友好**：单机跑通的 pipeline 无需重写即可在 Ray 集群上扩展
4. **工程基础设施可复用**：indexer / data reader / operator 抽象在静态和动态阶段共用

## Pipeline Overview

```
 ┌─────────────┐                                                 ┌──────────────┐
 │ rosbag/mcap │                                                 │ BEV 底图      │
 │ kitti/nusc  │                                                 │ 静态语义      │
 └──────┬──────┘                                                 │ 4D 动态轨迹   │
        │                                                        └──────▲───────┘
        ▼                                                               │
 ┌──────────────┐    ┌────────────────┐    ┌─────────────────────┐     │
 │   Indexer    │───►│  Frame 索引    │───►│  Operator Pipeline  │─────┘
 │              │    │  + 原始数据    │    │  LIO → Stitcher →   │
 │              │    │  (MCAP/blobs)  │    │  Detection → ...    │
 └──────────────┘    └────────────────┘    └─────────────────────┘
                            ▲                       │
                            │   按 Frame 取数据      │
                            └───────────────────────┘
                                RawDataReader
```

每个模块的详细职责见 [docs/architecture.md](docs/architecture.md)。

## Quickstart

> ⚠️ Stage 1 还在搭建中，命令以最终发布为准。

```bash
# 1. 启动开发容器
./scripts/dev.sh

# 2. 容器内编译
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -GNinja
ninja

# 3. 把 rosbag 转成 MCAP（一次性预处理）
adlabel-bag2mcap --input trip_001.bag --output trip_001.mcap

# 4. 生成 Frame 索引
adlabel-indexer --source-type mcap \
                --input trip_001.mcap \
                --calib  /path/to/calib_dir \
                --output trips/001/

# 5. 跑完整 pipeline，输出 BEV 底图
adlabel-pipeline --frames trips/001/frames.bin \
                 --output trips/001/bev_map.png
```

## Architecture at a Glance

| Layer | 模块 | 职责 |
|---|---|---|
| 数据接入 | `adlabel/io` | DataSource 抽象 + rosbag/MCAP/KITTI/nuScenes 实现 |
| 索引生成 | `adlabel/indexer` | 遍历数据源，输出 Frame 索引 + trip 元数据 |
| 数据访问 | `adlabel/data` | RawDataReader：按 Frame 取类型化数据，自动分派后端 |
| 算法层 | `adlabel/ops` | Operator 抽象 + LIO / Stitcher / Detection 等实现 |
| 编排层 | `adlabel/pipeline` | Pipeline 装配 + 顺序执行 |
| Proto | `adlabel/proto` | Frame、TripMetadata、Config 等数据结构 |

完整架构见 [docs/architecture.md](docs/architecture.md)。

## Roadmap

| Stage | 范围 | 状态 |
|---|---|---|
| **Stage 1** | 静态场景：LIO + 多帧 BEV 拼接 + 静态目标检测 → BEV 底图 | 🚧 In progress |
| Stage 2 | 动态场景：跨帧轨迹关联 + 4D bounding box | Planned |
| Stage 3 | 大规模批处理：Ray 分布式调度，trip 级并行 | Planned |
| Stage 4 | 数据湖集成：trip 元数据入 Iceberg / Lance，scenario mining | Planned |

## Design Decisions

ADLabel 的关键技术选型记录在 [docs/decisions/](docs/decisions/) 下，每个决策一份 ADR：

- **MCAP** 作为统一的原始数据存储格式（替代每帧落盘）
- **CMake** 而非 Bazel 管理工程
- 彻底**去 ROS 依赖**，IO 通过 MCAP 解耦
- **Operator 依赖注入**，Pipeline 内共享 RawDataReader
- 未来分布式选 **Ray** 而非 Spark

## Tech Stack

- C++17, CMake ≥ 3.16, Ninja
- Eigen3, PCL, OpenCV, Ceres, Boost
- Protobuf, gflags, glog, nlohmann_json
- MCAP (Foxglove)
- Docker + docker-compose 管理开发环境

## Project Structure

```
ADLabel/
├── adlabel/             # 核心库
│   ├── common/
│   ├── proto/
│   ├── io/              # DataSource (rosbag/mcap/kitti)
│   ├── indexer/         # Frame 索引生成
│   ├── data/            # RawDataReader
│   ├── ops/             # Operator 实现
│   └── pipeline/
├── apps/                # bin 程序入口
├── configs/
├── docker/
├── scripts/
├── tests/
├── docs/
│   ├── architecture.md
│   └── decisions/       # ADRs
├── cmake/
├── CMakeLists.txt
└── CMakePresets.json
```

## Contributing

This is primarily a personal portfolio project. Issues and discussion are welcome,
but active feature development is driven by the roadmap above.

## License

Apache License 2.0 — see [LICENSE](LICENSE).

## Acknowledgements

ADLabel 在设计上参考和借鉴了以下工作：

**数据格式 / 存储**
- [Foxglove MCAP](https://mcap.dev/guides) — 标准化日志容器格式
- [MCAP File Format Introduction](https://foxglove.dev/blog/introducing-the-mcap-file-format) / [Indexing internals](https://mmhaskell.com/blog/2025/12/22/mcap-indexing)
- [LanceDB AV/ML Stack](https://www.lancedb.com/blog/unifying-the-av-ml-stack-lancedb) — 现代 lakehouse 架构

**AV 数据栈**
- [Argoverse 2](https://github.com/argoverse/av2-api) / [nuScenes](https://www.nuscenes.org/) — 数据集格式参考
- [Awesome-Data-Centric-Autonomous-Driving](https://github.com/LincanLi98/Awesome-Data-Centric-Autonomous-Driving) — 行业综述

**算法实现**
- [Autoware](https://autoware.org/) — 自驾开源栈
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) — Lidar-Inertial Odometry
- [OpenPCDet](https://github.com/open-mmlab/OpenPCDet) — Dataset 抽象与点云检测

**分布式计算（未来 Stage 3+）**
- [Ray](https://www.anyscale.com/blog/offline-batch-inference-comparing-ray-apache-spark-and-sagemaker) — 异构集群批处理
- [Streaming Batch Model（arxiv 2501.12407）](https://arxiv.org/abs/2501.12407) — CPU/GPU 混合调度

**自动标注研究**
- [SPAM（arxiv 2404.11426）](https://arxiv.org/html/2404.11426v1) — 自动标注 + 少量人工
- [LiDAR Auto-labeling（arxiv 2201.04501）](https://arxiv.org/abs/2201.04501) — 离线 pipeline 经典做法

完整的引用与论证记录见 [docs/architecture.md#references](docs/architecture.md#references) 和 [docs/decisions/](docs/decisions/)。
