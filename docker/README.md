# ADLabel Dev Image

Single image `adlabel-dev` covering everything needed to compile, debug, and
run preprocessing tools for ADLabel.

## Quickstart

```bash
# First run: builds the image (~10 min) and starts the container.
./devtools/dev_start.sh

# Open a shell inside the running container.
./devtools/dev_into.sh

# Inside the container:
cd /workspace/ADLabel
mkdir -p build && cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Debug
ninja
ctest --output-on-failure
```

The container stays running in the background after `dev_start.sh`. Run
`dev_into.sh` from any terminal to attach a new shell.

## What's inside

**Toolchain**: gcc 11, clang 14, cmake 3.22+, ninja, ccache (auto-enabled via PATH).

**Core C++ libraries** (apt, Ubuntu 22.04 versions):
- Eigen 3.4, Boost 1.74
- PCL 1.12, OpenCV 4.5
- Protobuf 3.12, glog, gflags, nlohmann_json
- Ceres 2.0, TBB

GTSAM is intentionally **not** in the image yet — it'll be added when the
LIO operator lands (Stage 1 later half). Ubuntu 22.04 has no
`libgtsam-dev` package; we'll add it via the borglab PPA or build from
source at that point.

**Python tools** (system pip):
- `rosbags` — read ROS1/ROS2 bags without ROS installed
- `mcap`, `mcap-ros1-support` — write MCAP files
- `pre-commit`, `cmake-format`, `numpy`, `matplotlib`

**Debug tooling**: gdb, lldb, valgrind, clang-format, clang-tidy.

Libraries pulled at CMake configure time (not in the image): MCAP C++,
GoogleTest. See top-level `cmake/Dependencies.cmake` once that lands.

## Layout

```
docker/
├── Dockerfile              # single image definition
├── docker-compose.yml      # container service + volumes
└── README.md               # this file
```

## Volumes

| Host path             | Container path                       | Purpose                                       |
|-----------------------|--------------------------------------|-----------------------------------------------|
| `..` (project root)   | `/workspace/ADLabel`                 | Live source mount (build/, .cache/ live here) |

The build directory and ccache cache live under the repo bind mount
(`build/` and `.cache/ccache/`, both gitignored). They survive container
restarts because the bind mount itself is persistent, and they remain
visible from the host for inspection.

## UID/GID mapping

`dev_start.sh` passes the host user's UID/GID into the build args
`USER_UID` / `USER_GID`. The container runs as `dev`, so files created
inside the container appear as the host user outside — no permission
fighting with VS Code or git.

## Common operations

**Rebuild the image after editing `Dockerfile`**:
```bash
./devtools/dev_start.sh    # idempotent, picks up Dockerfile changes
```

**Stop the container**:
```bash
docker compose -f docker/docker-compose.yml stop
```

**Tear down the container** (build/ and ccache survive on host):
```bash
docker compose -f docker/docker-compose.yml down
```

**Run a one-off command without entering interactively**:
```bash
docker exec adlabel-dev bash -c 'cd /workspace/ADLabel/build && ninja'
```

## Version history

| Version | Date       | Changes                                                           |
|---------|------------|-------------------------------------------------------------------|
| 0.1.0   | 2026-05-23 | Initial: Ubuntu 22.04, PCL/OpenCV/Eigen, Ceres 2.0, TBB           |

Bump `ADLABEL_VERSION` (env var or compose default) when the Dockerfile
changes so old/new images coexist cleanly.

## Notes

- `network_mode: host` is used for development convenience (debuggers,
  local services, future bag replay). Remove it if running on Docker
  Desktop or in environments where host networking isn't supported.
- The container runs `sleep infinity` as PID 1 so multiple shells can
  attach via `docker exec` without racing on a single TTY.
