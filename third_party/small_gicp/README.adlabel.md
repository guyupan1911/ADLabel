# small_gicp vendored core

- Upstream: https://github.com/koide3/small_gicp
- Version: v1.0.1
- Commit: 57c1106daf83c2c79ee0c58a9c7ed0032298ff4e
- Vendored date: 2026-07-21
- License: MIT; see `LICENSE`
- Language requirement: C++17

## Included

The header-only registration core under `include/small_gicp`:

- `ann`
- `factors`
- `points`
- `registration`
- `util`

## Excluded

- Compiled helper API (`registration_helper.hpp` and its `.cpp` files)
- Benchmark and example programs
- Python bindings
- ROS adapters
- PCL registration facade and PCL traits adapters
- Build, CI, documentation, and test infrastructure

The included upstream headers are unmodified. The only packaging change is
selecting the core subset listed above. ADLabel-specific adapters and
