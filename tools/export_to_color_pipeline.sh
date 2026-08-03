#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ADLABEL_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONTAINER_NAME="adlabel-dev"
CONTAINER_REPO_ROOT="/workspace/ADLabel"
BAZEL_TARGET="//mapping/tools:multi_trips_fusion_cli"
SKIP_BUILD=0
ALLOW_DIRTY=0
FORCE=0
DRY_RUN=0
COLOR_PIPELINE_ROOT=""

RUNTIME_DIR_NAME="adlabel_multi_trips_fusion"
INDEXER_DIR_NAME="adlabel_indexer"
RUNTIME_ARCHIVE_NAME="adlabel_multi_trips_fusion-linux-x86_64.tar.gz"
LFS_RULE="third_party/adlabel_multi_trips_fusion/*.tar.gz filter=lfs diff=lfs merge=lfs -text"

PRIVATE_LIBRARIES=(
  "libgtsam.so.4"
  "libmetis-gtsam.so"
  "libtbb.so.2"
  "libGeographic.so.19"
)

INDEXER_FILES=(
  "indexer/plus_indexer.py"
  "mapping/protos/pose.proto"
  "mapping/protos/camera_calibration.proto"
  "mapping/protos/frame.proto"
  "mapping/protos/frame_pair.proto"
  "mapping/tools/plot_refined_pose_trajectories.py"
  "tools/export_pyproto.sh"
)

usage() {
  cat <<'EOF'
Usage:
  tools/export_to_color_pipeline.sh [options] COLOR_PIPELINE_ROOT

Build and export ADLabel's multi-trip fusion runtime and Python indexer tools
into the matching third_party directories of a color_pipeline checkout.

Options:
  --container NAME          Build container. Default: adlabel-dev
  --container-repo-root DIR ADLabel path in the container.
                            Default: /workspace/ADLabel
  --skip-build              Reuse the existing Bazel output.
  --allow-dirty             Allow export from a dirty ADLabel worktree.
  --force                   Overwrite dirty color_pipeline target paths.
  --dry-run                 Validate inputs and print planned actions only.
  -h, --help                Show this help.

Example:
  tools/export_to_color_pipeline.sh /home/guyu/projects/color_pipeline
EOF
}

log() {
  echo "[adlabel-export] $*"
}

die() {
  echo "[adlabel-export] ERROR: $*" >&2
  exit 1
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "required command not found: $1"
}

require_file() {
  [[ -f "$1" ]] || die "$2 does not exist: $1"
  [[ -s "$1" ]] || die "$2 is empty: $1"
}

require_dir() {
  [[ -d "$1" ]] || die "$2 does not exist: $1"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --container)
      [[ $# -ge 2 ]] || die "--container requires a value"
      CONTAINER_NAME="$2"
      shift 2
      ;;
    --container-repo-root)
      [[ $# -ge 2 ]] || die "--container-repo-root requires a value"
      CONTAINER_REPO_ROOT="$2"
      shift 2
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --allow-dirty)
      ALLOW_DIRTY=1
      shift
      ;;
    --force)
      FORCE=1
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --*)
      die "unknown option: $1"
      ;;
    *)
      [[ -z "${COLOR_PIPELINE_ROOT}" ]] || die "only one COLOR_PIPELINE_ROOT is allowed"
      COLOR_PIPELINE_ROOT="$1"
      shift
      ;;
  esac
done

[[ -n "${COLOR_PIPELINE_ROOT}" ]] || {
  usage >&2
  exit 2
}

require_command git
require_command docker
require_command tar
require_command sha256sum
require_command python3

git -C "${ADLABEL_ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1 || \
  die "ADLabel root is not a git worktree: ${ADLABEL_ROOT}"
COLOR_PIPELINE_ROOT="$(realpath "${COLOR_PIPELINE_ROOT}")"
git -C "${COLOR_PIPELINE_ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1 || \
  die "color_pipeline root is not a git worktree: ${COLOR_PIPELINE_ROOT}"
require_file \
  "${COLOR_PIPELINE_ROOT}/docker/argo/dockerfiles/bev_all_in_one.Dockerfile" \
  "color_pipeline bev_all_in_one Dockerfile"
require_dir \
  "${COLOR_PIPELINE_ROOT}/manifest_pipeline/wrappers/multi_trips_fusion" \
  "color_pipeline multi_trips_fusion wrapper"
require_dir "${COLOR_PIPELINE_ROOT}/third_party" "color_pipeline third_party directory"

for relative_path in "${INDEXER_FILES[@]}"; do
  require_file "${ADLABEL_ROOT}/${relative_path}" "ADLabel export source ${relative_path}"
done
require_file \
  "${ADLABEL_ROOT}/mapping/loop_closure/config/ndt_d2d_config.pb.txt" \
  "ADLabel NDT-D2D config"

ADLABEL_COMMIT="$(git -C "${ADLABEL_ROOT}" rev-parse HEAD)"
ADLABEL_STATUS="$(git -C "${ADLABEL_ROOT}" status --porcelain --untracked-files=all)"
ADLABEL_DIRTY=0
if [[ -n "${ADLABEL_STATUS}" ]]; then
  ADLABEL_DIRTY=1
  if [[ "${ALLOW_DIRTY}" -ne 1 ]]; then
    echo "${ADLABEL_STATUS}" >&2
    die "ADLabel worktree is dirty; commit/stash changes or pass --allow-dirty"
  fi
  log "WARNING: exporting from a dirty ADLabel worktree"
fi

COLOR_TARGET_STATUS="$(
  git -C "${COLOR_PIPELINE_ROOT}" status --porcelain --untracked-files=all -- \
    .gitattributes \
    "third_party/${INDEXER_DIR_NAME}" \
    "third_party/${RUNTIME_DIR_NAME}"
)"
if [[ -n "${COLOR_TARGET_STATUS}" && "${FORCE}" -ne 1 ]]; then
  echo "${COLOR_TARGET_STATUS}" >&2
  die "color_pipeline export targets are dirty; preserve changes or pass --force"
fi
if [[ -n "${COLOR_TARGET_STATUS}" ]]; then
  log "WARNING: --force will replace dirty color_pipeline export targets"
fi

COMMIT_LABEL="${ADLABEL_COMMIT}"
if [[ "${ADLABEL_DIRTY}" -eq 1 ]]; then
  COMMIT_LABEL="${ADLABEL_COMMIT}-dirty"
fi

log "ADLabel root: ${ADLABEL_ROOT}"
log "ADLabel commit: ${COMMIT_LABEL}"
log "color_pipeline root: ${COLOR_PIPELINE_ROOT}"
log "build container: ${CONTAINER_NAME}:${CONTAINER_REPO_ROOT}"

if [[ "${DRY_RUN}" -eq 1 ]]; then
  if [[ "${SKIP_BUILD}" -eq 1 ]]; then
    log "dry-run: would reuse ${BAZEL_TARGET} from ${CONTAINER_NAME}"
  else
    log "dry-run: would build ${BAZEL_TARGET} with bazel -c opt"
  fi
  log "dry-run: would update third_party/${INDEXER_DIR_NAME}"
  log "dry-run: would update third_party/${RUNTIME_DIR_NAME}/${RUNTIME_ARCHIVE_NAME}"
  exit 0
fi

docker inspect "${CONTAINER_NAME}" >/dev/null 2>&1 || \
  die "build container does not exist: ${CONTAINER_NAME}"
CONTAINER_RUNNING="$(docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}")"
[[ "${CONTAINER_RUNNING}" == "true" ]] || die "build container is not running: ${CONTAINER_NAME}"

CONTAINER_COMMIT="$(
  docker exec --workdir "${CONTAINER_REPO_ROOT}" "${CONTAINER_NAME}" \
    git rev-parse HEAD
)"
[[ "${CONTAINER_COMMIT}" == "${ADLABEL_COMMIT}" ]] || \
  die "host/container ADLabel commits differ: host=${ADLABEL_COMMIT}, container=${CONTAINER_COMMIT}"

EXPORT_STAGE="$(mktemp -d "${ADLABEL_ROOT}/.color_pipeline_export.XXXXXX")"
TARGET_INDEXER_NEW=""
TARGET_RUNTIME_NEW=""
TARGET_INDEXER_BACKUP=""
TARGET_RUNTIME_BACKUP=""
TARGET_INDEXER_HAD_ORIGINAL=0
TARGET_RUNTIME_HAD_ORIGINAL=0
INSTALL_STARTED=0
INSTALL_COMPLETE=0

cleanup() {
  local exit_code=$?
  if [[ "${INSTALL_COMPLETE}" -ne 1 && "${INSTALL_STARTED}" -eq 1 ]]; then
    if [[ -e "${TARGET_INDEXER_BACKUP}" ]]; then
      rm -rf -- "${COLOR_PIPELINE_ROOT}/third_party/${INDEXER_DIR_NAME}"
      mv -- "${TARGET_INDEXER_BACKUP}" \
        "${COLOR_PIPELINE_ROOT}/third_party/${INDEXER_DIR_NAME}"
    elif [[ "${TARGET_INDEXER_HAD_ORIGINAL}" -eq 0 ]]; then
      rm -rf -- "${COLOR_PIPELINE_ROOT}/third_party/${INDEXER_DIR_NAME}"
    fi
    if [[ -e "${TARGET_RUNTIME_BACKUP}" ]]; then
      rm -rf -- "${COLOR_PIPELINE_ROOT}/third_party/${RUNTIME_DIR_NAME}"
      mv -- "${TARGET_RUNTIME_BACKUP}" \
        "${COLOR_PIPELINE_ROOT}/third_party/${RUNTIME_DIR_NAME}"
    elif [[ "${TARGET_RUNTIME_HAD_ORIGINAL}" -eq 0 ]]; then
      rm -rf -- "${COLOR_PIPELINE_ROOT}/third_party/${RUNTIME_DIR_NAME}"
    fi
  fi
  [[ -z "${TARGET_INDEXER_NEW}" ]] || rm -rf -- "${TARGET_INDEXER_NEW}"
  [[ -z "${TARGET_RUNTIME_NEW}" ]] || rm -rf -- "${TARGET_RUNTIME_NEW}"
  rm -rf -- "${EXPORT_STAGE}"
  exit "${exit_code}"
}
trap cleanup EXIT

chmod 0755 "${EXPORT_STAGE}"
CONTAINER_STAGE="${CONTAINER_REPO_ROOT}/$(basename "${EXPORT_STAGE}")"
HOST_RUNTIME_STAGE="${EXPORT_STAGE}/runtime"
CONTAINER_RUNTIME_STAGE="${CONTAINER_STAGE}/runtime"
HOST_EXPORT_ROOT="${EXPORT_STAGE}/color_pipeline"
HOST_INDEXER_EXPORT="${HOST_EXPORT_ROOT}/third_party/${INDEXER_DIR_NAME}"
HOST_RUNTIME_EXPORT="${HOST_EXPORT_ROOT}/third_party/${RUNTIME_DIR_NAME}"

if [[ "${SKIP_BUILD}" -ne 1 ]]; then
  log "building ${BAZEL_TARGET}"
  docker exec --workdir "${CONTAINER_REPO_ROOT}" "${CONTAINER_NAME}" \
    bazel build -c opt "${BAZEL_TARGET}"
else
  log "reusing existing Bazel output"
fi

BAZEL_OUTPUT="$(
  docker exec --workdir "${CONTAINER_REPO_ROOT}" "${CONTAINER_NAME}" \
    bazel cquery -c opt --output=files "${BAZEL_TARGET}"
)"
BINARY_PATH="$(printf '%s\n' "${BAZEL_OUTPUT}" | tail -n 1)"
[[ -n "${BINARY_PATH}" ]] || die "bazel cquery returned no output for ${BAZEL_TARGET}"
if [[ "${BINARY_PATH}" != /* ]]; then
  BINARY_PATH="${CONTAINER_REPO_ROOT}/${BINARY_PATH}"
fi
docker exec "${CONTAINER_NAME}" test -x "${BINARY_PATH}" || \
  die "Bazel binary is missing or not executable: ${BINARY_PATH}"

docker exec "${CONTAINER_NAME}" mkdir -p \
  "${CONTAINER_RUNTIME_STAGE}/bin" \
  "${CONTAINER_RUNTIME_STAGE}/config" \
  "${CONTAINER_RUNTIME_STAGE}/lib"
docker exec "${CONTAINER_NAME}" cp -L -- "${BINARY_PATH}" \
  "${CONTAINER_RUNTIME_STAGE}/bin/multi_trips_fusion_cli"
docker exec "${CONTAINER_NAME}" cp -L -- \
  "${CONTAINER_REPO_ROOT}/mapping/loop_closure/config/ndt_d2d_config.pb.txt" \
  "${CONTAINER_RUNTIME_STAGE}/config/ndt_d2d_config.pb.txt"

LDD_OUTPUT="$(docker exec "${CONTAINER_NAME}" ldd "${BINARY_PATH}")"
if grep -q "not found" <<<"${LDD_OUTPUT}"; then
  echo "${LDD_OUTPUT}" >&2
  die "built binary has unresolved dependencies"
fi

for soname in "${PRIVATE_LIBRARIES[@]}"; do
  dependency_path="$(
    printf '%s\n' "${LDD_OUTPUT}" | \
      awk -v wanted="${soname}" '$1 == wanted {print $3; exit}'
  )"
  [[ -n "${dependency_path}" && "${dependency_path}" == /* ]] || \
    die "failed to resolve private dependency from ldd: ${soname}"
  log "bundle library: ${soname} <- ${dependency_path}"
  docker exec "${CONTAINER_NAME}" cp -L -- "${dependency_path}" \
    "${CONTAINER_RUNTIME_STAGE}/lib/${soname}"
done

STAGED_LDD_OUTPUT="$(
  docker exec "${CONTAINER_NAME}" env \
    "LD_LIBRARY_PATH=${CONTAINER_RUNTIME_STAGE}/lib" \
    ldd "${CONTAINER_RUNTIME_STAGE}/bin/multi_trips_fusion_cli"
)"
if grep -q "not found" <<<"${STAGED_LDD_OUTPUT}"; then
  echo "${STAGED_LDD_OUTPUT}" >&2
  die "staged runtime has unresolved dependencies"
fi
for soname in "${PRIVATE_LIBRARIES[@]}"; do
  expected_path="${CONTAINER_RUNTIME_STAGE}/lib/${soname}"
  if ! printf '%s\n' "${STAGED_LDD_OUTPUT}" | grep -Fq "${expected_path}"; then
    echo "${STAGED_LDD_OUTPUT}" >&2
    die "staged binary does not resolve ${soname} from the private runtime directory"
  fi
done

mkdir -p "${HOST_INDEXER_EXPORT}" "${HOST_RUNTIME_EXPORT}"
for relative_path in "${INDEXER_FILES[@]}"; do
  install -D -m 0644 \
    "${ADLABEL_ROOT}/${relative_path}" \
    "${HOST_INDEXER_EXPORT}/${relative_path}"
done
chmod 0755 \
  "${HOST_INDEXER_EXPORT}/indexer/plus_indexer.py" \
  "${HOST_INDEXER_EXPORT}/mapping/tools/plot_refined_pose_trajectories.py" \
  "${HOST_INDEXER_EXPORT}/tools/export_pyproto.sh"
printf '%s\n' "${COMMIT_LABEL}" > "${HOST_INDEXER_EXPORT}/ADLABEL_COMMIT"

for python_file in \
  "${HOST_INDEXER_EXPORT}/indexer/plus_indexer.py" \
  "${HOST_INDEXER_EXPORT}/mapping/tools/plot_refined_pose_trajectories.py"; do
  python3 -c \
    'import pathlib, sys; path = pathlib.Path(sys.argv[1]); compile(path.read_text(encoding="utf-8"), str(path), "exec")' \
    "${python_file}"
done

SOURCE_DATE_EPOCH="$(git -C "${ADLABEL_ROOT}" show -s --format=%ct "${ADLABEL_COMMIT}")"
tar \
  --sort=name \
  --mtime="@${SOURCE_DATE_EPOCH}" \
  --owner=0 \
  --group=0 \
  --numeric-owner \
  -C "${HOST_RUNTIME_STAGE}" \
  -czf "${HOST_RUNTIME_EXPORT}/${RUNTIME_ARCHIVE_NAME}" \
  .
printf '%s\n' "${COMMIT_LABEL}" > "${HOST_RUNTIME_EXPORT}/ADLABEL_COMMIT"
(
  cd "${HOST_RUNTIME_EXPORT}"
  sha256sum "${RUNTIME_ARCHIVE_NAME}" > SHA256SUMS
)

{
  printf '%s\n' '# ADLabel multi-trips fusion runtime'
  printf '\n'
  printf '%s\n' 'Generated by `ADLabel/tools/export_to_color_pipeline.sh`.'
  printf '\n'
  printf '%s\n' 'Contents:'
  printf '\n'
  printf '%s\n' '- `bin/multi_trips_fusion_cli`'
  printf '%s\n' '- `config/ndt_d2d_config.pb.txt`'
  printf '%s\n' '- private GTSAM, METIS, legacy TBB, and GeographicLib libraries'
  printf '\n'
  printf '%s\n' 'Build target:'
  printf '\n'
  printf '%s\n' '```bash'
  printf '%s\n' 'bazel build -c opt //mapping/tools:multi_trips_fusion_cli'
  printf '%s\n' '```'
  printf '\n'
  printf '%s\n' '`ADLABEL_COMMIT` records the exported source revision.'
} > "${HOST_RUNTIME_EXPORT}/README.md"

{
  printf 'commit=%s\n' "${ADLABEL_COMMIT}"
  printf 'dirty=%s\n' "${ADLABEL_DIRTY}"
  printf 'container=%s\n' "${CONTAINER_NAME}"
  printf 'container_repo_root=%s\n' "${CONTAINER_REPO_ROOT}"
  printf 'bazel_target=%s\n' "${BAZEL_TARGET}"
  printf 'exported_at_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
} > "${HOST_RUNTIME_EXPORT}/BUILD_INFO"

ARCHIVE_LIST="$(tar -tzf "${HOST_RUNTIME_EXPORT}/${RUNTIME_ARCHIVE_NAME}")"
REQUIRED_ARCHIVE_ENTRIES=(
  "./bin/multi_trips_fusion_cli"
  "./config/ndt_d2d_config.pb.txt"
  "./lib/libgtsam.so.4"
  "./lib/libmetis-gtsam.so"
  "./lib/libtbb.so.2"
  "./lib/libGeographic.so.19"
)
for archive_entry in "${REQUIRED_ARCHIVE_ENTRIES[@]}"; do
  if ! printf '%s\n' "${ARCHIVE_LIST}" | grep -Fxq "${archive_entry}"; then
    die "runtime archive is missing ${archive_entry}"
  fi
done

TARGET_THIRD_PARTY="${COLOR_PIPELINE_ROOT}/third_party"
TARGET_INDEXER="${TARGET_THIRD_PARTY}/${INDEXER_DIR_NAME}"
TARGET_RUNTIME="${TARGET_THIRD_PARTY}/${RUNTIME_DIR_NAME}"
TARGET_INDEXER_NEW="${TARGET_THIRD_PARTY}/.${INDEXER_DIR_NAME}.new.$$"
TARGET_RUNTIME_NEW="${TARGET_THIRD_PARTY}/.${RUNTIME_DIR_NAME}.new.$$"
TARGET_INDEXER_BACKUP="${TARGET_THIRD_PARTY}/.${INDEXER_DIR_NAME}.backup.$$"
TARGET_RUNTIME_BACKUP="${TARGET_THIRD_PARTY}/.${RUNTIME_DIR_NAME}.backup.$$"

cp -a -- "${HOST_INDEXER_EXPORT}" "${TARGET_INDEXER_NEW}"
cp -a -- "${HOST_RUNTIME_EXPORT}" "${TARGET_RUNTIME_NEW}"

if [[ -e "${TARGET_INDEXER}" ]]; then
  TARGET_INDEXER_HAD_ORIGINAL=1
fi
if [[ -e "${TARGET_RUNTIME}" ]]; then
  TARGET_RUNTIME_HAD_ORIGINAL=1
fi
INSTALL_STARTED=1
if [[ "${TARGET_INDEXER_HAD_ORIGINAL}" -eq 1 ]]; then
  mv -- "${TARGET_INDEXER}" "${TARGET_INDEXER_BACKUP}"
fi
if [[ "${TARGET_RUNTIME_HAD_ORIGINAL}" -eq 1 ]]; then
  mv -- "${TARGET_RUNTIME}" "${TARGET_RUNTIME_BACKUP}"
fi
mv -- "${TARGET_INDEXER_NEW}" "${TARGET_INDEXER}"
TARGET_INDEXER_NEW=""
mv -- "${TARGET_RUNTIME_NEW}" "${TARGET_RUNTIME}"
TARGET_RUNTIME_NEW=""

GITATTRIBUTES_PATH="${COLOR_PIPELINE_ROOT}/.gitattributes"
touch "${GITATTRIBUTES_PATH}"
if ! grep -Fqx "${LFS_RULE}" "${GITATTRIBUTES_PATH}"; then
  printf '%s\n' "${LFS_RULE}" >> "${GITATTRIBUTES_PATH}"
  log "added Git LFS rule to .gitattributes"
fi

rm -rf -- "${TARGET_INDEXER_BACKUP}" "${TARGET_RUNTIME_BACKUP}"
TARGET_INDEXER_BACKUP=""
TARGET_RUNTIME_BACKUP=""
INSTALL_COMPLETE=1

log "export complete"
log "indexer: ${TARGET_INDEXER}"
log "runtime: ${TARGET_RUNTIME}/${RUNTIME_ARCHIVE_NAME}"
log "runtime sha256: $(cut -d' ' -f1 "${TARGET_RUNTIME}/SHA256SUMS")"
log "next: rebuild the color_pipeline bev_all_in_one image"
