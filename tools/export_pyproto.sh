#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUT_DIR="${1:-${REPO_ROOT}/pyproto}"

cd "${REPO_ROOT}"

mapfile -t PROTO_FILES < <(
  find . \
    -path './bazel-*' -prune -o \
    -path './.cache' -prune -o \
    -path './pyproto' -prune -o \
    -name '*.proto' -print | sort
)

if [[ ${#PROTO_FILES[@]} -eq 0 ]]; then
  echo "No proto files found under ${REPO_ROOT}" >&2
  exit 1
fi

mkdir -p "${OUT_DIR}"

if python3 -m grpc_tools.protoc --version >/dev/null 2>&1; then
  python3 -m grpc_tools.protoc -I. --python_out="${OUT_DIR}" "${PROTO_FILES[@]}"
elif command -v protoc >/dev/null 2>&1; then
  protoc -I. --python_out="${OUT_DIR}" "${PROTO_FILES[@]}"
else
  cat >&2 <<'EOF'
Neither grpc_tools.protoc nor protoc was found.

Install one of:
  sudo apt-get install protobuf-compiler python3-protobuf
  python3 -m pip install grpcio-tools protobuf
EOF
  exit 1
fi

echo "Exported ${#PROTO_FILES[@]} Python proto files to ${OUT_DIR}"
echo "Use: export PYTHONPATH=${OUT_DIR}:\$PYTHONPATH"
