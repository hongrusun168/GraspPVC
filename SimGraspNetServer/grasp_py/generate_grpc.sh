#!/bin/bash
# 构建 gRPC 代码

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "正在激活 conda 环境..."
conda activate grasp_grpc 2>/dev/null || source "$(conda info --base)/etc/profile.d/conda.sh"

echo "正在生成 gRPC 代码..."
python -m grpc_tools.protoc \
    -I./proto \
    --python_out=./generated \
    --grpc_python_out=./generated \
    ./proto/grasp.proto

# 修复生成的 import
sed -i 's/import grasp_pb2/from generated import grasp_pb2/' generated/grasp_pb2_grpc.py
sed -i 's/from grasp_pb2 import/from generated import grasp_pb2 import/' generated/grasp_pb2_grpc.py

echo "完成! 生成的代码在 ./generated 目录"
