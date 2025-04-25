#!/bin/bash
# スクリプトの絶対パスを取得
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SCRIPTS_DIR="$(cd "$(dirname $SCRIPT_DIR)" && pwd)/scripts"

# リポジトリのスクリプトをコピー
cp -r $SCRIPTS_DIR "$SCRIPT_DIR/scripts"

# システムのアーキテクチャを取得
ARCH=$(uname -m)

# アーキテクチャを判別
if [ "$ARCH" == "x86_64" ]; then
    echo "This system is amd64 (x86_64)."
    docker build -t nedo_ros . &&sudo rm -rf "$SCRIPT_DIR/scripts"
elif [ "$ARCH" == "aarch64" ]; then
    echo "This system is arm64 (aarch64)."
    docker build -f $SCRIPT_DIR/arm64_humble/Dockerfile -t arm64-humble . 
    docker build --build-arg BASE_IMAGE=arm64-humble -t nedo_ros . &&sudo rm -rf "$SCRIPT_DIR/scripts"
else
    echo "This system architecture is not amd64 or arm64: $ARCH"
fi