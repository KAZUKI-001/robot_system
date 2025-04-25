#!/bin/bash

export XAUTHORITY=$HOME/.Xauthority

xhost +local:root

# スクリプトの絶対パスを取得
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# リポジトリの絶対パスを取得
WS_DIR="$(cd "$(dirname $SCRIPT_DIR)" && pwd)"
# 使用するDockerイメージ名
IMAGE_NAME="nedo_ros"
# コンテナの名前（ユニークに設定）
CONTAINER_NAME="nedo_ros_container"

# 自動検出するデバイスパターン（udevリンクは除外）
DEVICE_PATTERNS=(
    "/dev/ttyUSB*"
    "/dev/video*"
)

# デバイスマウント用のオプションを生成
DEVICE_OPTIONS=""
for PATTERN in "${DEVICE_PATTERNS[@]}"; do
    for DEVICE in $PATTERN; do
        if [ -e "$DEVICE" ]; then
            DEVICE_OPTIONS+="--device=$DEVICE "
        fi
    done
done

# udevシンボリックリンクをbind mountで明示的に追加（例: /dev/front_camera）
SYMLINKS=(/dev/front_camera /dev/left_camera /dev/right_camera)
for LINK in "${SYMLINKS[@]}"; do
    if [ -e "$LINK" ]; then
        DEVICE_OPTIONS+="--volume=$LINK:$LINK "
    fi
done

# 実行中のコンテナがあるか確認
RUNNING_CONTAINER=$(docker ps --filter "name=$CONTAINER_NAME" --format "{{.ID}}")

if [ -z "$RUNNING_CONTAINER" ]; then
    # コンテナが起動していない場合、新しいコンテナを起動
    echo "No running container found. Starting a new container..."
    sudo docker run -it --rm --net=host --ipc=host --shm-size=10gb -v "$WS_DIR:/root/NEDO_ROS" \
        --env="DISPLAY=$DISPLAY" \
        --volume="${XAUTHORITY}:/root/.Xauthority" \
        --privileged=true \
        $DEVICE_OPTIONS \
        --name "$CONTAINER_NAME" \
        "$IMAGE_NAME"
else
    # 既存のコンテナがある場合、新しいセッションを作成
    echo "Attaching to existing container: $RUNNING_CONTAINER"
    sudo docker exec -it "$RUNNING_CONTAINER" bash
fi
