#!/bin/bash

echo "----------------------------"
echo " Setting up LIDAR Ethernet"
echo "----------------------------"

# ネットワークインターフェース
#INTERFACE="enp4s0"
INTERFACE="eth0"
#IP設定(avia)
LIDAR_IP="192.168.1.50"

echo "[INFO] Setting IP ${LIDAR_IP} on interface ${INTERFACE}..."
sudo ifconfig $INTERFACE $LIDAR_IP up

# ptpd（Precision Time Protocol デーモン）を起動
echo "[INFO] Starting PTP daemon (ptpd)..."
sudo ptpd -M -i $INTERFACE

echo "[INFO] LIDAR setup complete."
