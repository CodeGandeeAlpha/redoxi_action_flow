#!/bin/bash

# 从 SSH_CONNECTION 环境变量中提取 IP 地址
SSH_IP=$(echo $SSH_CONNECTION | awk '{print $1}')

# 检查是否成功提取 IP 地址
if [ -z "$SSH_IP" ]; then
  echo "无法从 SSH_CONNECTION 中提取 IP 地址"
  exit 1
fi

# 设置 DISPLAY 环境变量
export DISPLAY=$SSH_IP:0

# 输出设置结果
echo "DISPLAY 环境变量已设置为: $DISPLAY"