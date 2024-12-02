#!/bin/bash

ScriptDir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# download test model
# ModelUrl="https://github.com/igamenovoer/test_models/raw/refs/heads/main/yolov8n-pose-640.onnx"
ModelUrl="https://github.com/igamenovoer/test_models/raw/refs/heads/main/yolov8n-pose-dynbatch.onnx"
ModelDstDir="${ScriptDir}/../tmp/models"

mkdir -p ${ModelDstDir}
wget --progress=bar:force ${ModelUrl} -O ${ModelDstDir}/yolov8n-pose-dynbatch.onnx
