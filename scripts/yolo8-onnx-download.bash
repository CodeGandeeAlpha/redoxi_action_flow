#!/bin/bash

ScriptDir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# download test model
# ModelUrl="https://github.com/igamenovoer/test_models/raw/refs/heads/main/yolov8n-pose-640.onnx"
# ModelUrl="https://github.com/igamenovoer/test_models/raw/refs/heads/main/yolov8n-pose-dynbatch.onnx"
# ModelDstDir="${ScriptDir}/../tmp/models"

# mkdir -p ${ModelDstDir}
# wget --progress=bar:force ${ModelUrl} -O ${ModelDstDir}/yolov8n-pose-dynbatch.onnx

model_urls=(
    "https://huggingface.co/Xenova/yolov8-pose-onnx/resolve/main/yolov8s-pose.onnx?download=true"
    "https://huggingface.co/Xenova/yolov8-pose-onnx/resolve/main/yolov8n-pose.onnx?download=true"
)

ModelDstDir="${ScriptDir}/../tmp/models"
mkdir -p "${ModelDstDir}"

for model_url in "${model_urls[@]}"; do
    model_name=$(basename "${model_url%\?*}")
    wget --progress=bar:force "${model_url}" -O "${ModelDstDir}/${model_name}"
done
