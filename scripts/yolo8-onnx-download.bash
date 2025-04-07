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
    "https://github.com/igamenovoer/test_models/raw/refs/heads/main/yolov8n-pose-640.onnx"
    "https://github.com/igamenovoer/test_models/raw/refs/heads/main/yolov8s.onnx.gz.001"
    "https://github.com/igamenovoer/test_models/raw/refs/heads/main/yolov8s.onnx.gz.002"
)

ModelDstDir="${ScriptDir}/../tmp/models"
mkdir -p "${ModelDstDir}"

for model_url in "${model_urls[@]}"; do
    model_name=$(basename "${model_url%\?*}")
    echo "Downloading ${model_name}"
    wget --progress=bar:force "${model_url}" -O "${ModelDstDir}/${model_name}"
done

# merge yolov8s.onnx.gz.001 and yolov8s.onnx.gz.002 into yolov8s.onnx
echo "Merging yolov8s.onnx.gz.001 and yolov8s.onnx.gz.002 into yolov8s.onnx"
cat "${ModelDstDir}/yolov8s.onnx.gz.001" "${ModelDstDir}/yolov8s.onnx.gz.002" > "${ModelDstDir}/yolov8s.onnx.gz"
gunzip -f "${ModelDstDir}/yolov8s.onnx.gz"

# remove yolov8s.onnx.gz.001 and yolov8s.onnx.gz.002
rm "${ModelDstDir}/yolov8s.onnx.gz.001" "${ModelDstDir}/yolov8s.onnx.gz.002"

