#!/bin/bash

# 获取系统的CMake安装目录
echo "正在获取系统的CMake安装目录..."
install_prefix=$(cmake --system-information | grep "CMAKE_INSTALL_PREFIX" | head -n1 | awk '{print $2}' | tr -d '"')
echo "CMake安装目录为: ${install_prefix}"

# 删除RedoxiTrack的动态链接库文件
echo "正在删除动态链接库文件..."
if sudo rm -fv ${install_prefix}/lib/libRedoxiTrack.so*; then
    echo "成功删除动态链接库文件"
else
    echo "删除动态链接库文件失败"
fi

# 删除RedoxiTrack的头文件目录
echo "正在删除头文件目录..."
if sudo rm -rfv ${install_prefix}/include/RedoxiTrack; then
    echo "成功删除头文件目录"
else
    echo "删除头文件目录失败"
fi

# 删除RedoxiTrack的CMake配置文件
echo "正在删除CMake配置文件..."
if sudo rm -fv ${install_prefix}/lib/cmake/RedoxiTrack/RedoxiTrackConfig*.cmake; then
    echo "成功删除CMake配置文件"
else
    echo "删除CMake配置文件失败"
fi

if sudo rm -fv ${install_prefix}/lib/cmake/RedoxiTrack/RedoxiTrackTargets*.cmake; then
    echo "成功删除CMake目标文件"
else
    echo "删除CMake目标文件失败"
fi

# 删除RedoxiTrack的pkg-config配置文件
echo "正在删除pkg-config配置文件..."
if sudo rm -fv ${install_prefix}/lib/pkgconfig/RedoxiTrack.pc; then
    echo "成功删除pkg-config配置文件"
else
    echo "删除pkg-config配置文件失败"
fi

echo "----------------------------------------"
echo "RedoxiTrack系统安装文件清理完成"
echo "清理目录: ${install_prefix}"
echo "----------------------------------------"
