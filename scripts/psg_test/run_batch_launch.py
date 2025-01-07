#!/usr/bin/env python3

import os
import subprocess
import yaml
import argparse
from pathlib import Path


def load_config(config_path):
    """
    读取yaml配置文件
    """
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


def run_launch(video_path, config_path, save_dir):
    """
    运行单个launch文件

    Args:
        video_path: 视频文件路径
        config_path: PSG配置文件路径
        save_dir: 结果保存目录
    """
    #! 设置环境变量
    env = os.environ.copy()
    env["ROS_FN_VIDEO"] = video_path
    env["ROS_PSG_CONFIG_PATH"] = config_path
    env["ROS_SAVE_MIDDLE_RESULT_DIR_PATH"] = save_dir

    #! 构建launch命令
    cmd = [
        "ros2",
        "launch",
        "test_cx",
        "lch_v2_psg_counter_from_video_hz_cppdet_driver.py",
    ]

    #! 运行命令
    try:
        subprocess.run(cmd, env=env, check=True)
    except subprocess.CalledProcessError as e:
        print(f"运行失败: {e}")
        return False
    return True


def main():
    parser = argparse.ArgumentParser(description="批量运行PSG计数launch文件")
    parser.add_argument(
        "--data_root", required=True, help="数据根目录,包含视频和配置文件"
    )
    parser.add_argument("--save_root", required=True, help="结果保存根目录")
    parser.add_argument("--video_ext", default=".mp4", help="视频文件扩展名")
    parser.add_argument("--config_ext", default=".json", help="配置文件扩展名")
    args = parser.parse_args()

    #! 创建保存目录
    os.makedirs(args.save_root, exist_ok=True)

    #! 遍历数据目录
    for video_dir in Path(args.data_root).iterdir():
        if not video_dir.is_dir():
            continue

        print(f"\n处理目录: {video_dir}")

        #! 查找视频文件
        videos = list(video_dir.glob(f"*{args.video_ext}"))
        if not videos:
            print(f"未找到视频文件: {video_dir}")
            continue

        #! 查找配置文件
        configs = list(video_dir.glob(f"*{args.config_ext}"))
        if not configs:
            print(f"未找到配置文件: {video_dir}")
            continue

        #! 为每个视频创建保存目录
        for video in videos:
            video_save_dir = Path(args.save_root) / video_dir.name / video.stem
            os.makedirs(video_save_dir, exist_ok=True)

            #! 运行launch文件
            print(f"\n处理视频: {video}")
            for config in configs:
                print(f"使用配置: {config}")
                success = run_launch(
                    str(video), str(config), str(video_save_dir / config.stem)
                )
                if not success:
                    print(f"处理失败: {video}")


if __name__ == "__main__":
    main()
