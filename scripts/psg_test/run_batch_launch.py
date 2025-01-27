#!/usr/bin/env python3

import os
import subprocess
import argparse
from pathlib import Path


def run_launch(video_path, config_path, model_path, save_dir, group_id):
    """
    运行单个launch文件

    Args:
        video_path: 视频文件路径
        config_path: PSG配置文件路径
        save_dir: 结果保存目录
        group_id: 组号
    """
    #! 设置环境变量
    env = os.environ.copy()
    env["ROS_FN_VIDEO"] = video_path
    env["ROS_PSG_CONFIG_PATH"] = config_path
    env["ROS_SAVE_MIDDLE_RESULT_DIR_PATH"] = save_dir
    env["ROS_DOMAIN_ID"] = group_id
    env["ROS_FN_MODEL"] = model_path
    #! 构建launch命令
    cmd = [
        "ros2",
        "launch",
        "test_cx",
        "lch_v2_psg_counter_from_video_person_cppdet_driver.py",
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
    parser.add_argument("--model_path", required=True, help="检测器的onnx模型路径")
    parser.add_argument("--video_ext", default=".mp4", help="视频文件扩展名")
    parser.add_argument("--config_ext", default=".json", help="配置文件扩展名")
    parser.add_argument("--total_groups", type=int, default=1, help="总组数")
    parser.add_argument(
        "--group_id", type=int, default=0, help="当前处理的组号(从0开始)"
    )
    args = parser.parse_args()

    if args.group_id >= args.total_groups:
        print(f"组号 {args.group_id} 超出总组数 {args.total_groups}")
        return

    #! 创建保存目录
    os.makedirs(args.save_root, exist_ok=True)

    data_root_path = Path(args.data_root)
    category = data_root_path.name
    video_dir_name = data_root_path.parent.name

    #! 获取所有视频目录并排序,以确保分组一致性
    video_dirs = sorted([d for d in Path(args.data_root).iterdir() if d.is_dir()])

    #! 计算当前组应处理的目录范围
    group_size = len(video_dirs) // args.total_groups
    start_idx = args.group_id * group_size
    end_idx = (
        start_idx + group_size
        if args.group_id < args.total_groups - 1
        else len(video_dirs)
    )

    #! 只处理当前组的目录
    for video_dir in video_dirs[start_idx:end_idx]:
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
            video_save_dir = (
                Path(args.save_root) / video_dir_name / category / video_dir.name
            )
            os.makedirs(video_save_dir, exist_ok=True)

            #! 运行launch文件
            print(f"\n处理视频: {video}")
            for config in configs:
                #! config以flow_开头
                if not config.stem.startswith("flow_"):
                    continue
                print(f"使用配置: {config}")
                success = run_launch(
                    str(video),
                    str(config),
                    str(args.model_path),
                    str(video_save_dir),
                    str(args.group_id),
                )
                if not success:
                    print(f"处理失败: {video}")


if __name__ == "__main__":
    main()
