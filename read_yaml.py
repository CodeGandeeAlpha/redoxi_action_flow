#!/usr/bin/env python3

import yaml


def read_yaml(file_path):
    """
    读取yaml文件

    Args:
        file_path: yaml文件路径

    Returns:
        读取的yaml内容
    """
    try:
        with open(file_path, "r") as f:
            data = yaml.safe_load(f)
            return data
    except Exception as e:
        print(f"读取yaml文件时发生错误: {e}")
        return None


if __name__ == "__main__":
    yaml_path = "test.yaml"
    data = read_yaml(yaml_path)

    if data:
        print("yaml内容:")
        print(data)
