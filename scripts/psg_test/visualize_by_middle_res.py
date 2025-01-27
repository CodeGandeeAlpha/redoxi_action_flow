#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import json
import os
import glob
from dataclasses import dataclass
from typing import List, Dict, Tuple
import argparse


def compute_extrinsic(floor_para):
    floor_param_in_camera = floor_para

    A, B, C, D = floor_param_in_camera
    point_origin_in_floor = np.array([0, 0, -D / C])
    point_a_in_floor = np.array([0, -D / B, 0])
    z_axis = np.array([A, B, C])
    x_axis = point_a_in_floor - point_origin_in_floor
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    T_c_p = np.identity(4)
    T_c_p[:3, 0] = x_axis
    T_c_p[:3, 1] = y_axis
    T_c_p[:3, 2] = z_axis
    T_c_p[:3, 3] = point_origin_in_floor
    return T_c_p


def get_door_zone(info, intrinsic_3x3, T_gc):
    door_pts = np.array(info["points"][:4]).reshape((-1, 2))
    door_pts = np.append(door_pts, [[1]] * door_pts.shape[0], axis=1)
    un_xyz = np.linalg.inv(intrinsic_3x3).dot(door_pts.T)
    # un_xyz = np.append(un_xyz, [[1]*un_xyz.shape[1]], axis=0)
    # door_pts_in_ground = T_gc.dot(un_xyz)
    temp_pts = T_gc[:3, :3].dot(un_xyz)
    scale_pts = -T_gc[2, 3] / temp_pts[2, :]  # point on ground pt
    pts_in_camera = scale_pts * un_xyz
    temp_pts_in_camera = np.append(
        pts_in_camera, [[1] * pts_in_camera.shape[1]], axis=0
    )
    pts_in_world = T_gc.dot(temp_pts_in_camera)
    print(pts_in_world)
    door_line = pts_in_world[:3, 0] - pts_in_world[:3, 1]
    door_vertical_line = np.array([-door_line[1], door_line[0], 0])
    door_vertical_line = door_vertical_line / np.linalg.norm(door_vertical_line)
    pt0 = pts_in_world[:3, 0] + door_vertical_line * 1.0
    pt1 = pts_in_world[:3, 1] + door_vertical_line * 1.0
    pt2 = pts_in_world[:3, 1] - door_vertical_line * 1.0
    pt3 = pts_in_world[:3, 0] - door_vertical_line * 1.0

    pt4 = pts_in_world[:3, 0] + door_vertical_line * 0.3
    pt5 = pts_in_world[:3, 1] + door_vertical_line * 0.3
    pt6 = pts_in_world[:3, 1] - door_vertical_line * 0.3
    pt7 = pts_in_world[:3, 0] - door_vertical_line * 0.3

    pt8 = pts_in_world[:3, 0]
    pt9 = pts_in_world[:3, 1]
    zone_in_world = np.stack([pt0, pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8, pt9], axis=0)
    zone_in_world = np.append(
        zone_in_world, [[1]] * zone_in_world.shape[0], axis=1
    )  # n * 4
    zone_in_camera = np.linalg.inv(T_gc).dot(zone_in_world.T)  # 4 * n
    zone_in_camera = zone_in_camera[:3, :]
    zone_in_camera /= zone_in_camera[2, :]
    zone_in_image = intrinsic_3x3.dot(zone_in_camera)  # 3 * n
    return zone_in_image[:2, :].T


def draw_rect(image, pts, scale_w=1.0, scale_h=1.0):
    for i in range(4):
        if i == 3:
            cv2.line(
                image,
                (int(pts[i][0] * scale_w), int(pts[i][1] * scale_h)),
                (int(pts[0][0] * scale_w), int(pts[0][1] * scale_h)),
                (0, 0, 255),
                2,
            )
            break
        cv2.line(
            image,
            (int(pts[i][0] * scale_w), int(pts[i][1] * scale_h)),
            (int(pts[i + 1][0] * scale_w), int(pts[i + 1][1] * scale_h)),
            (0, 0, 255),
            2,
        )


def draw_door(image, door_info, scale_w=1.0, scale_h=1.0):
    draw_rect(image, door_info[:4][:], scale_w, scale_h)
    draw_rect(image, door_info[4:8][:], scale_w, scale_h)
    cv2.line(
        image,
        (int(door_info[8][0] * scale_w), int(door_info[8][1] * scale_h)),
        (int(door_info[9][0] * scale_w), int(door_info[9][1] * scale_h)),
        (0, 0, 255),
        2,
    )


def draw_door_infos(image, door_infos, scale_w=1.0, scale_h=1.0):
    for door_info in door_infos:
        draw_door(image, door_info, scale_w, scale_h)


def get_door_infos(config_data):
    door_infos = []
    region_infos = config_data["region_infos"]
    intrinsic_3x3 = np.identity(3)
    T_wc = np.identity(4)
    intrinsic_3x3[0, 0] = config_data["camera_fx"]
    intrinsic_3x3[1, 1] = config_data["camera_fy"]
    intrinsic_3x3[0, 2] = config_data["camera_ux"]
    intrinsic_3x3[1, 2] = config_data["camera_uy"]
    T_wc = np.array(config_data["camera_extrinsic_inv"]).reshape((4, 4))
    for region_info in region_infos:
        if region_info["region_type"] == 0:  # type door
            door_info = get_door_zone(region_info, intrinsic_3x3, T_wc)
            door_info = door_info.tolist()
            door_infos.append(door_info)
    return door_infos


def convert_config_to_flow_config(rule_json_path, para_json_path, h, w):
    with open(rule_json_path, "r") as fr:
        rule_content = json.load(fr)
    with open(para_json_path, "r") as fr:
        para_content = json.load(fr)
    json_content = {}

    video_path = ""
    json_content["video_path"] = video_path
    json_content["image_height"] = h
    json_content["image_width"] = w
    json_content["camera_fx"] = para_content["camera_mat"][0]
    json_content["camera_fy"] = para_content["camera_mat"][1]
    json_content["camera_ux"] = para_content["camera_mat"][2]
    json_content["camera_uy"] = para_content["camera_mat"][3]
    json_content["ground_to_world"] = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]

    T_c_p = compute_extrinsic(para_content["floor_para"])
    print(T_c_p)
    T_p_c = np.linalg.inv(T_c_p)
    T_p_c[0, 3] = 0
    T_p_c[1, 3] = 0
    T_p_c = T_p_c.reshape((-1,))
    # print(T_p_c)
    json_content["camera_extrinsic_inv"] = T_p_c.tolist()

    # to do multi door lines
    door_points = rule_content["stores"][0]["rules"][0]["door"]
    door_in_point = rule_content["stores"][0]["rules"][0]["entry"]["d"][:2]
    region_points = door_points + door_in_point

    region_infos = []
    door_info = {
        "certain_region_size": 0.7,
        "likely_region_size": 0.3,
        "points": region_points,
        "region_name": "door0",
        "region_type": 0,
    }
    disappear_info = {
        "points": region_points,
        "region_name": "disappear0",
        "region_type": 2,
        "region_size": 1.0,
    }
    region_infos.append(door_info)
    region_infos.append(disappear_info)
    json_content["region_infos"] = region_infos
    return json_content


def convert_raw_config_to_flow_config(
    rule_json_path, para_json_path, save_path, h=1080, w=1920
):
    assert os.path.exists(rule_json_path) and os.path.exists(para_json_path)

    with open(rule_json_path, "r") as fr:
        rule_content = json.load(fr)
    with open(para_json_path, "r") as fr:
        para_content = json.load(fr)

    # save_path = os.path.join(save_root_path, "{}.json".format(ip_str))
    json_content = {}

    video_path = ""
    json_content["video_path"] = video_path
    json_content["image_height"] = h
    json_content["image_width"] = w
    json_content["camera_fx"] = para_content["camera_mat"][0]
    json_content["camera_fy"] = para_content["camera_mat"][1]
    json_content["camera_ux"] = para_content["camera_mat"][2]
    json_content["camera_uy"] = para_content["camera_mat"][3]
    json_content["ground_to_world"] = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    T_c_p = compute_extrinsic(para_content["floor_para"])
    T_p_c = np.linalg.inv(T_c_p)
    T_p_c[0, 3] = 0
    T_p_c[1, 3] = 0
    T_p_c = T_p_c.reshape((-1,))
    # print(T_p_c)
    json_content["camera_extrinsic_inv"] = T_p_c.tolist()

    # to do multi door lines
    region_infos = []
    store_num = len(rule_content["stores"])
    for store_i in range(store_num):
        rule_num = len(rule_content["stores"][store_i]["rules"])
        for rule_i in range(rule_num):
            try:
                door_points = rule_content["stores"][store_i]["rules"][rule_i]["door"]
                door_in_point = rule_content["stores"][store_i]["rules"][rule_i][
                    "entry"
                ]["d"][:2]
            except:
                continue
            region_points = door_points + door_in_point

            door_info = {
                "certain_region_size": 0.7,
                "likely_region_size": 0.3,
                "points": region_points,
                "region_name": "door{}-{}".format(store_i, rule_i),
                "region_type": 0,
            }
            disappear_info = {
                "points": region_points,
                "region_name": "disappear{}-{}".format(store_i, rule_i),
                "region_type": 2,
                "region_size": 1.0,
            }
            region_infos.append(door_info)
            region_infos.append(disappear_info)
    if len(region_infos) == 0:
        return False
    json_content["region_infos"] = region_infos
    with open(save_path, "w") as fw:
        json.dump(json_content, fw, indent=4)
    return True


def transform_plane(trans, plane):
    a, b, c, d = plane
    new_a, new_b, new_c = trans[:3, :3].dot(np.array([a, b, c]))
    point_in_plane = np.array([0, 0, -d / c, 1])
    new_point = trans.dot(point_in_plane)
    new_d = -1 * (new_a * new_point[0] + new_b * new_point[1] + new_c * new_point[2])
    return [new_a, new_b, new_c, new_d]


@dataclass
class Event:
    id: int
    zone_name: str
    event_type: str
    start_time: int
    end_time: int
    event_info: str
    event_pattern: str
    speed: Tuple[float, float]


class PSGVisualizer:
    def __init__(self, data_dir: str, psg_config: str):
        self.data_dir = data_dir
        self.persons_dir = os.path.join(data_dir, "persons")
        self.event_file = os.path.join(data_dir, "id_event.txt")
        self.events = self._load_events()
        self.event_counts = {}  # 事件类型到计数的映射
        self.psg_config_data = self._load_psg_config(psg_config)
        self.door_infos = get_door_infos(self.psg_config_data)

        # 颜色定义
        self.colors = {
            "normal": (0, 255, 0),  # 绿色
            "active": (0, 0, 255),  # 红色
            "keypoint": (255, 0, 0),  # 蓝色
            "text": (255, 255, 255),  # 白色
        }

    def _load_psg_config(self, psg_config: str):
        with open(psg_config, "r") as f:
            return json.load(f)

    def _load_events(self) -> List[Event]:
        events = []
        with open(self.event_file, "r") as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) < 18:
                    continue
                event = Event(
                    id=int(parts[1]),
                    zone_name=parts[3],
                    event_type=parts[6],
                    start_time=int(parts[8]),
                    end_time=int(parts[10]),
                    event_info=parts[12],
                    event_pattern=parts[14],
                    speed=(float(parts[16]), float(parts[17])),
                )
                if event.end_time - event.start_time > 2 * 25:
                    event.start_time = (
                        event.end_time - 2 * 25 if event.end_time > 2 * 25 else 0
                    )
                events.append(event)
        return events

    def _load_frame_data(self, frame_idx: int) -> List[dict]:
        frame_file = os.path.join(self.persons_dir, f"frame_{frame_idx}.json")
        if not os.path.exists(frame_file):
            return []
        with open(frame_file, "r") as f:
            return json.load(f)

    def _is_person_in_event(self, person_id: int, frame_idx: int) -> bool:
        for event in self.events:
            if (
                event.id == person_id
                and event.start_time <= frame_idx <= event.end_time
            ):
                return True
        return False

    def _get_color(self, id: int) -> Tuple[int, int, int]:
        idx = id * 3
        color = ((37 * idx) % 255, (17 * idx) % 255, (29 * idx) % 255)
        return color

    def _draw_keypoints(
        self,
        image: np.ndarray,
        keypoints: List[float],
        color: Tuple[int, int, int],
        scale_w: float = 1.0,
        scale_h: float = 1.0,
    ):
        # 定义骨骼连接关系
        skeleton = [
            [15, 13],
            [13, 11],
            [16, 14],
            [14, 12],
            [11, 12],
            [5, 11],
            [6, 12],
            [5, 6],
            [5, 7],
            [6, 8],
            [7, 9],
            [8, 10],
            [1, 2],
            [0, 1],
            [0, 2],
            [1, 3],
            [2, 4],
            [3, 5],
            [4, 6],
        ]

        # 绘制关键点
        points = []
        for i in range(0, len(keypoints), 2):
            x, y = int(keypoints[i] * scale_w), int(keypoints[i + 1] * scale_h)
            points.append((x, y))
            cv2.circle(image, (x, y), 3, color, -1)

        # 绘制骨骼连接线
        for pair in skeleton:
            if pair[0] >= len(points) or pair[1] >= len(points):
                continue
            pt1 = points[pair[0]]
            pt2 = points[pair[1]]
            cv2.line(image, pt1, pt2, color, 2)

    def _draw_person(
        self,
        image: np.ndarray,
        person: dict,
        frame_idx: int,
        scale_w: float = 1.0,
        scale_h: float = 1.0,
    ):
        # 跳过没跟踪到的人
        if "id" not in person or person["id"] == 0:
            return
        # 绘制人体框
        if "body" not in person:
            return
        x, y, w, h = map(int, person["body"])
        x = int(x * scale_w)
        y = int(y * scale_h)
        w = int(w * scale_w)
        h = int(h * scale_h)
        color = self._get_color(person["id"])
        # 如果是活跃事件中的人，画椭圆
        if self._is_person_in_event(person["id"], frame_idx):
            cv2.ellipse(
                image,
                (x + w // 2, y + h // 2),
                (w // 2, h // 2),
                0,
                0,
                360,
                (0, 0, 255),
                6,
            )
        else:
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)

        # 绘制人头框
        if "head" in person:
            hx, hy, hw, hh = map(int, person["head"])
            hx = int(hx * scale_w)
            hy = int(hy * scale_h)
            hw = int(hw * scale_w)
            hh = int(hh * scale_h)
            cv2.rectangle(image, (hx, hy), (hx + hw, hy + hh), color, 2)

        # 绘制关键点
        if "body_keypoints" in person:
            self._draw_keypoints(
                image,
                person["body_keypoints"],
                color,
                scale_w,
                scale_h,
            )

        # 绘制ID
        if "id" in person:
            cv2.putText(
                image,
                f"ID: {person['id']}",
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
            )

    def _update_event_counts(self, frame_idx: int):
        for event in self.events:
            if event.end_time == frame_idx:
                self.event_counts[event.event_type] = (
                    self.event_counts.get(event.event_type, 0) + 1
                )

    def _draw_event_counts(self, image: np.ndarray):
        y_offset = 50  # 从上方留出一些空间开始
        for event_type, count in self.event_counts.items():
            # 显示更详细的事件信息
            text = f"Event Type: {event_type} | Count: {count}"
            cv2.putText(
                image,
                text,
                (30, y_offset),  # 左上角位置
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,  # 更大的字体
                (0, 0, 255),  # 红色
                3,  # 更粗的线条
            )
            y_offset += 50  # 增大行间距

    def process_video(self, input_video: str, output_video: str):
        cap = cv2.VideoCapture(input_video)
        if not cap.isOpened():
            raise ValueError("Could not open input video")

        # 获取视频信息
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(cap.get(cv2.CAP_PROP_FPS))
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

        scale_w = width / 1920
        scale_h = height / 1080

        # 创建视频写入器
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        out = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

        frame_idx = 0
        while cap.isOpened():
            print(f"frame_idx: {frame_idx} / {frame_count} \r", end="")
            ret, frame = cap.read()
            if not ret:
                break

            # 加载当前帧的人员数据
            persons = self._load_frame_data(frame_idx)

            # 更新事件计数
            self._update_event_counts(frame_idx)

            # 绘制每个人的信息
            for person in persons:
                self._draw_person(frame, person, frame_idx, scale_w, scale_h)

            # 绘制事件计数
            self._draw_event_counts(frame)

            # 绘制门框
            draw_door_infos(frame, self.door_infos, scale_w, scale_h)

            # 写入帧
            out.write(frame)
            frame_idx += 1

        cap.release()
        out.release()


def main():
    parser = argparse.ArgumentParser(description="PSG Visualization Tool")
    parser.add_argument(
        "--data_dir",
        type=str,
        required=True,
        help="Directory containing persons/ and id_event.txt",
    )
    parser.add_argument(
        "--input_video", type=str, required=True, help="Path to input video file"
    )
    parser.add_argument(
        "--output_video", type=str, required=True, help="Path to output video file"
    )
    parser.add_argument(
        "--psg_config", type=str, required=True, help="Path to psg config file"
    )

    args = parser.parse_args()

    visualizer = PSGVisualizer(args.data_dir, args.psg_config)
    visualizer.process_video(args.input_video, args.output_video)


if __name__ == "__main__":
    main()
