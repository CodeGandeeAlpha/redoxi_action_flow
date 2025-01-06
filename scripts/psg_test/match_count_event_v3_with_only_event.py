import os
import xml.etree.ElementTree as ET

# from utils import read_xml
import json
import numpy as np
from scipy.optimize import linear_sum_assignment
import csv
import sys
import pandas as pd


def get_path(dir_path, key_name):
    files_name = [item for item in os.listdir(dir_path) if key_name in item]
    if files_name:
        return os.path.join(dir_path, files_name[0])
    return None


def parse_line(line):
    line = line.strip().split(" ")
    id = int(line[1])
    door_area = line[3]
    event_type = line[6]
    start_time = int(line[8])
    end_time = int(line[10])
    return id, door_area, event_type, start_time, end_time


def get_ids_events_info(path, event_types=["DoorIn"]):
    id_event_infos = []  # list: (id, door_area, event_type, start_time, end_time)
    with open(path, "r") as fr:
        lines = fr.readlines()
        for line in lines[::-1]:
            if "json" in line:
                break
            id, door_area, event_type, start_time, end_time = parse_line(line)
            if event_type not in event_types:
                continue
            # door_area = door_area.replace("disappear", "door")
            door_area = "door0"
            id_event_infos.append(
                (id, door_area, event_type, int(start_time), int(end_time))
            )
    return id_event_infos


def get_id_traj(traj_path):
    id_traj = {}
    with open(traj_path, "r") as fr:
        one_person_traj = json.load(fr)
        for frame_person_info in one_person_traj:
            if "body" in frame_person_info.keys():
                body_box = frame_person_info["body"]
                body_box = (
                    body_box[0],
                    body_box[1],
                    body_box[0] + body_box[2],
                    body_box[1] + body_box[3],
                )
                frame_num = frame_person_info["frameNum"]
                id_traj[frame_num] = body_box
    return id_traj


def get_id_event_traj(traj, start_frame_num, end_frame_num):
    id_event_traj = {}
    for frame_num in range(start_frame_num, end_frame_num + 1):
        if frame_num in traj.keys():
            id_event_traj[frame_num] = traj[frame_num]
    return id_event_traj


def get_event_trajs(id_event_infos, trajs_dir_path, event_types=["DoorIn"]):
    event_trajs = {}
    id_trajs = {}
    not_found_ids = []
    for id_event_info in id_event_infos:
        id, door_area, event_type, start_time, end_time = id_event_info
        if event_type in event_types:
            id_traj = None
            if id in id_trajs.keys():
                id_traj = id_trajs[id]
            else:
                id_traj_path = os.path.join(trajs_dir_path, "person_{}.json".format(id))
                if os.path.exists(id_traj_path):
                    id_traj = get_id_traj(id_traj_path)  # 获取id的2d bbox轨迹
                    id_trajs[id] = id_traj
                else:
                    not_found_ids.append(id)
                    continue
            id_event_traj = get_id_event_traj(
                id_traj, start_time, end_time
            )  # 获取id的2d bbox轨迹在start_time到end_time之间的轨迹
            id_event_info_key = "{}_{}_{}_{}_{}".format(*id_event_info)
            event_trajs[id_event_info_key] = id_event_traj
    return event_trajs, not_found_ids


# def get_gt_trajs(xml_dir_path, flow_image_size):
#     gt_trajs = {}
#     xml_index = sorted([int(item[:-4]) for item in os.listdir(xml_dir_path)])

#     for frame_num in xml_index:
#         xml_path = os.path.join(xml_dir_path, "{}.xml".format(frame_num))
#         gt_image_size, object_infos = read_xml(xml_path)
#         for object in object_infos:
#             if object["type"] == "person":
#                 person_id = object["id"]
#                 if person_id not in gt_trajs.keys():
#                     gt_trajs[person_id] = []
#                     gt_trajs[person_id].append(
#                         {"start_frame": frame_num, "end_frame": frame_num}
#                     )
#                 else:
#                     last_id_traj = gt_trajs[person_id][-1]
#                     if frame_num > last_id_traj["end_frame"] + 1:
#                         gt_trajs[person_id].append(
#                             {"start_frame": frame_num, "end_frame": frame_num}
#                         )

#                 scale = float(flow_image_size[0]) / float(gt_image_size[0])
#                 bbox = [item * scale for item in object["bbox"]]
#                 gt_trajs[person_id][-1][frame_num] = bbox
#                 gt_trajs[person_id][-1]["end_frame"] = frame_num
#     return gt_trajs


def compute_iou(bbox_a, bbox_b):
    xa = max(bbox_a[0], bbox_b[0])
    ya = max(bbox_a[1], bbox_b[1])
    xb = min(bbox_a[2], bbox_b[2])
    yb = min(bbox_a[3], bbox_b[3])

    inter_area = max(0, xb - xa + 1) * max(0, yb - ya + 1)

    bbox_a_area = (bbox_a[2] - bbox_a[0] + 1) * (bbox_a[3] - bbox_a[1] + 1)
    bbox_b_area = (bbox_b[2] - bbox_b[0] + 1) * (bbox_b[3] - bbox_b[1] + 1)

    iou = inter_area / float(bbox_a_area + bbox_b_area - inter_area)
    return iou


def compute_cost(traj_a, traj_b, start_frame, end_frame):
    num = 0
    sum_cost = 0
    for frame_num in range(start_frame, end_frame + 1):
        if frame_num in traj_a.keys() and frame_num in traj_b.keys():
            temp_cost = 1 - compute_iou(traj_a[frame_num], traj_b[frame_num])
            num += 1
            sum_cost += temp_cost
    return sum_cost / num


def match_event_trajs_with_gt_trajs(event_trajs, gt_trajs, iou_cost_thresh=0.5):
    gt_trajs_keys = list(gt_trajs.keys())
    event_trajs_keys = list(event_trajs.keys())
    total_gt_events = {}
    total_count_events = {}
    total_match_events = {}
    cost = np.zeros((len(gt_trajs_keys), len(event_trajs_keys)))
    for col_index, event_traj_info in enumerate(event_trajs_keys):
        event_id, door_area, event_type, event_start_time, event_end_time = (
            event_traj_info.split("_")
        )
        event_start_time = int(event_start_time)
        event_end_time = int(event_end_time)
        event_traj = event_trajs[event_traj_info]
        total_count_events[event_traj_info] = None
        for row_index, gt_traj_id in enumerate(gt_trajs_keys):
            total_gt_events[gt_traj_id] = None
            gt_traj = gt_trajs[gt_traj_id]
            gt_start_frame = gt_traj["start_frame"]
            gt_end_frame = gt_traj["end_frame"]
            if gt_end_frame < event_start_time or event_end_time < gt_start_frame:
                cost[row_index][col_index] = 9999
                continue
            max_start_frame = max(gt_start_frame, event_start_time)
            min_end_frame = min(gt_end_frame, event_end_time)
            temp_cost = compute_cost(
                gt_traj, event_traj, max_start_frame, min_end_frame
            )
            cost[row_index][col_index] = temp_cost
    row_ind, col_ind = linear_sum_assignment(cost)

    for i, row in enumerate(row_ind):
        col = col_ind[i]
        if cost[row, col] > iou_cost_thresh:
            continue
        gt_event_key = gt_trajs_keys[row]
        track_event_key = event_trajs_keys[col]
        assert gt_event_key not in total_match_events.keys()
        total_match_events[gt_event_key] = track_event_key
        assert total_gt_events[gt_event_key] is None
        total_gt_events[gt_event_key] = track_event_key
        assert total_count_events[track_event_key] is None
        total_count_events[track_event_key] = gt_event_key
    return total_gt_events, total_count_events, total_match_events


def get_match_events(track_infos, gt_infos):
    """
    track_infos = [ (id, door_area, event_type, start_time, end_time), ()]
    gt_infos = [(door, id, frameNum, event_type), ()]  # id is no use, just for hash code
    event_type(in: "DoorIn“; out: "DoorOut")

    return:
    total_gt_events, total_tracker_events, total_match_events
    {(door, gt_id, frameNum, eventType):(door, track_id, frameNum, eventType), (door, gt_id, frameNum, eventType):None}
    """
    total_gt_events, total_tracker_events, total_match_events = {}, {}, {}
    door_types = set()
    for gt_info in gt_infos:
        door_types.add(gt_info[0])
    print(door_types)

    for door_type in door_types:
        gt_info = []
        track_info = []
        for gt in gt_infos:
            if gt[0] == door_type:
                gt_info.append(gt)
        for track in track_infos:
            if track[1] == door_type:
                track_info.append(track)

        cost = np.zeros((len(gt_info), len(track_info)))
        for row, gt_event in enumerate(gt_info):
            gt_event_key = "{}_{}_{}_{}".format(*gt_event)
            total_gt_events[gt_event_key] = None
            for col, track_event in enumerate(track_info):

                track_event_key = "{}_{}_{}_{}_{}".format(*track_event)

                total_tracker_events[track_event_key] = None
                gt_frame = int(gt_event[2])
                track_end_time = int(track_event[4])
                gt_in_or_out, track_in_or_out = gt_event[3], track_event[2]
                temp_cost = 9999
                if abs(gt_frame - track_end_time) > 1000:
                    temp_cost = 9999
                else:
                    if ("In" in gt_in_or_out and "In" in track_in_or_out) or (
                        "Out" in gt_in_or_out and "Out" in track_in_or_out
                    ):
                        temp_cost = abs(gt_frame - track_end_time)
                    else:
                        temp_cost = 9999
                cost[row, col] = temp_cost
        row_ind, col_ind = linear_sum_assignment(cost)
        for i, row in enumerate(row_ind):
            col = col_ind[i]
            if cost[row, col] == 9999:
                continue
            gt_event = gt_info[row]
            track_event = track_info[col]
            gt_event_key = "{}_{}_{}_{}".format(*gt_event)
            track_event_key = "{}_{}_{}_{}_{}".format(*track_event)
            assert gt_event not in total_match_events.keys()
            total_match_events[gt_event_key] = track_event
            assert total_gt_events[gt_event_key] is None
            total_gt_events[gt_event_key] = track_event
            # assert total_tracker_events[track_event_key] is None  ## May exist duplicate records in track_infos
            total_tracker_events[track_event_key] = gt_event
    return total_gt_events, total_tracker_events, total_match_events


def save_json(path, content):
    with open(path, "w") as fw:
        json.dump(content, fw, indent=4)


def get_gt_event_infos(csv_path, door_num=1):
    total_gt_infos = []
    gt_id = 0
    with open(csv_path, "r") as fr:
        csv_content = csv.reader(fr)
        for index, row in enumerate(csv_content):
            print(row)
            if index == 0:
                # door_num = int((len(row) - 1) / 2)
                # door_num = 1
                continue
            frame = int(row[0])
            for i in range(door_num):
                area_in, area_out = row[i * 2 + 1 : i * 2 + 3]
                area_in, area_out = int(float(area_in)), int(float(area_out))
                if area_in == 0 and area_out == 0:
                    continue

                for _ in range(area_in):
                    gt_id += 1
                    gt_event = ("door{}".format(i), gt_id, frame, "DoorIn")
                    total_gt_infos.append(gt_event)
                for _ in range(area_out):
                    gt_id += 1
                    gt_event = ("door{}".format(i), gt_id, frame, "DoorOut")
                    total_gt_infos.append(gt_event)
    return total_gt_infos


def get_gt_event_infos_by_pandas(csv_path, door_num=1):
    total_gt_infos = []
    gt_id = 0
    # with open(csv_path, 'r') as fr:
    df = pd.read_csv(csv_path)
    # csv_content = csv.reader(fr)
    # for index, row in enumerate(csv_content):
    for index, row in df.iterrows():
        # print(row)
        # print(row[:3])
        # if index == 0:
        #     # door_num = int((len(row) - 1) / 2)
        #     # door_num = 1
        #     continue
        frame = int(row.iloc[0])
        for i in range(door_num):
            area_in, area_out = row[i * 2 + 1 : i * 2 + 3]
            area_in, area_out = int(float(area_in)), int(float(area_out))
            # print('data', frame, area_in, area_out)
            if area_in == 0 and area_out == 0:
                continue

            for _ in range(area_in):
                gt_id += 1
                gt_event = ("door{}".format(i), gt_id, frame, "DoorIn")
                total_gt_infos.append(gt_event)
            for _ in range(area_out):
                gt_id += 1
                gt_event = ("door{}".format(i), gt_id, frame, "DoorOut")
                total_gt_infos.append(gt_event)
    return total_gt_infos


exp_name = "ddq-0604-short-crop-offline"
dataset_name = "flow_short_test_data_v2"
result_dir_name = "flow_short_test_data_v2_ddq-0604-short-crop-offline"
# exp_name = "pose-fairmot-crop"
passenger_data_root_path = "/mnt/ms-3d/shidebo/dataset/passengerflow/{}".format(
    dataset_name
)
result_data_root_path = "/mnt/ms-3d/shidebo/data/flow/result/{}/".format(
    result_dir_name
)

result_sub_dirs = [
    item for item in os.listdir(result_data_root_path) if exp_name in item
]
# result_sub_dirs = ["cx--20.22.12.212-2023-11-30-17-00-02--pose-ddq-bytetrack-crop","cx--20.22.9.143-2023-11-21-19-35-36--pose-ddq-bytetrack-crop",
#                    "cx--20.22.9.39-2023-11-22-20-00-04--pose-ddq-bytetrack-crop", "cx--20.22.9.46-2023-11-19-15-00-02--pose-ddq-bytetrack-crop",
#    "cx--20.22.6.139-2023-11-22-12-00-02--pose-ddq-bytetrack-crop","cx--20.22.6.111-2023-11-20-20-00-06--pose-ddq-bytetrack-crop"]
# result_sub_dirs = ["cx--20.22.6.97-2023-11-18-20-00-05--pose-ddq-bytetrack-crop"]
print("numbers: ", len(result_sub_dirs))


event_types = ["DoorIn", "DoorOut", "DoorSpeedIn", "DoorSpeedOut"]
flow_image_size = (1920, 1080)
cnt_not_found_ids = 0

for result_dir in result_sub_dirs:
    result_dir_path = os.path.join(result_data_root_path, result_dir)
    print("process: ", result_dir_path)
    id_event_info_path = os.path.join(result_dir_path, "id_event.txt")
    trajs_dir_path = os.path.join(result_dir_path, "trajs")

    video_name = result_dir.split("--")[1]
    raw_data_dir_path = os.path.join(passenger_data_root_path, video_name)
    csv_path = get_path(raw_data_dir_path, "count")

    match_res_dir_path = os.path.join(result_dir_path, "match_result")
    if not os.path.exists(match_res_dir_path):
        os.mkdir(match_res_dir_path)
    event_trajs_save_path = os.path.join(
        match_res_dir_path, "event_count_trajs_only_event.json"
    )
    gt_match_res_save_path = os.path.join(
        match_res_dir_path, "gt_events_match_only_event.json"
    )
    count_match_res_save_path = os.path.join(
        match_res_dir_path, "count_events_match_only_event.json"
    )
    match_res_save_path = os.path.join(
        match_res_dir_path, "gt_match_count_only_event.json"
    )

    id_event_infos = get_ids_events_info(id_event_info_path, event_types)
    print(id_event_infos)
    id_event_trajs, not_found_ids = get_event_trajs(
        id_event_infos, trajs_dir_path, event_types=event_types
    )
    cnt_not_found_ids += len(not_found_ids)
    print("---------------------------------------------")
    # gt_event_infos = get_gt_event_infos(csv_path)
    gt_event_infos = get_gt_event_infos_by_pandas(csv_path)
    print(gt_event_infos)

    gt_events_match, count_events_match, match_events = get_match_events(
        id_event_infos, gt_event_infos
    )
    # gt_events_match, count_events_match, match_events = match_event_trajs_with_gt_trajs(id_event_trajs, gt_trajs,iou_cost_thresh)
    print(gt_events_match)
    print("-------------------")
    print(count_events_match)
    print("-------------------------")
    print(match_events)

    # save_json(gt_trajs_save_path, gt_trajs)
    save_json(event_trajs_save_path, id_event_trajs)
    save_json(gt_match_res_save_path, gt_events_match)
    save_json(count_match_res_save_path, count_events_match)
    save_json(match_res_save_path, match_events)
    # break

print(f"No. not found ids: {cnt_not_found_ids}")
