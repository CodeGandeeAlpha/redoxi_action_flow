import numpy as np
from copy import deepcopy
import cv2
import json
import os

def __get_up_pt_by_plane_root(point_on_plane: np.ndarray, plane: np.ndarray, dist: float, inv_flag: bool = False,
                                        camera_ct_pt: np.ndarray = np.zeros(3)):
    t = dist * plane[2] / np.sqrt(np.power(plane[0], 2) + np.power(plane[1], 2) + np.power(plane[2], 2))
    zp = point_on_plane[2] + t
    zn = point_on_plane[2] - t

    xp = (zp - point_on_plane[2]) * plane[0] / plane[2] + point_on_plane[0]
    yp = (zp - point_on_plane[2]) * plane[1] / plane[2] + point_on_plane[1]

    xn = (zn - point_on_plane[2]) * plane[0] / plane[2] + point_on_plane[0]
    yn = (zn - point_on_plane[2]) * plane[1] / plane[2] + point_on_plane[1]

    pt_p = np.array([xp, yp, zp], dtype=np.float64)
    pt_n = np.array([xn, yn, zn], dtype=np.float64)

    camera_side = __get_plane_side_of_pt(camera_ct_pt, plane)
    pside = __get_plane_side_of_pt(pt_p, plane)
    nside = __get_plane_side_of_pt(pt_n, plane)

    if pside == camera_side:
        return pt_p if inv_flag is False else pt_n
    else:
        return pt_n if inv_flag is False else pt_p


def __get_plane_side_of_pt(pt: np.ndarray, plane: np.ndarray):
    r = pt[0] * plane[0] + pt[1] * plane[1] + pt[2] * plane[2] + plane[3]

    if r > 0:
        return 1
    elif r == 0:
        return 0
    else:
        return -1


def __get_plane_by_3pts(pt1: np.ndarray, pt2: np.ndarray, pt3: np.ndarray):
    v1 = np.zeros(3)
    v1[0] = pt1[0] - pt2[0]
    v1[1] = pt1[1] - pt2[1]
    v1[2] = pt1[2] - pt2[2]

    v2 = np.zeros(3)
    v2[0] = pt1[0] - pt3[0]
    v2[1] = pt1[1] - pt3[1]
    v2[2] = pt1[2] - pt3[2]

    nv = np.cross(v1, v2)
    return __get_plane_by_pt_and_nv(pt1, nv)


def __get_dist_of_2pts(pt1: np.ndarray, pt2: np.ndarray):
    return np.sqrt(np.power(pt1[0] - pt2[0], 2) + np.power(pt1[1] - pt2[1], 2) + np.power(pt1[2] - pt2[2], 2))


def __get_plane_by_pt_and_nv(pt: np.ndarray, nv: np.ndarray):
    plane = np.zeros(4)
    plane[0] = nv[0]
    plane[1] = nv[1]
    plane[2] = nv[2]
    plane[3] = -1.0 * (plane[0] * pt[0] + plane[1] * pt[1] + plane[2] * pt[2])

    return plane


def __get_pt_of_plane_by_pt_and_pv(pt: np.ndarray, pv: np.ndarray, plane: np.ndarray):
    xp, yp, zp = pt
    A, B, C, D = plane
    a, b, c = A, B, C
    zx, zy, zl = 0.0, 0.0, 0.0
    if a * b * c != 0:
        zl = -1.0 * c * (A * xp + B * yp + C * zp + D) / (A * a + B * b + C * c) + zp
        xl = a / c * (zl - zp) + xp
        yl = b / c * (zl - zp) + yp
    elif c == 0:
        zl = zp
        yl = -1.0 * b * (A * xp + B * yp + C * zp + D) / (A * a + B * b) + yp
        xl = a / b * (yl - yp) + xp
    elif a == 0:
        xl = xp
        yl = -1.0 * b * (A * xp + B * yp + C * zp + D) / (B * b + C * c) + yp
        zl = c / b * (yl - yp) + zp
    elif b == 0:
        yl = yp
        xl = -1.0 * b * (A * xp + B * yp + C * zp + D) / (A * a + C * c) + xp
        zl = c / a * (xl - xp) + zp
    pt_in_camera = np.zeros(3, dtype=np.float64)
    pt_in_camera[0] = xl
    pt_in_camera[1] = yl
    pt_in_camera[2] = zl

    return pt_in_camera

def gen_local_extrinsic_matrix(projection_matrix: np.ndarray, local_ground: np.ndarray, image_width: float, image_height: float):
    # got world_zero_pt_in_camera
    uv = np.array([image_width / 2, image_height / 2], dtype=np.float64)
    half_line = np.linalg.inv(projection_matrix) @ np.append(uv, 1)
    z = -1.0 * local_ground[3] / (half_line[0] * local_ground[0] + half_line[1] * local_ground[1] + local_ground[2])

    center_local_ground_pt = half_line * z
    local_zero_pt_in_camera = deepcopy(center_local_ground_pt)

    # got x y z axis
    xpt_in_center_line = local_zero_pt_in_camera * 0.9
    xpt_in_camera = np.zeros(3)
    ypt_in_camera = np.zeros(3)
    zpt_in_camera = np.zeros(3)
    if local_ground[0] == 0 and local_ground[1] == 0:  # local ground is parallel to image
        xpt_in_camera = local_zero_pt_in_camera + np.array([1, 0, 0])
        ypt_in_camera = local_zero_pt_in_camera + np.array([0, 1, 0])
        zpt_in_camera = local_zero_pt_in_camera + np.array([0, 0, 1])
    else:
        # got x axis, len = 1
        xpt_in_camera = __get_pt_of_plane_by_pt_and_pv(xpt_in_center_line, local_ground[:-1], local_ground)

        # got z axis, len = 1
        zpt_in_camera = __get_up_pt_by_plane_root(local_zero_pt_in_camera, local_ground, 1.0)

        # got y axis, len = 1
        zx_plane_in_camera = __get_plane_by_3pts(xpt_in_camera, zpt_in_camera, local_zero_pt_in_camera)
        ypt_in_camera = __get_up_pt_by_plane_root(local_zero_pt_in_camera, zx_plane_in_camera, 1.0, camera_ct_pt=np.array([1, 0, 0]))

    # local_extrinsic_matrix
    # R matrix
    local_extrinsic_matrix = np.eye(4)
    len_x = __get_dist_of_2pts(xpt_in_camera, local_zero_pt_in_camera)
    local_extrinsic_matrix[0][0] = (xpt_in_camera[0] - local_zero_pt_in_camera[0]) / len_x
    local_extrinsic_matrix[1][0] = (xpt_in_camera[1] - local_zero_pt_in_camera[1]) / len_x
    local_extrinsic_matrix[2][0] = (xpt_in_camera[2] - local_zero_pt_in_camera[2]) / len_x

    len_y = __get_dist_of_2pts(ypt_in_camera, local_zero_pt_in_camera)
    local_extrinsic_matrix[0][1] = (ypt_in_camera[0] - local_zero_pt_in_camera[0]) / len_y
    local_extrinsic_matrix[1][1] = (ypt_in_camera[1] - local_zero_pt_in_camera[1]) / len_y
    local_extrinsic_matrix[2][1] = (ypt_in_camera[2] - local_zero_pt_in_camera[2]) / len_y

    len_z = __get_dist_of_2pts(zpt_in_camera, local_zero_pt_in_camera)
    local_extrinsic_matrix[0][2] = (zpt_in_camera[0] - local_zero_pt_in_camera[0]) / len_z
    local_extrinsic_matrix[1][2] = (zpt_in_camera[1] - local_zero_pt_in_camera[1]) / len_z
    local_extrinsic_matrix[2][2] = (zpt_in_camera[2] - local_zero_pt_in_camera[2]) / len_z

    # T matrix
    local_extrinsic_matrix[0][3] = local_zero_pt_in_camera[0]
    local_extrinsic_matrix[1][3] = local_zero_pt_in_camera[1]
    local_extrinsic_matrix[2][3] = local_zero_pt_in_camera[2]

    return local_extrinsic_matrix


def on_mouse_get_roi(event, x, y, flags, param):
    global region_pt, image, region_pts
    if event == cv2.EVENT_LBUTTONDOWN:
        point1 = [x, y]
        print(point1)
        region_pt = point1
        region_pts.append(region_pt)
        cv2.circle(image, (x,y),2,(0,0,255),2)
        cv2.imshow('scene', image)
    elif event == cv2.EVENT_MBUTTONDOWN:
        region_pt = []
        region_pts = []
        cv2.imshow('scene', image)

def gen_region_infos(video_path, image_width, image_height, door_pts):
    cap = cv2.VideoCapture(video_path)
    global image, region_pt, region_pts
    region_pt =[]
    region_pts = []
    while True:
        ret, frame = cap.read()
        if ret:
            image = frame
            image = cv2.resize(frame, (image_width, image_height))
            break
    cv2.namedWindow('scene', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('scene', on_mouse_get_roi)
    region_infos = []
    for door_index, door_info in door_pts.items():
        region_info = {}
        region_info["certain_region_size"] = 0.7
        region_info["likely_region_size"] = 0.3
        region_info["points"] = []
        door_pt_1 = list(map(int, door_info[0]))
        door_pt_2 = list(map(int, door_info[1]))
        cv2.circle(image, door_pt_1, 2, (0,0,255), 2)
        cv2.circle(image, door_pt_2, 2, (0,0,255), 2)
        cv2.line(image, door_pt_1,door_pt_2,(0,0,255),2)
        cv2.putText(image, str(door_index), door_pt_1, cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2)
        region_info["points"].extend(door_pt_1)
        region_info["points"].extend(door_pt_2)
        region_info["region_name"] = "door"+str(door_index)
        region_info["region_type"] = 0 #door 0; passing 1; disappear 2
        region_infos.append(region_info)

        disappear_region_info = {}
        disappear_region_info["points"] = region_info["points"][:]
        disappear_region_info['region_name'] = "disappear"+str(door_index)
        disappear_region_info['region_type'] = 2
        disappear_region_info["region_size"] = 0.3
        region_infos.append(disappear_region_info)

    cv2.imshow('scene', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    for i, door_in_pt in enumerate(region_pts):
        region_infos[i*2]["points"].extend(door_in_pt)
        region_infos[i*2+1]["points"].extend(door_in_pt)
    return region_infos

def get_door_infos(area_info):
    door_pts = {}
    for door_index, store_info in enumerate(area_info['stores']):
        door_pt = store_info['rules'][0]['door']
        door_pts[door_index] = []
        door_pts[door_index].append(door_pt[:2])
        door_pts[door_index].append(door_pt[2:])
    return door_pts


def convert_config(video_path, param_config_path, rule_config_path, save_path, image_size=(1920, 1080)):
    image_w, image_h = image_size

    with open(param_config_path, 'r', encoding='UTF-8') as f:
        config = json.load(f)
    camera_mat = config['camera_mat']
    floor_param = config['floor_para']
    distortion = config['distortion']

    with open(rule_config_path, 'r', encoding='UTF-8') as f:
        rule_config = json.load(f)
    door_pts = get_door_infos(rule_config)

    # [camera_mat[0], 0, camera_mat[2]]
    # [0, camera_mat[1], camera_mat[3]]
    # [0, 0, 1]
    projection_matrix = np.eye(3)
    projection_matrix[0][0] = camera_mat[0]
    projection_matrix[1][1] = camera_mat[1]
    projection_matrix[0][2] = camera_mat[2]
    projection_matrix[1][2] = camera_mat[3]

    local_extrinsic_matrix = gen_local_extrinsic_matrix(projection_matrix, np.array(floor_param), image_w, image_h) # Twc

    # construct passenger_flow_config
    passenger_flow_config = {}
    passenger_flow_config['video_path'] = video_path
    passenger_flow_config['image_height'] = image_h
    passenger_flow_config['image_width'] = image_w

    passenger_flow_config['camera_fx'] = camera_mat[0]
    passenger_flow_config['camera_fy'] = camera_mat[1]
    passenger_flow_config['camera_ux'] = camera_mat[2]
    passenger_flow_config['camera_uy'] = camera_mat[3]
    passenger_flow_config['ground_to_world'] = [
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    ]

    passenger_flow_config['camera_extrinsic_inv'] = np.linalg.inv(local_extrinsic_matrix).reshape(-1).tolist() # Tcw


    region_infos = gen_region_infos(video_path, image_w, image_h, door_pts)
    passenger_flow_config['region_infos'] = region_infos

    save_dir = os.path.dirname(save_path)
    os.makedirs(save_dir, exist_ok=True)
    with open(save_path, 'w') as f:
        json.dump(passenger_flow_config, f, indent=4)


if __name__ == '__main__':
    video_path = '/mnt/chengxiao/20.22.6.30-2023-06-18-15-00-02_27908_28658-00.00.00.000-00.00.06.926.mp4'
    config_path = '/3d/chengxiao/data/passengerflow/fairmot_train_230907/configs/device-config-2024-01-19/20.22.6.30_para_config.json'
    rule_config_path = '/3d/chengxiao/data/passengerflow/fairmot_train_230907/configs/device-config-2024-01-19/20.22.6.30_rule.json'
    save_path = '/3d/chengxiao/data/passengerflow/fairmot_train_230907/psg_configs/20.22.6.30.json'
    convert_config(video_path, config_path, rule_config_path, save_path)