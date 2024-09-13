import re
from collections import defaultdict

# 日志文件路径
log_file_path = '/home/chengxiao/.ros/log/2024-09-12-09-18-07-418558-06dfefabfabe-99700/launch.log'

# 正则表达式匹配---TIME LOG行
time_log_pattern = re.compile(
    r'\[INFO\] \[\d+\.\d+\] \[(?P<node>[^\]]+)\]: ---TIME LOG: framenum (?P<frame_num>\d+) node (?P<node_name>[^\s]+) type (?P<type>[^\s]+) time (?P<time>\d+)'
)

# 存储结果的字典
time_logs = defaultdict(lambda: defaultdict(lambda: [None, None]))

# 解析日志文件
with open(log_file_path, 'r') as log_file:
    for line in log_file:
        match = time_log_pattern.search(line)
        if match:
            node_name = match.group('node_name')
            frame_num = int(match.group('frame_num'))
            log_type = match.group('type')
            time = int(match.group('time'))

            if log_type == 'IN':
                time_logs[node_name][frame_num][0] = time
            elif log_type == 'OUT':
                time_logs[node_name][frame_num][1] = time


# 计算每个节点每帧的时间
node_time_dict = defaultdict(lambda: defaultdict(float))
for node, frames in time_logs.items():
    for frame_num, times in frames.items():
        if times[0] is not None and times[1] is not None:
            node_time_dict[node][frame_num] = (times[1] - times[0]) / 1e6

# 计算每个节点的平均时间
node_avg_time = {}
for node, frame_times in node_time_dict.items():
    node_avg_time[node] = sum(frame_times.values()) / len(frame_times)

# # 计算每一帧的平均时间
# total_frame_count = len(node_time_dict['psg_collector'])
# max_frame_num = node_time_dict['psg_collector'].keys()
# total_frame_time = time_logs['psg_collector'][total_frame_count]


# 打印结果
# for node, frames in time_logs.items():
#     print(f'Node: {node}')
#     for frame_num, times in frames.items():
#         print(f'  Frame {frame_num}: IN_time={times[0]}, OUT_time={times[1]}')


for node, avg_time in node_avg_time.items():
    print(f'Node: {node}, Avg_time: {avg_time}')