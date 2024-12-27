import json
import os


def get_tp_fp(count_match_res_save_path, event_type="DoorIn"):
    tp_num = 0
    fp_num = 0
    with open(count_match_res_save_path, "r") as fr:
        content = json.load(fr)
        for key, value in content.items():
            if event_type in key:
                if value:
                    tp_num += 1
                else:
                    fp_num += 1
    return tp_num, fp_num


def get_fn(gt_match_res_save_path, event_type="DoorIn"):
    tp_num = 0
    fn_num = 0
    with open(gt_match_res_save_path, "r") as fr:
        content = json.load(fr)
        for key, value in content.items():
            if event_type in key:
                if value:
                    tp_num += 1
                else:
                    fn_num += 1
    return tp_num, fn_num


# passenger_data_root_path = "/mnt/lc-data/THREED/songfangjing/works/data/passengerflow/flow_test_data"
if __name__ == "__main__":
    dataset_name = "flow_short_test_data_v2_ddq-0604-short-crop-offline"
    exp_name = "ddq-0604-short-crop-offline"
    min_event_num = 0
    result_data_root_path = os.path.join(
        "/mnt/ms-3d/shidebo/data/flow/result/", dataset_name
    )
    result_sub_dirs = [
        item
        for item in os.listdir(result_data_root_path)
        if exp_name in item and "negative" not in item
    ]
    # result_sub_dirs = ["cx--20.22.8.71-2023-11-21-18-38-28--pose-yolov6-bytetrack-crop", "cx--20.22.8.111-2023-11-22-19-00-03--pose-yolov6-bytetrack-crop"]
    event_types = ["DoorIn", "DoorSpeedIn", "DoorOut", "DoorSpeedOut"]
    mk_result_path = "./result/{}-{}-{}-min-event-num-{}.md".format(
        dataset_name, exp_name, "-".join(event_types), min_event_num
    )
    json_result_path = "./result/{}-{}-{}-min-event-num-{}.json".format(
        dataset_name, exp_name, "-".join(event_types), min_event_num
    )

    videoes_info = {}
    sum_tp = 0
    sum_fp = 0
    sum_fn = 0

    for result_dir in result_sub_dirs:
        result_dir_path = os.path.join(result_data_root_path, result_dir)
        match_res_dir_path = os.path.join(result_dir_path, "match_result")
        if not os.path.exists(match_res_dir_path):
            continue
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
        # gt_match_res_save_path = "/mnt/lc-data/THREED/songfangjing/works/result/stereo_test_in_sjz/cx-20.22.6.237-depth-predict-yolov6-bytetrack/gt_events_match_only_event.json"
        # count_match_res_save_path = "/mnt/lc-data/THREED/songfangjing/works/result/stereo_test_in_sjz/cx-20.22.6.237-depth-predict-yolov6-bytetrack/count_events_match_only_event.json"
        # match_res_save_path = "/mnt/lc-data/THREED/songfangjing/works/result/stereo_test_in_sjz/cx-20.22.6.237-depth-predict-yolov6-bytetrack/gt_match_count_only_event.json"

        video_name = result_dir.split("--")[1]
        for event_type in event_types:
            print("event type: ", event_type)
            tp, fp = get_tp_fp(count_match_res_save_path, event_type=event_type)
            print("tp, fp: ", tp, fp)
            tp, fn = get_fn(gt_match_res_save_path, event_type=event_type)
            print("tp, fn: ", tp, fn)
            print("gt num: tp+fn ", tp + fn)

            # precision = float(tp)/(tp+fp)
            # print('precision: ', precision)

            # recall = float(tp)/(tp+fn)
            # print('recall: ', recall)
            ip_name = video_name.split("-")[0]
            if ip_name not in videoes_info.keys():
                videoes_info[ip_name] = {
                    "precision": -1,
                    "recall": -1,
                    "f1_score": -1,
                    "relative_error": -1,
                    "tp": 0,
                    "fp": 0,
                    "fn": 0,
                }
            videoes_info[ip_name]["tp"] += tp
            videoes_info[ip_name]["fp"] += fp
            videoes_info[ip_name]["fn"] += fn
            sum_tp += tp
            sum_fp += fp
            sum_fn += fn

    sum_video_precision = 0
    sum_video_recall = 0
    sum_video_f1_score = 0
    sum_video_rel_error = 0
    ip_set = set()
    video_num_precision = 0
    video_num_recall = 0
    video_num_f1 = 0
    video_num_relative_error = 0
    videoes_info = dict(sorted(videoes_info.items()))
    for ip_name, video_info in videoes_info.items():
        print(ip_name)
        video_event_type_tp = video_info["tp"]
        video_event_type_fp = video_info["fp"]
        video_event_type_fn = video_info["fn"]
        if video_event_type_tp + video_event_type_fp > 0:
            video_info["precision"] = float(video_event_type_tp) / (
                video_event_type_tp + video_event_type_fp
            )
            sum_video_precision += video_info["precision"]
            video_num_precision += 1
        if video_event_type_tp + video_event_type_fn > 0:
            video_info["recall"] = float(video_event_type_tp) / (
                video_event_type_tp + video_event_type_fn
            )
            video_info["relative_error"] = abs(
                float(video_event_type_fp - video_event_type_fn)
                / (video_event_type_tp + video_event_type_fn)
            )
            sum_video_rel_error += video_info["relative_error"]
            sum_video_recall += video_info["recall"]
            video_num_recall += 1
            video_num_relative_error += 1
        if 2 * video_event_type_tp + video_event_type_fp + video_event_type_fn > 0:
            video_info["f1_score"] = (
                2
                * float(video_event_type_tp)
                / (2 * video_event_type_tp + video_event_type_fp + video_event_type_fn)
            )
            sum_video_f1_score += video_info["f1_score"]
            video_num_f1 += 1

        # video_ip = video_name.split('-')[0]
        ip_set.add(ip_name)

    # video_num = len(videoes_info.keys())
    avg_precision = sum_video_precision / float(video_num_precision)
    avg_recall = sum_video_recall / float(video_num_recall)
    avg_f1_score = sum_video_f1_score / float(video_num_f1)
    avg_relative_error = sum_video_rel_error / float(video_num_relative_error)
    # event_type_info = {}
    # for event_type in event_types:
    with open(json_result_path, "w") as fw:
        json.dump(videoes_info, fw, indent=4)

    with open(mk_result_path, "w") as fw:
        fw.write(
            "# dataset: {} exp-name: {} event types {} \n".format(
                dataset_name, exp_name, "-".join(event_types)
            )
        )
        fw.write(
            "video_ip_num: {} videoes: {} \n\n".format(
                len(ip_set), len(result_sub_dirs)
            )
        )
        fw.write(
            "| event_type | tp | fp | fn | presicision | recall | f1_score| relative_error|\n"
        )
        fw.write("| --- |--- | --- | --- | ---| --- | --- |---|\n")

        event_types_precision = float(sum_tp) / (sum_tp + sum_fp)
        event_types_recall = float(sum_tp) / (sum_tp + sum_fn)
        event_types_f1_score = 2 * float(sum_tp) / (2 * sum_tp + sum_fp + sum_fn)
        event_types_relative_error = float(sum_fp - sum_fn) / (sum_tp + sum_fn)
        fw.write(
            "| {} | {} | {} | {} | {}| {} |{} |{}|\n".format(
                "-".join(event_types),
                sum_tp,
                sum_fp,
                sum_fn,
                event_types_precision,
                event_types_recall,
                event_types_f1_score,
                event_types_relative_error,
            )
        )
        fw.write(
            "| {} | {} | {} | {} | {}| {} | {}|{}|\n".format(
                "video_avg",
                sum_tp,
                sum_fp,
                sum_fn,
                avg_precision,
                avg_recall,
                avg_f1_score,
                avg_relative_error,
            )
        )

        fw.write("\n## video  {}\n".format("-".join(event_types)))
        fw.write(
            "| ip_name | tp | fp | fn | presicision | recall | f1_score| relative_error|\n"
        )
        fw.write("| --- |--- | --- | --- | ---| --- |---|---|\n")
        sum_video_precision = 0
        sum_video_recall = 0
        for ip_name, video_info in videoes_info.items():
            video_event_type_tp = video_info["tp"]
            video_event_type_fp = video_info["fp"]
            video_event_type_fn = video_info["fn"]
            fw.write(
                "| {} | {} | {} | {} | {}| {} | {} | {} |\n".format(
                    ip_name,
                    video_event_type_tp,
                    video_event_type_fp,
                    video_event_type_fn,
                    video_info["precision"],
                    video_info["recall"],
                    video_info["f1_score"],
                    video_info["relative_error"],
                )
            )
    print("save to : ", mk_result_path)
