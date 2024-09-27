import queue
import time
import sys

sys.path.append("src/flow_ros2_pipeline/psg_common")
from psg_common.pub_sub import StreamWorker


# 定义输入函数
def input_function(worker):
    try:
        data = worker.input_queue.get(timeout=worker.input_queue_get_timeout_sec)
        print("input_function get data time: ", time.time())
        return True, data
    except queue.Empty:
        return False, None


# 定义输出函数
def output_function(data, worker):
    try:
        worker.output_queue.put(data, timeout=worker.output_queue_put_timeout_sec)
        print("output_function put data time: ", time.time())
        return True
    except queue.Full:
        return False


# 定义处理函数
def worker_function_one_step(data, worker):
    # 简单地将输入数据加1
    processed_data = data + 1
    print("worker_function_one_step process data time: ", time.time())
    return True, processed_data


# 定义停止回调函数
def on_stop_callback(worker):
    print("Worker has stopped")


# 创建 StreamWorker 实例
worker = StreamWorker(
    input_function=input_function,
    output_function=output_function,
    worker_function_one_step=worker_function_one_step,
    on_stop_callback=on_stop_callback,
    input_queue_get_timeout_sec=1.0,
    output_queue_put_timeout_sec=1.0,
)

# 启动工作线程
worker.start()
print("Worker started time: ", time.time())

# 向输入队列添加数据
for i in range(5):
    worker.input_queue.put(i)
    print("input data time: ", time.time())
    time.sleep(0.5)  # 模拟数据生成的间隔

# 等待处理完成
time.sleep(3)
print("Worker wait stopping time: ", time.time())

# 停止工作线程
worker.stop()
print("Worker stopped time: ", time.time())

# 检查输出队列中的数据
while not worker.output_queue.empty():
    output_data = worker.output_queue.get()
    print(f"Output data: {output_data}")
    print("output data time: ", time.time())
