import vineyard
import timeit

# 连接到 Vineyard 客户端
client = vineyard.connect('/var/run/vineyard.sock')

# 定义需要测试的函数
def test_code():
    shared_array = client.get('o01224aacb80afcaa')

# 使用 timeit.Timer 测试函数的执行时间
timer = timeit.Timer(test_code)
execution_time = timer.timeit(number=1000)

print(f"代码执行时间: {execution_time} 秒")
